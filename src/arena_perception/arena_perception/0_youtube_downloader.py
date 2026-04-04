#!/usr/bin/env python3

import os
import re
import yt_dlp
import rclpy
from rclpy.node import Node

# Neue Imports für Scene Detection
from scenedetect import detect, ContentDetector, split_video_ffmpeg


class YouTubeDownloader(Node):
    def __init__(self):
        super().__init__('youtube_downloader')
        
        self.declare_parameter('url', '')
        self.declare_parameter('start_time', '')
        self.declare_parameter('end_time', '')
        self.declare_parameter('quality', '720')
        self.declare_parameter('format', 'mp4')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('threshold', 27.0)
        self.declare_parameter('skip_splitting', False)
        
        self.url = self.get_parameter('url').value
        self.start_time = self.get_parameter('start_time').value
        self.end_time = self.get_parameter('end_time').value
        self.quality = self.get_parameter('quality').value
        self.format = self.get_parameter('format').value
        self.output_dir = self.get_parameter('output_dir').value
        self.threshold = self.get_parameter('threshold').value
        self.skip_splitting = self.get_parameter('skip_splitting').value
        
        if not self.url:
            self.get_logger().error("Parameter 'url' is required")
            self._should_exit = True
            return
            
        if not self.start_time or not self.end_time:
            self.get_logger().error("Parameters 'start_time' and 'end_time' are required")
            self._should_exit = True
            return
        
        if not self.output_dir:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            if 'build' in script_dir:
                workspace_root = os.path.normpath(os.path.join(script_dir, '..', '..', '..'))
                self.output_dir = os.path.join(workspace_root, 'src', 'arena_perception', 'videos')
            else:
                package_root = os.path.dirname(script_dir)
                self.output_dir = os.path.join(package_root, 'videos')
        
        os.makedirs(self.output_dir, exist_ok=True)
        
        self._should_exit = False
        self.download_and_process_video()
        self._should_exit = True

    def time_to_seconds(self, time_str: str) -> int:
        parts = time_str.split(':')
        if len(parts) == 3:
            hours, minutes, seconds = map(int, parts)
            return hours * 3600 + minutes * 60 + seconds
        elif len(parts) == 2:
            minutes, seconds = map(int, parts)
            return minutes * 60 + seconds
        else:
            return int(parts[0])

    def get_next_number(self) -> int:
        """Gibt die nächste freie Nummer im Ordner zurück."""
        existing_files = [f for f in os.listdir(self.output_dir) if f.endswith(f'.{self.format}')]
        max_num = -1
        for filename in existing_files:
            match = re.match(r'(\d+)\.' + re.escape(self.format), filename)
            if match:
                num = int(match.group(1))
                max_num = max(max_num, num)
        return max_num + 1

    def download_and_process_video(self):
        try:
            start_seconds = self.time_to_seconds(self.start_time)
            end_seconds = self.time_to_seconds(self.end_time)
            
            if end_seconds <= start_seconds:
                self.get_logger().error("End time must be greater than start time")
                return
            
            # 1. Download in eine temporäre Datei
            temp_filename = os.path.join(self.output_dir, f"temp_download_{os.getpid()}.{self.format}")
            
            ydl_opts = {
                'format': f'bestvideo[height<={self.quality}]+bestaudio/best[height<={self.quality}]',
                'download_ranges': lambda _, __: [{'start_time': start_seconds, 'end_time': end_seconds}],
                'merge_output_format': self.format,
                'outtmpl': temp_filename,
                'quiet': True,
                'no_warnings': True,
            }
            
            self.get_logger().info(f"Downloading segment to temp file...")
            with yt_dlp.YoutubeDL(ydl_opts) as ydl:
                ydl.download([self.url])

            if self.skip_splitting:
                next_num = self.get_next_number()
                final_path = os.path.join(self.output_dir, f"{next_num:04d}.{self.format}")
                os.rename(temp_filename, final_path)
                self.get_logger().info(f"Scene detection skipped. Saved as {final_path}")
            else:
                self.get_logger().info(f"Analyzing scenes (Threshold: {self.threshold})...")
                scene_list = detect(temp_filename, ContentDetector(threshold=self.threshold))
                
                if not scene_list:
                    next_num = self.get_next_number()
                    final_path = os.path.join(self.output_dir, f"{next_num:04d}.{self.format}")
                    os.rename(temp_filename, final_path)
                    self.get_logger().info(f"No cuts detected. Saved as {final_path}")
                else:
                    self.get_logger().info(f"Detected {len(scene_list)} scenes. Splitting...")
                    
                    split_dir = os.path.join(self.output_dir, "temp_splits")
                    os.makedirs(split_dir, exist_ok=True)
                    
                    split_video_ffmpeg(temp_filename, scene_list, output_file_template=os.path.join(split_dir, "scene-$SCENE_NUMBER.mp4"))
                    
                    split_files = sorted([f for f in os.listdir(split_dir) if f.startswith("scene-")])
                    
                    for f in split_files:
                        next_num = self.get_next_number()
                        old_path = os.path.join(split_dir, f)
                        new_path = os.path.join(self.output_dir, f"{next_num:04d}.{self.format}")
                        os.rename(old_path, new_path)
                        self.get_logger().info(f"Saved scene: {new_path}")
                    
                    os.rmdir(split_dir)
                    if os.path.exists(temp_filename):
                        os.remove(temp_filename)

        except Exception as e:
            self.get_logger().error(f"Error during processing: {str(e)}")
            # Cleanup bei Fehler
            if 'temp_filename' in locals() and os.path.exists(temp_filename):
                os.remove(temp_filename)

def main(args=None):
    rclpy.init(args=args)
    node = YouTubeDownloader()
    
    if hasattr(node, '_should_exit') and node._should_exit:
        node.destroy_node()
        rclpy.shutdown()
    else:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()