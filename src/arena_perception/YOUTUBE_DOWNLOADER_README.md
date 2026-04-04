# YouTube Video Downloader for ROS 2

A ROS 2 node for downloading specific segments of YouTube videos using yt-dlp. This tool is integrated into the `arena_perception` package for downloading training data videos.

## Features

- Download only specific time ranges from YouTube videos (saves bandwidth and storage)
- **Automatic scene detection and splitting** - detects cuts and splits video into separate files
- Automatic sequential file naming (0000.mp4, 0001.mp4, etc.)
- Configurable video quality (720p, 1080p, etc.)
- Multiple output format support (mp4, webm, mkv, etc.)
- Default output to `arena_perception/videos/` directory
- Progress tracking during download
- Configurable scene detection threshold

## Installation

### Build the Package

```bash
cd /home/dario/Dokumente/projects/robofetz/robofetz_as
colcon build --packages-select arena_perception --symlink-install
source install/local_setup.bash
```

### Install yt-dlp (if not already installed)

```bash
pip install yt-dlp
```

Or use the system package:
```bash
sudo apt install python3-yt-dlp
```

### Install FFmpeg (required for scene splitting)

```bash
sudo apt install ffmpeg
```

## Usage

### Basic Usage

Download a video segment with default settings (720p, mp4 format):

```bash
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=VIDEO_ID" \
  -p start_time:="00:01:30" \
  -p end_time:="00:02:45"
```

### Advanced Usage

#### Specify Video Quality (1080p)

```bash
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=VIDEO_ID" \
  -p start_time:="00:00:10" \
  -p end_time:="00:00:20" \
  -p quality:="1080"
```

#### Specify Output Format (webm)

```bash
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=VIDEO_ID" \
  -p start_time:="00:00:10" \
  -p end_time:="00:00:20" \
  -p format:="webm"
```

#### Custom Output Directory

```bash
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=VIDEO_ID" \
  -p start_time:="00:00:00" \
  -p end_time:="00:00:30" \
  -p output_dir:="/home/user/my_videos"
```

#### Scene Detection with Custom Threshold

The node automatically detects scene changes (cuts) and splits the video into separate files. Adjust the threshold for more or less sensitivity:

```bash
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=VIDEO_ID" \
  -p start_time:="00:00:10" \
  -p end_time:="00:00:40" \
  -p threshold:=27.0
```

Lower threshold = more scenes detected (sensitive to small changes)
Higher threshold = fewer scenes detected (only major cuts)

#### Skip Scene Detection (Download Only)

To download without scene detection and splitting:

```bash
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=VIDEO_ID" \
  -p start_time:="00:00:10" \
  -p end_time:="00:00:30" \
  -p skip_splitting:=true
```

#### All Parameters

```bash
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=dQw4w9WgXcQ" \
  -p start_time:="00:00:15" \
  -p end_time:="00:00:25" \
  -p quality:="1080" \
  -p format:="mp4" \
  -p output_dir:="/custom/output/path" \
  -p threshold:=27.0 \
  -p skip_splitting:=false
```

## Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `url` | string | **Yes** | - | YouTube video URL |
| `start_time` | string | **Yes** | - | Start time in `hh:mm:ss` or `mm:ss` format |
| `end_time` | string | **Yes** | - | End time in `hh:mm:ss` or `mm:ss` format |
| `quality` | string | No | `"720"` | Video quality height in pixels (480, 720, 1080, etc.) |
| `format` | string | No | `"mp4"` | Output video format (mp4, webm, mkv, etc.) |
| `output_dir` | string | No | `arena_perception/videos/` | Custom output directory path |
| `threshold` | double | No | `27.0` | Scene detection sensitivity (lower = more scenes) |
| `skip_splitting` | bool | No | `false` | Skip scene detection, save as single file |

## Time Format Examples

- `00:00:10` = 10 seconds
- `00:01:30` = 1 minute 30 seconds
- `01:15:45` = 1 hour 15 minutes 45 seconds
- `02:30` = 2 minutes 30 seconds (short form)

## Output File Naming

Videos are automatically named with sequential numbering:

- First download: `0000.mp4`
- Second download: `0001.mp4`
- Third download: `0002.mp4`
- And so on...

The script automatically detects existing files in the output directory and increments the counter.

## Default Output Location

By default, videos are saved to:
```
/home/dario/Dokumente/projects/robofetz/robofetz_as/src/arena_perception/videos/
```

## Common Issues

### Video Quality Not Available

If the requested quality is not available, yt-dlp will automatically select the best available quality that is equal to or lower than the specified value.

### FFmpeg Not Found

yt-dlp requires FFmpeg for merging video and audio streams. Install it with:
```bash
sudo apt install ffmpeg
```

### Download Fails

- Check your internet connection
- Verify the YouTube URL is correct and the video is accessible
- Some videos may have download restrictions

## Use Case Example: Training Data Collection

This tool is designed to help collect video segments for training computer vision models. Example workflow:

1. Find relevant YouTube videos with combat robot footage
2. Identify interesting segments (e.g., specific moves or behaviors)
3. Download only those segments to save space
4. Use the downloaded videos with `dataset_create` node to generate YOLO training data

Example:
```bash
# Download 15 seconds of combat footage
ros2 run arena_perception youtube-downloader --ros-args \
  -p url:="https://www.youtube.com/watch?v=EXAMPLE123" \
  -p start_time:="02:30:00" \
  -p end_time:="02:30:15" \
  -p quality:="720"
```

## Integration with Dataset Creation

After downloading videos, use the dataset creation tool:

```bash
ros2 run arena_perception dataset_create --ros-args \
  -p video_folder:=/path/to/videos \
  -p target_fps:=7
```

## Technical Details

- **Backend**: Uses `yt-dlp` library for reliable YouTube downloads
- **Format Selection**: Automatically selects best video+audio streams up to specified quality
- **Time Range**: Downloads only the specified segment using native yt-dlp range support
- **ROS 2 Integration**: Runs as a standard ROS 2 node with parameter support
