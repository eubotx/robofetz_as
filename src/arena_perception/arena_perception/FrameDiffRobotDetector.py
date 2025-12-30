import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
import threading
from collections import deque

class OpponentDetectionNode(Node):
    def __init__(self):
        super().__init__('opponent_detection_node')

        # 1. Subscriber für das rectified image
        self.image_sub = self.create_subscription(
            Image,
            '/arena_camera/image_rect',  # Du hast rectified image
            self.image_callback,
            10
        )

        # 2. Bridge für OpenCV
        self.bridge = CvBridge()

        # 3. TF für Koordinatentransformation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4. Publisher für die erkannte Pose
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/detected_robot/pose',
            10
        )

        # 5. Publisher für Debug-Images (optional)
        self.debug_pub = self.create_publisher(
            Image,
            '/debug/detection_image',
            10
        )

        # 6. State-Variablen
        self.background_model = None
        self.background_initialized = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.lock = threading.Lock()

        # 7. Parameter
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('robot_radius_m', 0.15)
        self.declare_parameter('background_learning_frames', 30)
        self.declare_parameter('publish_debug', False)

        # 8. Timer für Hauptverarbeitung
        self.processing_timer = self.create_timer(
            0.033,  # 30 Hz
            self.process_latest_image
        )
        
        self.get_logger().info('FrameDiff Robot Detector gestartet')

    def image_callback(self, msg):
        """
        Schneller Callback: Nur Bild speichern
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            with self.lock:
                self.latest_image = cv_image
                self.latest_image_time = msg.header.stamp
                self.latest_image_frame = msg.header.frame_id

                # Hintergrund initialisieren
                self.update_background(cv_image)

        except Exception as e:
            self.get_logger().error(f'Fehler in image_callback: {e}')

    def update_background(self, cv_image):
        """
        Hintergrundmodell aktualisieren
        """
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if self.background_model is None:
            # Initialisierung mit ersten Frames
            frames_needed = self.get_parameter('background_learning_frames').value

            if not hasattr(self, 'background_frames'):
                self.background_frames = deque(maxlen=frames_needed)
                self.background_model = None

            self.background_frames.append(gray)

            if len(self.background_frames) >= frames_needed:
                # Median-Hintergrund berechnen
                self.background_model = np.median(
                    list(self.background_frames), axis=0
                ).astype(np.uint8)
                self.background_initialized = True
                self.get_logger().info('Hintergrundmodell initialisiert')
        else:
            # Adaptiven Hintergrund aktualisieren
            alpha = 0.05  # Lernrate
            self.background_model = cv2.addWeighted(
                self.background_model, 1 - alpha,
                gray, alpha,
                0
            )

    def detect_by_frame_difference(self, current_frame):
        """
        Frame-Differenz zur Roboter-Detektion
        """
        if not self.background_initialized or self.background_model is None:
            return None, None

        # Zu Grau konvertieren
        gray_current = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # Absolute Differenz
        diff = cv2.absdiff(gray_current, self.background_model)

        # Threshold
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

        # Rauschen reduzieren
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # Konturen finden
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return None, thresh

        # Größte Kontur finden (wahrscheinlich der Roboter)
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        min_area = self.get_parameter('min_contour_area').value
        if area < min_area:
            return None, thresh

        return largest_contour, thresh

    def estimate_pose_from_contour(self, contour):
        """
        Position und Orientierung aus Kontur schätzen
        """
        # 1. Schwerpunkt berechnen
        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None, None

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # 2. Orientierung über Ellipse
        if len(contour) >= 5:
            ellipse = cv2.fitEllipse(contour)
            (x_ell, y_ell), (MA, ma), angle = ellipse

            # Roboter zeigt in Richtung der Hauptachse
            # Winkel anpassen für ROS-Konvention (x-Achse vorwärts)
            orientation_rad = np.deg2rad(angle)

            return (cx, cy), orientation_rad
        else:
            return (cx, cy), 0.0

    def pixel_to_camera_frame(self, pixel_position):
        """
        Transformiert Pixelkoordinaten in Kamerakoordinaten
        Annahme: Roboter steht auf Bodenebene (z=0 im Welt-Koordinatensystem)
        """
        try:
            # 1. Hole Transformation von Kamera zu Welt
            transform = self.tf_buffer.lookup_transform(
                'world',  # Ziel-Frame
                'arena_camera_optical',  # Quell-Frame
                rclpy.time.Time()
            )

            # 2. Für Bodenebene (z=0) brauchen wir die Homographie
            # Vereinfachung: Wenn Kamera senkrecht nach unten schaut
            # Transformiere Pixel zu Kamera-Ray, dann Schnitt mit Boden

            # Alternative: Verwende bekannte Roboterhöhe und Projektion
            # Hier vereinfachte 2D-Transformation

            # Da du die Kamerapose bereits hast, kannst du eine
            # inverse Projektion durchführen

            return self.inverse_projection(pixel_position, transform)

        except TransformException as e:
            self.get_logger().warn(f'TF Transformation fehlgeschlagen: {e}')
            return None

    def inverse_projection(self, pixel_point, camera_transform):
        """
        Inverse Projektion für Bodenebene
        """
        # Hier kommt deine spezifische Implementierung
        # Abhängig von deiner Kameraposition und -orientierung

        # Beispiel für senkrechte Kamera:
        # 1. Pixel zu Normalized Device Coordinates
        # 2. Mit inverser Kameramatrix multiplizieren
        # 3. Mit Kamerapose transformieren
        # 4. Schnitt mit Bodenebene berechnen

        pass

    def simple_pixel_to_world(self, pixel_point):
        """
        Vereinfachte Transformation über bekannte Homographie
        Kalibrierung über 4 Punkte im Bild und deren Welt-Koordinaten
        """
        # Wenn deine Arena rechteckig ist und du 4 Eckpunkte kennst:
        # 1. Kalibriere Homographie-Matrix offline
        # 2. Wende sie hier an

        # Beispiel-Homographie (muss kalibriert werden!)
        H = np.array([
            [0.001, 0.000, -1.5],
            [0.000, 0.001, -1.0],
            [0.000, 0.000, 1.0]
        ])

        # Pixel zu homogenen Koordinaten
        pixel_hom = np.array([pixel_point[0], pixel_point[1], 1.0])

        # Transformation
        world_hom = H @ pixel_hom
        world_hom = world_hom / world_hom[2]

        return (world_hom[0], world_hom[1], 0.0)

    def process_latest_image(self):
        """
        Hauptverarbeitungsschleife - wird vom Timer aufgerufen
        """
        with self.lock:
            if not hasattr(self, 'latest_image') or self.latest_image is None:
                return

            current_frame = self.latest_image.copy()

        # 1. Roboter detektieren
        contour, mask = self.detect_by_frame_difference(current_frame)

        if contour is None:
            return

        # 2. Pose im Bild schätzen
        pixel_position, orientation = self.estimate_pose_from_contour(contour)

        if pixel_position is None:
            return

        # 3. In Weltkoordinaten transformieren
        # Option A: Über TF und inverse Projektion
        world_position = self.pixel_to_camera_frame(pixel_position)

        # Option B: Falls Option A nicht funktioniert, vereinfachte Methode
        if world_position is None:
            world_position = self.simple_pixel_to_world(pixel_position)

        # 4. Pose publizieren
        self.publish_robot_pose(world_position, orientation)

        # 5. Debug-Image publizieren (optional)
        if self.get_parameter('publish_debug').value:
            self.publish_debug_image(current_frame, contour, pixel_position, mask)

    def publish_robot_pose(self, world_position, orientation):
        """
        Publisht die erkannte Roboterpose
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'  # oder 'map', je nach deinem Setup

        pose_msg.pose.position.x = float(world_position[0])
        pose_msg.pose.position.y = float(world_position[1])
        pose_msg.pose.position.z = float(world_position[2])

        # Quaternion aus Yaw berechnen
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, orientation)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_publisher.publish(pose_msg)

        # Debug-Logging
        self.get_logger().debug(
            f'Pose publiziert: ({world_position[0]:.2f}, '
            f'{world_position[1]:.2f}, {orientation:.2f} rad)',
            throttle_duration_sec=1.0
        )

    def publish_debug_image(self, frame, contour, pixel_position, mask):
        """
        Erstellt Debug-Image mit Detektion
        """
        debug_frame = frame.copy()

        # Kontur zeichnen
        cv2.drawContours(debug_frame, [contour], -1, (0, 255, 0), 2)

        # Schwerpunkt markieren
        cv2.circle(debug_frame, pixel_position, 5, (0, 0, 255), -1)

        # Orientierungslinie
        length = 50
        end_x = int(pixel_position[0] + length * np.cos(pixel_position[2]))
        end_y = int(pixel_position[1] + length * np.sin(pixel_position[2]))
        cv2.arrowedLine(debug_frame, pixel_position, (end_x, end_y),
                        (255, 0, 0), 2)

        # Maske als Overlay
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 0] = 0  # Nur grüner Kanal
        debug_frame = cv2.addWeighted(debug_frame, 0.7, mask_colored, 0.3, 0)

        # Text hinzufügen
        cv2.putText(debug_frame, f'Pos: {pixel_position}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Publishen
        debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, "bgr8")
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = 'arena_camera_optical'
        self.debug_pub.publish(debug_msg)

    def main(args=None):
        rclpy.init(args=args)
        node = FrameDiffRobotDetector()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()