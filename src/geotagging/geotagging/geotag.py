#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
from PIL import Image as PILImage
import piexif
import datetime
import os
import threading
import time
import queue
import json
from pathlib import Path

try:
    import openpyxl
    EXCEL_AVAILABLE = True
except ImportError:
    EXCEL_AVAILABLE = False

class GeoTaggingNode(Node):
    def __init__(self):
        super().__init__('geotagging_node')
        
        # Declare parameters with Docker-friendly defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/camera/image_raw'),
                ('gps_topic', '/mavros/global_position/global'),
                ('heading_topic', '/mavros/global_position/compass_hdg'),
                ('detection_trigger_topic', '/detection_trigger'),
                ('mission_start_topic', '/mission_start'),
                ('gps_position_topic', '/gps_position'),
                ('output_base_dir', '/data/geotagged_images'),
                ('excel_file', 'geotag_info.xlsx'),
                ('json_file', 'geotag_info.json'),
                ('continuous_capture', False),
                ('capture_interval', 5.0),
                ('max_images_per_session', 1000),
                ('image_quality', 95),
                ('backup_to_json', True),
                ('create_thumbnails', True),
                ('thumbnail_size', 200),
                ('gps_timeout', 10.0),
                ('min_gps_accuracy', 5.0),  # meters
                ('waypoint_radius', 2.0),   # meters for waypoint-based capture
                ('compress_images', True)
            ]
        )
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value
        self.detection_trigger_topic = self.get_parameter('detection_trigger_topic').value
        self.mission_start_topic = self.get_parameter('mission_start_topic').value
        self.gps_position_topic = self.get_parameter('gps_position_topic').value
        self.output_base_dir = self.get_parameter('output_base_dir').value
        self.excel_file = self.get_parameter('excel_file').value
        self.json_file = self.get_parameter('json_file').value
        self.continuous_capture = self.get_parameter('continuous_capture').value
        self.capture_interval = self.get_parameter('capture_interval').value
        self.max_images_per_session = self.get_parameter('max_images_per_session').value
        self.image_quality = self.get_parameter('image_quality').value
        self.backup_to_json = self.get_parameter('backup_to_json').value
        self.create_thumbnails = self.get_parameter('create_thumbnails').value
        self.thumbnail_size = self.get_parameter('thumbnail_size').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.min_gps_accuracy = self.get_parameter('min_gps_accuracy').value
        self.waypoint_radius = self.get_parameter('waypoint_radius').value
        self.compress_images = self.get_parameter('compress_images').value

        # QoS Profile for MAVROS topics
        self.mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # State variables
        self.latest_gps = None
        self.latest_heading = None
        self.latest_gps_time = None
        self.mission_active = False
        self.capture_enabled = False
        self.image_counter = 0
        self.session_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Thread safety
        self.data_lock = threading.Lock()
        self.capture_queue = queue.Queue(maxsize=50)
        
        # Setup directories
        self.setup_directories()
        
        # Initialize data files
        self.initialize_data_files()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, qos_profile=self.mavros_qos
        )
        
        self.heading_sub = self.create_subscription(
            Float64, self.heading_topic, self.heading_callback, qos_profile=self.mavros_qos
        )
        
        self.trigger_sub = self.create_subscription(
            Bool, self.detection_trigger_topic, self.trigger_callback, 10
        )
        
        self.mission_sub = self.create_subscription(
            Bool, self.mission_start_topic, self.mission_callback, 10
        )
        
        # Optional NTRIP GPS position subscriber
        self.ntrip_gps_sub = self.create_subscription(
            Point, self.gps_position_topic, self.ntrip_gps_callback, 10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/geotagging/status', 10)
        
        # Start worker thread for image processing
        self.processing_thread = threading.Thread(target=self.image_processing_worker, daemon=True)
        self.processing_thread.start()
        
        # Continuous capture timer
        if self.continuous_capture:
            self.capture_timer = self.create_timer(self.capture_interval, self.continuous_capture_callback)
        
        # Status reporting timer
        self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info(f"Geotagging node initialized - Session: {self.session_id}")
        self.get_logger().info(f"Output directory: {self.current_session_dir}")

    def setup_directories(self):
        """Create necessary directories with proper structure"""
        self.current_session_dir = Path(self.output_base_dir) / f"session_{self.session_id}"
        self.images_dir = self.current_session_dir / "images"
        self.thumbnails_dir = self.current_session_dir / "thumbnails" if self.create_thumbnails else None
        self.data_dir = self.current_session_dir / "data"
        
        # Create directories
        self.images_dir.mkdir(parents=True, exist_ok=True)
        if self.thumbnails_dir:
            self.thumbnails_dir.mkdir(parents=True, exist_ok=True)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info(f"Created session directory: {self.current_session_dir}")

    def initialize_data_files(self):
        """Initialize Excel and JSON data files"""
        self.excel_path = self.data_dir / self.excel_file
        self.json_path = self.data_dir / self.json_file
        
        # Initialize Excel file if available
        if EXCEL_AVAILABLE:
            try:
                if not self.excel_path.exists():
                    wb = openpyxl.Workbook()
                    ws = wb.active
                    ws.title = "Geotag Info"
                    headers = [
                        "File Name", "Latitude", "Longitude", "Altitude", 
                        "Heading", "GPS Accuracy", "Timestamp", "UTC Time",
                        "Mission Active", "Trigger Type", "Session ID"
                    ]
                    ws.append(headers)
                    wb.save(str(self.excel_path))
                    self.get_logger().info(f"Excel file created: {self.excel_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to create Excel file: {e}")
        else:
            self.get_logger().warn("OpenPyXL not available - Excel logging disabled")
        
        # Initialize JSON file
        if self.backup_to_json:
            if not self.json_path.exists():
                initial_data = {
                    "session_id": self.session_id,
                    "created": datetime.datetime.now().isoformat(),
                    "images": []
                }
                with open(self.json_path, 'w') as f:
                    json.dump(initial_data, f, indent=2)
                self.get_logger().info(f"JSON file created: {self.json_path}")

    def gps_callback(self, msg: NavSatFix):
        """Update GPS data with accuracy checking"""
        with self.data_lock:
            # Check GPS fix quality
            if hasattr(msg, 'status') and msg.status.status < 0:
                self.get_logger().debug("GPS fix not available")
                return
            
            # Check horizontal accuracy if available
            if hasattr(msg, 'position_covariance') and len(msg.position_covariance) > 0:
                # Covariance diagonal elements give variance
                horizontal_accuracy = (msg.position_covariance[0] + msg.position_covariance[4]) ** 0.5
                if horizontal_accuracy > self.min_gps_accuracy:
                    self.get_logger().debug(f"GPS accuracy too low: {horizontal_accuracy:.1f}m")
                    return
            
            self.latest_gps = {
                'latitude': msg.latitude,
                'longitude': msg.longitude,
                'altitude': msg.altitude,
                'accuracy': getattr(msg, 'position_covariance', [0])[0] ** 0.5 if hasattr(msg, 'position_covariance') else 0.0,
                'status': msg.status.status if hasattr(msg, 'status') else -1
            }
            self.latest_gps_time = time.time()

    def ntrip_gps_callback(self, msg: Point):
        """Update GPS from NTRIP node (higher accuracy)"""
        with self.data_lock:
            self.latest_gps = {
                'latitude': msg.x,
                'longitude': msg.y,
                'altitude': msg.z,
                'accuracy': 0.1,  # RTK accuracy
                'status': 4  # RTK Fixed
            }
            self.latest_gps_time = time.time()

    def heading_callback(self, msg: Float64):
        """Update heading data"""
        with self.data_lock:
            self.latest_heading = msg.data

    def trigger_callback(self, msg: Bool):
        """Handle detection trigger"""
        if msg.data:
            self.capture_enabled = True
            self.get_logger().info("Image capture triggered by detection")

    def mission_callback(self, msg: Bool):
        """Handle mission start/stop"""
        self.mission_active = msg.data
        if self.mission_active:
            self.get_logger().info("Mission started - Geotagging active")
        else:
            self.get_logger().info("Mission stopped - Geotagging paused")
            self.capture_enabled = False

    def continuous_capture_callback(self):
        """Timer callback for continuous capture mode"""
        if self.mission_active:
            self.capture_enabled = True

    def image_callback(self, msg: Image):
        """Queue image for processing"""
        if not self.should_capture_image():
            return
        
        if self.image_counter >= self.max_images_per_session:
            self.get_logger().warn(f"Maximum images ({self.max_images_per_session}) reached for this session")
            return
        
        try:
            # Queue the image data for processing
            image_data = {
                'msg': msg,
                'timestamp': time.time(),
                'gps': self.latest_gps.copy() if self.latest_gps else None,
                'heading': self.latest_heading,
                'mission_active': self.mission_active
            }
            
            self.capture_queue.put_nowait(image_data)
            self.capture_enabled = False  # Reset trigger
            
        except queue.Full:
            self.get_logger().warn("Image processing queue full - dropping frame")

    def should_capture_image(self):
        """Determine if image should be captured"""
        if not (self.capture_enabled or self.continuous_capture):
            return False
        
        if not self.mission_active and not self.continuous_capture:
            return False
        
        # Check if we have recent GPS data
        if not self.latest_gps or not self.latest_gps_time:
            self.get_logger().warn("No GPS data available for geotagging")
            return False
        
        if time.time() - self.latest_gps_time > self.gps_timeout:
            self.get_logger().warn("GPS data too old for geotagging")
            return False
        
        return True

    def image_processing_worker(self):
        """Background thread for image processing"""
        while rclpy.ok():
            try:
                # Get image data with timeout
                image_data = self.capture_queue.get(timeout=1.0)
                self.process_image(image_data)
                self.capture_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Image processing error: {e}")

    def process_image(self, image_data):
        """Process and save geotagged image"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(image_data['msg'], desired_encoding="bgr8")
            
            # Generate filename
            self.image_counter += 1
            timestamp = datetime.datetime.now()
            filename = f"IMG_{self.session_id}_{self.image_counter:04d}.jpg"
            filepath = self.images_dir / filename
            
            # Save image with compression
            if self.compress_images:
                cv2.imwrite(str(filepath), cv_image, [cv2.IMWRITE_JPEG_QUALITY, self.image_quality])
            else:
                cv2.imwrite(str(filepath), cv_image)
            
            # Create thumbnail if enabled
            if self.create_thumbnails:
                self.create_thumbnail(cv_image, filename)
            
            # Add EXIF metadata
            if image_data['gps'] and image_data['heading'] is not None:
                self.add_exif_metadata(
                    str(filepath), 
                    image_data['gps'], 
                    image_data['heading'], 
                    timestamp
                )
            
            # Log to data files
            self.log_image_data(
                filename, 
                image_data['gps'], 
                image_data['heading'], 
                timestamp,
                image_data['mission_active']
            )
            
            self.get_logger().info(f"Saved geotagged image: {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def create_thumbnail(self, cv_image, filename):
        """Create thumbnail image"""
        try:
            # Resize image
            height, width = cv_image.shape[:2]
            if width > height:
                new_width = self.thumbnail_size
                new_height = int(height * self.thumbnail_size / width)
            else:
                new_height = self.thumbnail_size
                new_width = int(width * self.thumbnail_size / height)
            
            thumbnail = cv2.resize(cv_image, (new_width, new_height))
            thumb_filename = f"thumb_{filename}"
            thumb_path = self.thumbnails_dir / thumb_filename
            cv2.imwrite(str(thumb_path), thumbnail, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
        except Exception as e:
            self.get_logger().warn(f"Failed to create thumbnail: {e}")

    def add_exif_metadata(self, image_path, gps_data, heading, timestamp):
        """Add comprehensive EXIF metadata"""
        try:
            def deg_to_dms_rational(deg):
                d = int(abs(deg))
                m = int((abs(deg) - d) * 60)
                s = (abs(deg) - d - m / 60) * 3600
                return ((d, 1), (m, 1), (int(s * 100), 100))

            lat = gps_data['latitude']
            lon = gps_data['longitude']
            alt = gps_data['altitude']

            # Create EXIF dictionary
            exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "thumbnail": None}
            
            # Timestamp
            timestamp_str = timestamp.strftime("%Y:%m:%d %H:%M:%S")
            exif_dict["0th"][piexif.ImageIFD.DateTime] = timestamp_str
            exif_dict["Exif"][piexif.ExifIFD.DateTimeOriginal] = timestamp_str
            exif_dict["Exif"][piexif.ExifIFD.DateTimeDigitized] = timestamp_str
            
            # GPS coordinates
            exif_dict["GPS"][piexif.GPSIFD.GPSLatitude] = deg_to_dms_rational(lat)
            exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef] = "N" if lat >= 0 else "S"
            exif_dict["GPS"][piexif.GPSIFD.GPSLongitude] = deg_to_dms_rational(lon)
            exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef] = "E" if lon >= 0 else "W"
            exif_dict["GPS"][piexif.GPSIFD.GPSAltitude] = (int(abs(alt) * 100), 100)
            exif_dict["GPS"][piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
            
            # GPS timestamp
            utc_time = datetime.datetime.utcnow()
            exif_dict["GPS"][piexif.GPSIFD.GPSTimeStamp] = (
                (utc_time.hour, 1), (utc_time.minute, 1), (utc_time.second, 1)
            )
            exif_dict["GPS"][piexif.GPSIFD.GPSDateStamp] = utc_time.strftime("%Y:%m:%d")
            
            # Heading/direction
            if heading is not None:
                exif_dict["GPS"][piexif.GPSIFD.GPSImgDirection] = (int(abs(heading) * 100), 100)
                exif_dict["GPS"][piexif.GPSIFD.GPSImgDirectionRef] = "T"
            
            # GPS status information
            exif_dict["GPS"][piexif.GPSIFD.GPSStatus] = "A"  # Active
            exif_dict["GPS"][piexif.GPSIFD.GPSMeasureMode] = "3"  # 3D measurement
            
            # Insert EXIF data
            exif_bytes = piexif.dump(exif_dict)
            piexif.insert(exif_bytes, image_path)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to add EXIF metadata: {e}")

    def log_image_data(self, filename, gps_data, heading, timestamp, mission_active):
        """Log image data to Excel and JSON files"""
        try:
            if gps_data:
                lat, lon, alt = gps_data['latitude'], gps_data['longitude'], gps_data['altitude']
                accuracy = gps_data.get('accuracy', 0.0)
                gps_status = gps_data.get('status', -1)
            else:
                lat = lon = alt = accuracy = 0.0
                gps_status = -1
            
            utc_time = datetime.datetime.utcnow().isoformat()
            timestamp_str = timestamp.strftime("%Y-%m-%d %H:%M:%S")
            
            # Excel logging
            if EXCEL_AVAILABLE and self.excel_path.exists():
                try:
                    wb = openpyxl.load_workbook(str(self.excel_path))
                    ws = wb.active
                    ws.append([
                        filename, lat, lon, alt, heading, accuracy,
                        timestamp_str, utc_time, mission_active, 
                        "trigger" if not self.continuous_capture else "continuous",
                        self.session_id
                    ])
                    wb.save(str(self.excel_path))
                except Exception as e:
                    self.get_logger().warn(f"Excel logging failed: {e}")
            
            # JSON logging
            if self.backup_to_json and self.json_path.exists():
                try:
                    with open(self.json_path, 'r') as f:
                        data = json.load(f)
                    
                    image_record = {
                        'filename': filename,
                        'latitude': lat,
                        'longitude': lon,
                        'altitude': alt,
                        'heading': heading,
                        'gps_accuracy': accuracy,
                        'gps_status': gps_status,
                        'timestamp': timestamp_str,
                        'utc_time': utc_time,
                        'mission_active': mission_active,
                        'trigger_type': "trigger" if not self.continuous_capture else "continuous"
                    }
                    
                    data['images'].append(image_record)
                    
                    with open(self.json_path, 'w') as f:
                        json.dump(data, f, indent=2)
                        
                except Exception as e:
                    self.get_logger().warn(f"JSON logging failed: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Data logging failed: {e}")

    def publish_status(self):
        """Publish geotagging status"""
        gps_status = "No GPS"
        if self.latest_gps and self.latest_gps_time:
            age = time.time() - self.latest_gps_time
            accuracy = self.latest_gps.get('accuracy', 0.0)
            gps_status = f"GPS OK (±{accuracy:.1f}m, {age:.1f}s old)"
        
        status = (f"Session: {self.session_id} | Images: {self.image_counter} | "
                 f"Mission: {'Active' if self.mission_active else 'Inactive'} | "
                 f"Capture: {'Enabled' if self.capture_enabled else 'Disabled'} | "
                 f"{gps_status}")
        
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down geotagging node...")
        
        # Wait for queue to empty
        if not self.capture_queue.empty():
            self.get_logger().info("Processing remaining images...")
            time.sleep(2)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GeoTaggingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
