#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import piexif
import datetime
from pathlib import Path
import openpyxl

class SimpleGeotaggingNode(Node):
    def __init__(self):
        super().__init__('simple_geotagging_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # State variables
        self.latest_gps = None
        self.latest_heading = None
        self.image_counter = 0
        
        # Setup daily folder
        self.setup_daily_folder()
        
        # Initialize Excel file
        self.setup_excel_file()
        
        # Subscribers - process every image that comes in
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback, 10
        )
        
        self.heading_sub = self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self.heading_callback, 10
        )
        
        self.get_logger().info(f"Simple geotagging node started - saving to {self.daily_folder}")

    def setup_daily_folder(self):
        """Create folder with today's date"""
        today = datetime.date.today().strftime("%Y-%m-%d")
        self.daily_folder = Path(f"/data/geotagged_images/{today}")
        self.daily_folder.mkdir(parents=True, exist_ok=True)

    def setup_excel_file(self):
        """Create Excel file for today's data"""
        today = datetime.date.today().strftime("%Y-%m-%d")
        self.excel_file = self.daily_folder / f"geotagged_images_{today}.xlsx"
        
        if not self.excel_file.exists():
            wb = openpyxl.Workbook()
            ws = wb.active
            ws.title = "Geotagged Images"
            headers = ["Image Name", "Latitude", "Longitude", "Altitude", "Heading", "Timestamp"]
            ws.append(headers)
            wb.save(str(self.excel_file))

    def gps_callback(self, msg: NavSatFix):
        """Store latest GPS data"""
        self.latest_gps = {
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude
        }

    def heading_callback(self, msg: Float64):
        """Store latest heading data"""
        self.latest_heading = msg.data

    def image_callback(self, msg: Image):
        """Process every incoming image - no triggers or conditions"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Generate filename with timestamp
            self.image_counter += 1
            timestamp = datetime.datetime.now()
            filename = f"IMG_{timestamp.strftime('%H%M%S')}_{self.image_counter:04d}.jpg"
            filepath = self.daily_folder / filename
            
            # Save image
            cv2.imwrite(str(filepath), cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            # Add GPS EXIF data (if available)
            if self.latest_gps is not None:
                self.add_gps_exif(str(filepath), timestamp)
            
            # Log to Excel
            self.log_to_excel(filename, timestamp)
            
            self.get_logger().info(f"Saved image: {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def add_gps_exif(self, image_path, timestamp):
        """Add GPS coordinates and heading to EXIF data"""
        try:
            def deg_to_dms_rational(deg):
                d = int(abs(deg))
                m = int((abs(deg) - d) * 60)
                s = (abs(deg) - d - m / 60) * 3600
                return ((d, 1), (m, 1), (int(s * 100), 100))

            lat = self.latest_gps['latitude']
            lon = self.latest_gps['longitude']
            alt = self.latest_gps['altitude']

            # Create EXIF dictionary
            exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "thumbnail": None}
            
            # Add timestamp
            timestamp_str = timestamp.strftime("%Y:%m:%d %H:%M:%S")
            exif_dict["0th"][piexif.ImageIFD.DateTime] = timestamp_str
            
            # Add GPS coordinates
            exif_dict["GPS"][piexif.GPSIFD.GPSLatitude] = deg_to_dms_rational(lat)
            exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef] = "N" if lat >= 0 else "S"
            exif_dict["GPS"][piexif.GPSIFD.GPSLongitude] = deg_to_dms_rational(lon)
            exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef] = "E" if lon >= 0 else "W"
            exif_dict["GPS"][piexif.GPSIFD.GPSAltitude] = (int(abs(alt) * 100), 100)
            exif_dict["GPS"][piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
            
            # Add heading if available
            if self.latest_heading is not None:
                exif_dict["GPS"][piexif.GPSIFD.GPSImgDirection] = (int(abs(self.latest_heading) * 100), 100)
                exif_dict["GPS"][piexif.GPSIFD.GPSImgDirectionRef] = "T"
            
            # Insert EXIF data
            exif_bytes = piexif.dump(exif_dict)
            piexif.insert(exif_bytes, image_path)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to add EXIF data: {e}")

    def log_to_excel(self, filename, timestamp):
        """Log image data to Excel file"""
        try:
            wb = openpyxl.load_workbook(str(self.excel_file))
            ws = wb.active
            
            # Use GPS data if available, otherwise log N/A
            if self.latest_gps is not None:
                lat = self.latest_gps['latitude']
                lon = self.latest_gps['longitude']
                alt = self.latest_gps['altitude']
            else:
                lat = lon = alt = "N/A"
            
            heading = self.latest_heading if self.latest_heading is not None else "N/A"
            
            row_data = [
                filename,
                lat,
                lon,
                alt,
                heading,
                timestamp.strftime("%Y-%m-%d %H:%M:%S")
            ]
            
            ws.append(row_data)
            wb.save(str(self.excel_file))
            
        except Exception as e:
            self.get_logger().warn(f"Failed to log to Excel: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGeotaggingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
