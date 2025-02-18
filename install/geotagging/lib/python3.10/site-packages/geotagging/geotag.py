#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
from PIL import Image as PILImage
import piexif
import datetime
import os
import openpyxl

class GeoTaggingNode(Node):
    def __init__(self):
        super().__init__('geotagging_node')
        # Declare parameters for topics and file paths.
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('gps_topic', '/mavros/global_position/global')
        self.declare_parameter('heading_topic', '/mavros/global_position/compass_hdg')
        self.declare_parameter('output_image_dir', 'geotagged_images')
        self.declare_parameter('excel_file', 'geotag_info.xlsx')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        heading_topic = self.get_parameter('heading_topic').get_parameter_value().string_value
        self.output_image_dir = self.get_parameter('output_image_dir').get_parameter_value().string_value
        self.excel_file = self.get_parameter('excel_file').get_parameter_value().string_value

        # Ensure the output directory exists.
        os.makedirs(self.output_image_dir, exist_ok=True)

        self.bridge = CvBridge()
        # Subscriptions: image, GPS, and heading.
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, gps_topic, self.gps_callback, 10)
        self.heading_sub = self.create_subscription(Float64, heading_topic, self.heading_callback, 10)

        self.latest_gps = None       # Tuple: (latitude, longitude, altitude)
        self.latest_heading = None   # Heading value (float)
        self.image_counter = 0

        # Create an Excel file with headers if it does not already exist.
        if not os.path.exists(self.excel_file):
            wb = openpyxl.Workbook()
            ws = wb.active
            ws.title = "Geotag Info"
            headers = ["File Name", "Latitude", "Longitude", "Altitude", "Heading", "Timestamp"]
            ws.append(headers)
            wb.save(self.excel_file)
            self.get_logger().info(f"Excel file created: {self.excel_file}")

    def gps_callback(self, msg: NavSatFix):
        self.latest_gps = (msg.latitude, msg.longitude, msg.altitude)
        self.get_logger().info(f"GPS data updated: {self.latest_gps}")

    def heading_callback(self, msg: Float64):
        self.latest_heading = msg.data
        self.get_logger().info(f"Heading updated: {self.latest_heading}")

    def image_callback(self, msg: Image):
        # Only process if we have valid GPS and heading data.
        if self.latest_gps is None or self.latest_heading is None:
            self.get_logger().warn("Waiting for both GPS and heading data to geotag image.")
            return

        try:
            # Convert the ROS Image message to an OpenCV image.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Generate a simple sequential file name.
        self.image_counter += 1
        filename = f"image_{self.image_counter}.jpg"
        filepath = os.path.join(self.output_image_dir, filename)
        # Save the image using OpenCV.
        cv2.imwrite(filepath, cv_image)

        # Get the current timestamp in EXIF format.
        timestamp = datetime.datetime.now().strftime("%Y:%m:%d %H:%M:%S")

        # Add EXIF metadata (GPS, heading, and timestamp) to the image.
        self.add_exif_metadata(filepath, self.latest_gps, self.latest_heading, timestamp)

        # Append the image info to the Excel file.
        self.append_excel_row(filename, self.latest_gps, self.latest_heading, timestamp)

        self.get_logger().info(f"Saved geotagged image: {filepath}")

    def add_exif_metadata(self, image_path, gps_data, heading, timestamp):
        def deg_to_dms_rational(deg):
            # Convert decimal degrees into degrees, minutes, and seconds (rational numbers).
            d = int(abs(deg))
            m = int((abs(deg) - d) * 60)
            s = (abs(deg) - d - m / 60) * 3600
            return ((d, 1), (m, 1), (int(s * 100), 100))

        lat, lon, alt = gps_data

        # Create the EXIF dictionary.
        exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}, "thumbnail": None}
        # Set the date/time.
        exif_dict["0th"][piexif.ImageIFD.DateTime] = timestamp
        exif_dict["Exif"][piexif.ExifIFD.DateTimeOriginal] = timestamp
        exif_dict["Exif"][piexif.ExifIFD.DateTimeDigitized] = timestamp
        # Set GPS latitude.
        exif_dict["GPS"][piexif.GPSIFD.GPSLatitude] = deg_to_dms_rational(lat)
        exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef] = "N" if lat >= 0 else "S"
        # Set GPS longitude.
        exif_dict["GPS"][piexif.GPSIFD.GPSLongitude] = deg_to_dms_rational(lon)
        exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef] = "E" if lon >= 0 else "W"
        # Set GPS altitude.
        exif_dict["GPS"][piexif.GPSIFD.GPSAltitude] = (int(abs(alt) * 100), 100)
        exif_dict["GPS"][piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
        # Store heading as the image direction.
        exif_dict["GPS"][piexif.GPSIFD.GPSImgDirection] = (int(abs(heading) * 100), 100)
        exif_dict["GPS"][piexif.GPSIFD.GPSImgDirectionRef] = "T"  # T = True direction

        # Dump and insert the EXIF data.
        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, image_path)

    def append_excel_row(self, filename, gps_data, heading, timestamp):
        # Open the existing Excel file and append a new row.
        wb = openpyxl.load_workbook(self.excel_file)
        ws = wb.active
        lat, lon, alt = gps_data
        ws.append([filename, lat, lon, alt, heading, timestamp])
        wb.save(self.excel_file)

def main(args=None):
    rclpy.init(args=args)
    node = GeoTaggingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
