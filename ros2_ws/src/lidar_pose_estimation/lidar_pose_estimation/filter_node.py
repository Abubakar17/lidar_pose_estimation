import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math

class AngleFilter(Node):

    def __init__(self):
        super().__init__('angle_filter_node')
        
        # Souscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )
            
        # Publication
        self.publisher_ = self.create_publisher(
            LaserScan, 
            '/scan_filtered', 
            qos_profile=qos_profile_sensor_data
        )
        
      
        self.angle_min_deg = -10.0
        self.angle_max_deg = 10.0

        self.get_logger().info('Filtre Lidar inversé (Mode Tunnel) démarré !')

    def scan_callback(self, msg):
        filtered_scan = msg
        ranges = list(msg.ranges)
        current_angle = msg.angle_min
        
        for i in range(len(ranges)):
            # Conversion en degrés
            angle_deg = math.degrees(current_angle)
            
            # Normalisation entre -180 et +180
            angle_deg = (angle_deg + 180) % 360 - 180
            
            # --- LOGIQUE INVERSÉE ---
            # Si l'angle est PLUS PETIT que le min OU PLUS GRAND que le max
            # C'est-à-dire s'il est "sur les côtés ou derrière", on l'efface.
            if angle_deg < self.angle_min_deg or angle_deg > self.angle_max_deg:
                ranges[i] = float('inf')
            
            current_angle += msg.angle_increment

        filtered_scan.ranges = ranges
        self.publisher_.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = AngleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
