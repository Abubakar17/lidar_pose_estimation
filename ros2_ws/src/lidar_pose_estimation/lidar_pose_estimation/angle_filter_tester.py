#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math

class AngleFilterTester(Node):
    def __init__(self):
        super().__init__('angle_filter_tester')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )
            
        self.publisher_ = self.create_publisher(
            LaserScan, 
            '/scan_filtered', 
            qos_profile=qos_profile_sensor_data
        )
        
        # Start with a narrow window around 180 degrees (where the cube appears to be)
        self.angle_min_deg = 160.0
        self.angle_max_deg = 200.0
        
        self.get_logger().info(f'Angle Filter Tester started!')
        self.get_logger().info(f'Current range: {self.angle_min_deg}° to {self.angle_max_deg}°')
        self.get_logger().info('Adjust with: ros2 param set /angle_filter_tester angle_min_deg <value>')
        
        # Declare parameters for runtime adjustment
        self.declare_parameter('angle_min_deg', self.angle_min_deg)
        self.declare_parameter('angle_max_deg', self.angle_max_deg)
        
        # Timer to check for parameter updates
        self.timer = self.create_timer(1.0, self.update_params)
        
        self.last_point_count = 0
    
    def update_params(self):
        """Check if parameters were updated"""
        new_min = self.get_parameter('angle_min_deg').value
        new_max = self.get_parameter('angle_max_deg').value
        
        if new_min != self.angle_min_deg or new_max != self.angle_max_deg:
            self.angle_min_deg = new_min
            self.angle_max_deg = new_max
            self.get_logger().info(f'Updated range: {self.angle_min_deg}° to {self.angle_max_deg}°')
    
    def scan_callback(self, msg):
        filtered_scan = msg
        ranges = list(msg.ranges)
        current_angle = msg.angle_min
        valid_points = 0
        angle_distribution = {}
        
        for i in range(len(ranges)):
            angle_deg = math.degrees(current_angle)
            # Normalize to 0-360 range instead of -180 to +180
            angle_deg = angle_deg % 360
            
            # Keep only points within our angle range
            if self.angle_min_deg <= angle_deg <= self.angle_max_deg:
                if math.isfinite(ranges[i]) and ranges[i] > 0.0:
                    valid_points += 1
                    angle_bucket = int(angle_deg / 10) * 10
                    angle_distribution[angle_bucket] = angle_distribution.get(angle_bucket, 0) + 1
            else:
                ranges[i] = float('inf')
            
            current_angle += msg.angle_increment
        
        # Log info periodically
        if valid_points != self.last_point_count:
            self.get_logger().info(f'Valid points: {valid_points}')
            if angle_distribution:
                self.get_logger().info(f'Angle distribution (10° buckets): {angle_distribution}')
            self.last_point_count = valid_points
        
        filtered_scan.ranges = ranges
        self.publisher_.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = AngleFilterTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
