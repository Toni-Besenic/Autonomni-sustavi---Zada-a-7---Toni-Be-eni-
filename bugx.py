import math
import rclpy
from rclpy.node import Node
import tf_transformations as transform
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

class Bug0(Node):
    # Initialization
    def __init__(self):
        super().__init__('bug0')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.location_callback, 10)
        self.fl_sensor_sub = self.create_subscription(Range, '/fl_range_sensor', self.fl_sensor_callback, 10)
        self.fr_sensor_sub = self.create_subscription(Range, '/fr_range_sensor', self.fr_sensor_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.bug_algorithm_timer = self.create_timer(0.1, self.bug_algorithm_callback)
        self.goal_x = None
        self.goal_y = None
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None
        self.current_theta = None
        self.fl_sensor_value = 0.0
        self.fr_sensor_value = 0.0
        self.cmd_vel_msg = Twist()
        self.state = 'GO_TO_GOAL'
        self.hit_point_x = None
        self.hit_point_y = None

    # Method for goal update
    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.start_x = self.current_x
        self.start_y = self.current_y

    # Method for robot current position
    def location_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        self.current_theta = transform.euler_from_quaternion(q)[2]  # [-pi, pi]

    # Methods for updating sensor values
    def fl_sensor_callback(self, msg):
        self.fl_sensor_value = msg.range

    def fr_sensor_callback(self, msg):
        self.fr_sensor_value = msg.range

    def bug_algorithm_callback(self):
        if self.goal_x is None or self.goal_y is None:
            return  

        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

        if distance_to_goal < 0.1:  
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pub.publish(self.cmd_vel_msg)
            return

        if self.state == 'GO_TO_GOAL':
            if self.fl_sensor_value < 0.7 or self.fr_sensor_value < 0.7:  
                self.state = 'FOLLOW_WALL'
                self.hit_point_x = self.current_x
                self.hit_point_y = self.current_y
            else:
                angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                angle_diff = self.normalize_angle(angle_to_goal - self.current_theta)

                if abs(angle_diff) > 0.1: 
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_msg.angular.z = 0.2 if angle_diff > 0 else -0.2               
                else:
                    self.cmd_vel_msg.linear.x = 0.3
                    self.cmd_vel_msg.angular.z = 0.0

        if self.state == 'GO_TO_GOAL_BRZE':
            if self.fl_sensor_value < 0.7 or self.fr_sensor_value < 0.7:  
                self.state = 'FOLLOW_WALL'
                self.hit_point_x = self.current_x
                self.hit_point_y = self.current_y
            else:
                angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                angle_diff = self.normalize_angle(angle_to_goal - self.current_theta)

                if abs(angle_diff) > 0.1: #and abs(angle_diff) < 0.3:
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_msg.angular.z = 0.5 if angle_diff > 0 else -0.5               
                else:
                    self.cmd_vel_msg.linear.x = 0.3
                    self.cmd_vel_msg.angular.z = 0.0            

        elif self.state == 'FOLLOW_WALL':
            if self.fl_sensor_value > 0.8 and self.fr_sensor_value > 0.8:  
                self.state = 'GO_TO_GOAL_BRZE'
            else:
                self.cmd_vel_msg.linear.x = 0.1  
                if self.fl_sensor_value < 0.7 : 
                    self.cmd_vel_msg.angular.z = -0.9
                    self.cmd_vel_msg.linear.x = 0.2
                if self.fr_sensor_value < 0.7:
                    self.cmd_vel_msg.angular.z = 0.9
                    self.cmd_vel_msg.linear.x = 0.2
                

        
        self.cmd_pub.publish(self.cmd_vel_msg)
        self.get_logger().info(f"State: {self.state}")
        self.get_logger().info(f"Current X: {self.current_x}, Current Y: {self.current_y}, Current theta: {self.current_theta}")
        self.get_logger().info(f"FL Sensor: {self.fl_sensor_value}, FR Sensor: {self.fr_sensor_value}")
        self.get_logger().info(f"Goal X: {self.goal_x}, Goal Y: {self.goal_y}")

    def is_on_m_line(self):     
        if self.goal_x is None or self.goal_y is None or self.start_x is None or self.start_y is None:
            return False
        m = (self.goal_y - self.start_y) / (self.goal_x - self.start_x) if (self.goal_x - self.start_x) != 0 else float('inf')
        c = self.start_y - m * self.start_x
        return abs(self.current_y - (m * self.current_x + c)) < 0.1

    def distance_from_start(self):
        return math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)

    def distance_from_hit_point(self):
        return math.sqrt((self.current_x - self.hit_point_x)**2 + (self.current_y - self.hit_point_y)**2)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    bug_node = Bug0()
    rclpy.spin(bug_node)
    bug_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()