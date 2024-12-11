import rclpy
import math
import tf2_py
from rclpy.node import Node
from nav_msgs.msg import Odometry    # Odometryメッセージ型をインポート
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist, TransformStamped
from std_msgs.msg import String
from std_msgs.msg import Char
from std_msgs.msg import Float32 
from turtlesim.msg import Pose as NPose
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class OdomNode(Node):
    def __init__(self):
        super().__init__('robot_odom_node')
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_odom_pub = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.sub = self.create_subscription(NPose, 'op', self.msg_callback, 10)
        self.tf_laser_pub = StaticTransformBroadcaster(self) #static 値がかわらない
        

#/odom topic
#header
#   stamp(タイムスタンプ):UNIX時間
#   frame_id (基準となる座標系の名前)
#child_frame_id(ロボット座標系の名前) 
#pose(姿勢)
#   pose:姿勢
#       position:位置(x, y, z)[m]
#       orientation:向き(x, y, z, w:クォータニオン)
#   convariance:分散共分散行列
#twist(ツイスト)
#   twist:ツイスト
#       linear:並進速度(x, y, z)[m/s] 
#       angular:角速度(x, y, z)[rad/s]
#   convariance:分散共分散行列
# 
        #odometory header
        self.odom_header = Header()
        self.odom_header.frame_id = "odom"
        #pose
        self.m_pose = Pose()
        self.m_pose.position.x = 0.0
        self.m_pose.position.y = 0.0
        self.m_pose.position.z = 0.0
        self.m_pose.orientation.x = 0.0
        self.m_pose.orientation.y = 0.0
        self.m_pose.orientation.z = 0.0
        self.m_pose.orientation.w = 1.0

        #twist
        self.m_twist = Twist()

        #odometory
        self.m_odom = Odometry()
        self.m_odom.header = self.odom_header
        self.m_odom.child_frame_id = 'base_footprint'
        self.m_odom.pose.pose = self.m_pose
        self.m_odom.twist.twist = self.m_twist 
        
        #robot_op_node odom
        self.robot_odom = NPose() #init
        self.robot_odom.x = 0.0
        self.robot_odom.y = 0.0
        self.robot_odom.theta = 0.0

        tf_lidar = TransformStamped()
        tf_lidar.header.frame_id = 'base_footprint'
        tf_lidar.child_frame_id = 'laser_frame'
        tf_lidar.transform.translation.x = -0.075
        tf_lidar.transform.translation.y = -0.035
        tf_lidar.transform.translation.z = 0.18
        tf_lidar.transform.rotation.w = 0.9238796594468422
        tf_lidar.transform.rotation.x = 0.0
        tf_lidar.transform.rotation.y = 0.0
        tf_lidar.transform.rotation.z = 0.3826831259154
        tf_lidar.header.stamp = self.get_clock().now().to_msg()
        self.tf_laser_pub.sendTransform(tf_lidar)
        

    def msg_callback(self, msg):
        print(msg)
        print(type(msg))
        self.robot_odom = msg

    def timer_callback(self):
        #受け取ったデータの加工
        q = self.quaternion_from_euler(0, 0, self.robot_odom.theta)
        self.m_pose.position.x = self.robot_odom.x
        self.m_pose.position.y = self.robot_odom.y
        self.m_pose.orientation.x = q[1]
        self.m_pose.orientation.y = q[2]
        self.m_pose.orientation.z = q[3]
        self.m_pose.orientation.w = q[0]
        #print(q)

        self.m_odom.header = self.odom_header
        self.m_odom.pose.pose = self.m_pose
        self.m_odom.twist.twist = self.m_twist

        #update tf
        map_frame = TransformStamped()
        map_frame.header.frame_id = 'odom'
        map_frame.child_frame_id = 'base_footprint'
        map_frame.transform.translation.x = self.m_pose.position.x
        map_frame.transform.translation.y = self.m_pose.position.y
        map_frame.transform.translation.z = 0.0
        map_frame.transform.rotation.w = self.m_pose.orientation.w
        map_frame.transform.rotation.x = self.m_pose.orientation.x
        map_frame.transform.rotation.y = self.m_pose.orientation.y
        map_frame.transform.rotation.z = self.m_pose.orientation.z

        self.m_odom.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.m_odom)

        map_frame.header.stamp = self.get_clock().now().to_msg()
        self.tf_odom_pub.sendTransform(map_frame)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion ( w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr #w
        q[1] = cy * cp * sr - sy * sp * cr #x
        q[2] = sy * cp * sr + cy * sp * cr #y
        q[3] = sy * cp * cr - cy * sp * sr #z

        return q

def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+C')
    node.destroy_node() #destory
    rclpy.shutdown()

if __name__ == '__main__':
    main()