import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1, self.publish_goal)
        self.di = 0.0 #distance 

    def publish_goal(self):
        # PoseStampedメッセージの作成
        goal = PoseStamped()

        # Header情報の設定
        goal.header = Header()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'  # 　mapの座標系を使う
        # 4m dake susumaseru
        goal.pose.position.x = self.di
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0

        # クォータニオン形式での回転姿勢（例: 0度回転）
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # トピックに目標を送信
        self.publisher.publish(goal)
        self.get_logger().info(f"Published goal: {goal}")

        #di
        if self.di != 0.0:
            self.di = 0.0
        else:
            self.di = 4.0

def main():
    rclpy.init()
    node = GoalPublisher()
    try:
        rclpy.spin(node)
        # while True:
        #     c = input()
        #     if(c == 'm'):
        #         print("mae ni 50m susumuyo")
        #         node.publish_goal(di)
        #     elif(c == 'u'):
        #         node.publish_goal(-di)
        #         print("usiro ni 50m modoruyo")
        #     rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('Ctrl+C')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
