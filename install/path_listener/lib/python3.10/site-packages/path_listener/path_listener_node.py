import rclpy
import math
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String


class PathListener(Node):
    def __init__(self):
        super().__init__('path_listener')
        self.sub1 = self.create_subscription(MarkerArray,'/trajectories',self.local_plan_cb,10)
        self.turn_signal_pub = self.create_publisher(String,'/blinker_led_command',10)
        
        self.angular_threshold = 20 #degの閾値
        self.last_pub = None
        self.get_logger().info('num_points:compersion_deg:string')
        self.get_logger().info('trajectoriesBA-JON222')

    
    def angle_diff(self,a, b):  #2つの角度a,b[deg]の差を[-180,180]の範囲で返す
        d = (a - b + 180) % 360 - 180
        return d

    def deg(self,pose_stamp):
        w = pose_stamp.pose.orientation.w
        z = pose_stamp.pose.orientation.z
        yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
        deg = math.degrees(yaw)
        return deg


    def pub(self,string):
        if self.last_pub == string:
            return
        else:
            self.last_pub = string

        msg = String()
        msg.data = string
        self.turn_signal_pub.publish(msg)

    
    def local_plan_cb(self,msg:MarkerArray):
        # MarkerArrayからMarkerを取得
        if len(msg.markers) == 0:
            return
        
        # 最初のMarkerのpointsを使用
        marker = msg.markers[0]
        num_points = len(marker.points)
        
        if num_points < 2: #点が少ないときは無視
            return

        # 始点と終点の位置から角度を計算
        start_point = marker.points[0]
        end_point = marker.points[-1]
        
        # ベクトルの角度を計算
        dx = end_point.x - start_point.x
        dy = end_point.y - start_point.y
        
        # 始点から終点への方向角度
        angle_rad = math.atan2(dy, dx)
        comparsion_deg = math.degrees(angle_rad)

        if comparsion_deg > self.angular_threshold:
            string = 'left'
        elif comparsion_deg < -self.angular_threshold:
            string = 'right'
        else:
            string = 'strai'
            
        self.pub(string)
        self.get_logger().info(f'{num_points} : {comparsion_deg:.1f} : {string}')


def main(args=None):
    rclpy.init(args=args)
    node = PathListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
