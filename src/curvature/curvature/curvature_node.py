import rclpy
import math
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String




class Curvature(Node):
   def __init__(self):
       super().__init__('Curvature')
       self.sub1 = self.create_subscription(MarkerArray ,'/trajectories',self.trajectories_cb,10)
       self.turn_signal_pub = self.create_publisher(String,'/blinker_led_command',10)




   def curvature_from_three_points(p1, p2, p3):
       (x1,y1),(x2,y2),(x3,y3) = p1,p2,p3
       a = math.hypot(x2-x1, y2-y1)
       b = math.hypot(x3-x2, y3-y2)
       c = math.hypot(x3-x1, y3-y1)
       # area by shoelace
       area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
       if area == 0 or a*b*c == 0:
           return 0.0  # 直線(または重複点)は曲率0
       R = (a*b*c) / (4.0 * area)
       k = 1.0 / R
       # sign from cross product of vectors (p2-p1) x (p3-p2)
       v1x, v1y = x2 - x1, y2 - y1
       v2x, v2y = x3 - x2, y3 - y2
       cross = v1x * v2y - v1y * v2x
       return math.copysign(k, cross)


# テスト
   p1=(0,0); p2=(1,1); p3=(2,0)
   print(curvature_from_three_points(p1,p2,p3))  # -> -1.0




   def trajectories_cb(self,msg:MarkerArray):
       if len(msg.markers) < 3:
           return
       points = [(m.pose.position.x, m.pose.position.y) for m in msg.markers]
       k = self.curvature_from_three_points(points[0], points[1], points[2])
       turn_signal = String()
       if k > 0.01:
           turn_signal.data = 'RIGHT'
       elif k < -0.01:
           turn_signal.data = 'LEFT'
       else:
           turn_signal.data = 'OFF'
       self.turn_signal_pub.publish(turn_signal)










def main(args=None):
   rclpy.init(args=args)
   node = Curvature()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()

