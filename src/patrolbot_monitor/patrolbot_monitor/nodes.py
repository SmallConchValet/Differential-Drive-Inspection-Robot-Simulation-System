import math,rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer,TransformListener,TransformException
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,UInt8
from patrolbot_interfaces.msg import RobotStatus
class Monitor(Node):
 def __init__(self):
  super().__init__('patrol_monitor_node'); self.buf=Buffer(); self.listener=TransformListener(self.buf,self); self.last=None; self.distance=0.; self.create_timer(.1,self.tick)
 def tick(self):
  try:
   t=self.buf.lookup_transform('map','base_footprint',rclpy.time.Time(),timeout=Duration(seconds=.03)); p=(t.transform.translation.x,t.transform.translation.y)
   if self.last:self.distance+=math.hypot(p[0]-self.last[0],p[1]-self.last[1])
   self.last=p
  except TransformException as e:self.get_logger().warning('TF unavailable: '+str(e),throttle_duration_sec=2.)
class Status(Node):
 def __init__(self):
  super().__init__('robot_status_node'); self.declare_parameter('publish_frequency',5.); self.buf=Buffer(); self.listener=TransformListener(self.buf,self); self.mode=0; self.estop=False; self.obstacle=False; self.odom=Odometry(); self.pub=self.create_publisher(RobotStatus,'/robot_status',10)
  self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),10); self.create_subscription(UInt8,'/patrol_mode',lambda m:setattr(self,'mode',m.data),1); self.create_subscription(Bool,'/emergency_stop',lambda m:setattr(self,'estop',m.data),1); self.create_subscription(Bool,'/obstacle_present',lambda m:setattr(self,'obstacle',m.data),10); self.create_timer(1/self.get_parameter('publish_frequency').value,self.tick)
 def tick(self):
  m=RobotStatus(); m.header.stamp=self.get_clock().now().to_msg(); m.header.frame_id='map'; m.battery_level=100.; m.patrol_mode=self.mode; m.obstacle_present=self.obstacle; m.closest_obstacle_distance=float('inf'); m.linear_velocity=float(self.odom.twist.twist.linear.x); m.angular_velocity=float(self.odom.twist.twist.angular.z); m.status=4 if self.estop else (3 if self.mode==2 else (1 if abs(m.linear_velocity)+abs(m.angular_velocity)>0.01 else 0))
  try:
   t=self.buf.lookup_transform('map','base_footprint',rclpy.time.Time()); m.pose.position.x=t.transform.translation.x; m.pose.position.y=t.transform.translation.y; m.pose.orientation=t.transform.rotation
  except TransformException: pass
  self.pub.publish(m)
def run(cls):
 rclpy.init(); n=cls()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.shutdown()
def monitor_main():run(Monitor)
def status_main():run(Status)
