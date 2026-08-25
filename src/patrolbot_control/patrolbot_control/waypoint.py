import math,threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import Bool,UInt8
from tf2_ros import Buffer,TransformListener,TransformException
from patrolbot_interfaces.action import NavigateWaypoint
latched=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL,depth=1)
def norm(a): return math.atan2(math.sin(a),math.cos(a))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
class Controller(Node):
 def __init__(self):
  super().__init__('waypoint_controller'); self.lock=threading.Lock(); self.goal=None; self.mode=0; self.stop=True; self.started=None; self.tf_bad=None
  defaults={'control_frequency':20.,'linear_kp':.8,'angular_kp':1.5,'final_angular_kp':1.2,'max_linear_velocity':.5,'max_angular_velocity':1.,'rotate_in_place_threshold':.5,'goal_tolerance':.15,'yaw_tolerance':.10,'waypoint_timeout':60.,'tf_failure_timeout':1.}
  for k,v in defaults.items(): self.declare_parameter(k,v)
  self.pub=self.create_publisher(Twist,'/cmd_vel_auto',1); self.create_subscription(UInt8,'/patrol_mode',lambda m:setattr(self,'mode',m.data),latched); self.create_subscription(Bool,'/emergency_stop',lambda m:setattr(self,'stop',m.data),latched)
  self.buf=Buffer(); self.listener=TransformListener(self.buf,self); self.server=ActionServer(self,NavigateWaypoint,'/navigate_waypoint',goal_callback=self.accept,cancel_callback=lambda _:CancelResponse.ACCEPT,execute_callback=self.execute,callback_group=MutuallyExclusiveCallbackGroup())
 def accept(self,g):
  t=g.target
  return GoalResponse.ACCEPT if self.mode==1 and self.goal is None and all(math.isfinite(v) for v in (t.x,t.y,t.yaw)) else GoalResponse.REJECT
 def pose(self):
  t=self.buf.lookup_transform('map','base_footprint',rclpy.time.Time(),timeout=Duration(seconds=.05)); p=PoseStamped(); p.header=t.header; p.pose.position.x=t.transform.translation.x; p.pose.position.y=t.transform.translation.y; p.pose.orientation=t.transform.rotation; return p
 def execute(self,gh):
  self.goal=gh; self.started=self.get_clock().now(); rate=self.create_rate(self.get_parameter('control_frequency').value); target=gh.request.target
  try:
   while rclpy.ok():
    if gh.is_cancel_requested or self.mode==0: self.pub.publish(Twist()); gh.canceled(); return NavigateWaypoint.Result(success=False,message='canceled')
    if self.mode==2 or self.stop: self.pub.publish(Twist()); rate.sleep(); continue
    if (self.get_clock().now()-self.started).nanoseconds/1e9>self.get_parameter('waypoint_timeout').value: self.pub.publish(Twist()); gh.abort(); return NavigateWaypoint.Result(success=False,message='timeout')
    try: p=self.pose(); self.tf_bad=None
    except TransformException:
     self.pub.publish(Twist()); self.tf_bad=self.tf_bad or self.get_clock().now()
     if (self.get_clock().now()-self.tf_bad).nanoseconds/1e9>self.get_parameter('tf_failure_timeout').value: gh.abort(); return NavigateWaypoint.Result(success=False,message='TF unavailable')
     rate.sleep(); continue
    x,y=p.pose.position.x,p.pose.position.y; a=yaw(p.pose.orientation); dx,dy=target.x-x,target.y-y; d=math.hypot(dx,dy); he=norm(math.atan2(dy,dx)-a); fy=norm(target.yaw-a); cmd=Twist()
    if d<self.get_parameter('goal_tolerance').value:
     if abs(fy)<self.get_parameter('yaw_tolerance').value: self.pub.publish(cmd); gh.succeed(); return NavigateWaypoint.Result(success=True,message='reached')
     cmd.angular.z=self.get_parameter('final_angular_kp').value*fy
    else:
     cmd.angular.z=self.get_parameter('angular_kp').value*he
     if abs(he)<=self.get_parameter('rotate_in_place_threshold').value: cmd.linear.x=self.get_parameter('linear_kp').value*d*max(0.,math.cos(he))
    cmd.linear.x=min(self.get_parameter('max_linear_velocity').value,max(0.,cmd.linear.x)); ma=self.get_parameter('max_angular_velocity').value; cmd.angular.z=max(-ma,min(ma,cmd.angular.z)); self.pub.publish(cmd)
    fb=NavigateWaypoint.Feedback(); fb.current_pose=p; fb.distance_remaining=float(d); fb.yaw_error=float(fy if d<self.get_parameter('goal_tolerance').value else he); gh.publish_feedback(fb); rate.sleep()
  finally: self.pub.publish(Twist()); self.goal=None
 def destroy_node(self): self.pub.publish(Twist()); self.server.destroy(); return super().destroy_node()
def main():
 rclpy.init(); n=Controller(); ex=MultiThreadedExecutor(num_threads=3); ex.add_node(n)
 try: ex.spin()
 finally: n.destroy_node(); rclpy.shutdown()
