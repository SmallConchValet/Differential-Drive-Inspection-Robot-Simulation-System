import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float32,UInt8
latched=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,depth=1)
cmdq=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE,depth=1)
class Arbiter(Node):
 def __init__(self):
  super().__init__('velocity_arbiter'); self.lock=threading.Lock(); self.mode=0; self.estop=True; self.limit=0.; self.auto=Twist(); self.manual=Twist(); self.at=self.mt=None
  for n,v in [('cmd_vel_timeout',.3),('output_frequency',20.),('max_linear_velocity',.5),('max_angular_velocity',1.)]: self.declare_parameter(n,v)
  self.pub=self.create_publisher(Twist,'/cmd_vel',1); self.create_subscription(Twist,'/cmd_vel_auto',lambda m:self.cmd(m,True),cmdq); self.create_subscription(Twist,'/cmd_vel_manual',lambda m:self.cmd(m,False),cmdq)
  self.create_subscription(UInt8,'/patrol_mode',lambda m:setattr(self,'mode',m.data),latched); self.create_subscription(Bool,'/emergency_stop',lambda m:setattr(self,'estop',m.data),latched); self.create_subscription(Float32,'/safety_speed_limit',lambda m:setattr(self,'limit',m.data),latched)
  self.create_timer(1./self.get_parameter('output_frequency').value,self.tick)
 def cmd(self,m,auto):
  setattr(self,'auto' if auto else 'manual',m); setattr(self,'at' if auto else 'mt',self.get_clock().now())
 def tick(self):
  z=Twist(); now=self.get_clock().now(); selected=self.auto if self.mode==1 else self.manual; stamp=self.at if self.mode==1 else self.mt
  if self.mode==2 or self.estop or stamp is None or (now-stamp).nanoseconds/1e9>self.get_parameter('cmd_vel_timeout').value: self.pub.publish(z); return
  s=max(0.,min(1.,self.limit)); selected.linear.x=max(-self.get_parameter('max_linear_velocity').value,min(self.get_parameter('max_linear_velocity').value,selected.linear.x*s)); selected.angular.z=max(-self.get_parameter('max_angular_velocity').value,min(self.get_parameter('max_angular_velocity').value,selected.angular.z*s)); self.pub.publish(selected)
 def destroy_node(self): self.pub.publish(Twist()); return super().destroy_node()
def main():
 rclpy.init(); n=Arbiter(); ex=MultiThreadedExecutor(num_threads=3); ex.add_node(n)
 try: ex.spin()
 finally: n.destroy_node(); rclpy.shutdown()
