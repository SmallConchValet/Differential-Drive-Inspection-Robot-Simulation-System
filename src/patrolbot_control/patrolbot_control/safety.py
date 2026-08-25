import math, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

latched=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL,history=HistoryPolicy.KEEP_LAST,depth=1)
class Safety(Node):
 def __init__(self):
  super().__init__('safety_node'); self.lock=threading.Lock(); self.state='RUNNING'; self.clear=True; self.valid=False
  self.declare_parameter('slowdown_distance',.70); self.declare_parameter('emergency_stop_distance',.40); self.declare_parameter('scan_half_angle',.261799)
  self.stop=self.create_publisher(Bool,'/emergency_stop',latched); self.limit=self.create_publisher(Float32,'/safety_speed_limit',latched); self.obs=self.create_publisher(Bool,'/obstacle_present',10)
  self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data,callback_group=MutuallyExclusiveCallbackGroup())
  self.create_service(Trigger,'/resume_patrol',self.resume,callback_group=MutuallyExclusiveCallbackGroup()); self.add_on_set_parameters_callback(self.params)
  self.timer=self.create_timer(.1,self.publish); self.closest=float('inf')
 def params(self,ps):
  vals={p.name:p.value for p in ps}; slow=vals.get('slowdown_distance',self.get_parameter('slowdown_distance').value); stop=vals.get('emergency_stop_distance',self.get_parameter('emergency_stop_distance').value); half=vals.get('scan_half_angle',self.get_parameter('scan_half_angle').value)
  ok=slow>stop>0 and 0<half<=math.pi
  return SetParametersResult(successful=ok,reason='' if ok else 'require slowdown > emergency > 0 and 0 < angle <= pi')
 def scan(self,m):
  half=self.get_parameter('scan_half_angle').value; good=[]
  for i,r in enumerate(m.ranges):
   a=m.angle_min+i*m.angle_increment
   if abs(math.atan2(math.sin(a),math.cos(a)))<=half and math.isfinite(r) and m.range_min<=r<=m.range_max: good.append(r)
  with self.lock:
   self.valid=bool(good); self.closest=min(good) if good else float('inf'); stop=self.get_parameter('emergency_stop_distance').value
   self.clear=self.closest>=stop
   if self.closest<stop: self.state='EMERGENCY_STOP'
   elif self.state=='EMERGENCY_STOP': self.state='WAIT_RESUME'
   elif self.state!='WAIT_RESUME': self.state='OBSTACLE' if self.closest<self.get_parameter('slowdown_distance').value else 'RUNNING'
 def publish(self):
  with self.lock:
   slow=self.get_parameter('slowdown_distance').value; stopd=self.get_parameter('emergency_stop_distance').value
   lim=1.0 if self.state=='RUNNING' else ((self.closest-stopd)/(slow-stopd) if self.state=='OBSTACLE' else 0.0); stopped=self.state in ('EMERGENCY_STOP','WAIT_RESUME')
   self.stop.publish(Bool(data=stopped)); self.limit.publish(Float32(data=float(max(0,min(1,lim))))); self.obs.publish(Bool(data=self.closest<slow))
 def resume(self,req,res):
  with self.lock:
   if self.state=='WAIT_RESUME' and self.clear and self.valid: self.state='RUNNING'; res.success=True; res.message='patrol resumed'
   else: res.success=False; res.message='not safely resumable'
  return res
def main():
 rclpy.init(); n=Safety(); ex=MultiThreadedExecutor(num_threads=3); ex.add_node(n)
 try: ex.spin()
 finally: n.destroy_node(); rclpy.shutdown()
