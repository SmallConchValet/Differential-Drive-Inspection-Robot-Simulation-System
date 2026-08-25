import math,threading,time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import UInt8
from patrolbot_interfaces.srv import SetPatrolMode
from patrolbot_interfaces.action import Patrol,NavigateWaypoint
latched=QoSProfile(reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL,depth=1)
class Manager(Node):
 def __init__(self):
  super().__init__('patrol_task_manager'); self.mode=0; self.active=None; self.inner=None; self.cancel=False; self.lock=threading.Lock(); self.declare_parameter('wait_time_at_waypoint',3.)
  self.mode_pub=self.create_publisher(UInt8,'/patrol_mode',latched); self.mode_pub.publish(UInt8(data=0))
  self.create_service(SetPatrolMode,'/set_patrol_mode',self.set_mode,callback_group=MutuallyExclusiveCallbackGroup()); self.client=ActionClient(self,NavigateWaypoint,'/navigate_waypoint',callback_group=ReentrantCallbackGroup())
  self.server=ActionServer(self,Patrol,'/patrol',goal_callback=self.goal_cb,cancel_callback=self.cancel_cb,execute_callback=self.execute,callback_group=ReentrantCallbackGroup())
 def set_mode(self,req,res):
  if req.mode not in (0,1,2): res.success=False; res.message='invalid mode'; return res
  with self.lock:
   self.mode=req.mode; self.mode_pub.publish(UInt8(data=req.mode))
   if req.mode==0 and self.active: self.cancel=True
  res.success=True; res.message=['MANUAL: task canceled','AUTO','PAUSE'][req.mode]; return res
 def valid(self,g): return bool(g.waypoints) and all(math.isfinite(v) and (-.01<=w.x<=8.01 if i==0 else -.01<=w.y<=6.01 if i==1 else True) for w in g.waypoints for i,v in enumerate((w.x,w.y,w.yaw)))
 def goal_cb(self,g):
  with self.lock: ok=self.mode==1 and self.active is None and self.valid(g)
  return GoalResponse.ACCEPT if ok else GoalResponse.REJECT
 def cancel_cb(self,_): self.cancel=True; return CancelResponse.ACCEPT
 def inner_fb(self,outer,index,total,fb):
  f=Patrol.Feedback(); f.current_waypoint_index=index; f.total_waypoints=total; f.current_pose=fb.feedback.current_pose; f.distance_to_goal=fb.feedback.distance_remaining; f.robot_status=1; outer.publish_feedback(f)
 def execute(self,gh):
  with self.lock: self.active=gh; self.cancel=False
  start=self.get_clock().now(); done=0; result=Patrol.Result()
  try:
   for i,w in enumerate(gh.request.waypoints,1):
    if self.cancel or gh.is_cancel_requested: break
    while not self.client.wait_for_server(timeout_sec=.2):
     if self.cancel: break
    send=self.client.send_goal_async(NavigateWaypoint.Goal(target=w),feedback_callback=lambda f,j=i:self.inner_fb(gh,j,len(gh.request.waypoints),f))
    while not send.done() and not self.cancel: time.sleep(.02)
    if self.cancel: break
    self.inner=send.result()
    if not self.inner.accepted: result.message='inner goal rejected'; gh.abort(); return result
    rf=self.inner.get_result_async()
    while not rf.done() and not self.cancel: time.sleep(.02)
    if self.cancel:
     self.inner.cancel_goal_async(); break
    if not rf.result().result.success: result.message=rf.result().result.message; gh.abort(); return result
    done=i; end=self.get_clock().now()+rclpy.duration.Duration(seconds=self.get_parameter('wait_time_at_waypoint').value)
    while self.get_clock().now()<end and not self.cancel: time.sleep(.05)
   result.completed_waypoints=done; ns=(self.get_clock().now()-start).nanoseconds; result.total_time.sec=ns//1000000000; result.total_time.nanosec=ns%1000000000
   if self.cancel or gh.is_cancel_requested: result.success=False; result.message='canceled'; gh.canceled()
   else: result.success=True; result.message='patrol complete'; gh.succeed()
   return result
  finally:
   with self.lock: self.active=None; self.inner=None; self.cancel=False
def main():
 rclpy.init(); n=Manager(); ex=MultiThreadedExecutor(num_threads=4); ex.add_node(n)
 try: ex.spin()
 finally: n.destroy_node(); rclpy.shutdown()
