#!/usr/bin/env python3
"""
Simple node to send Open Manipulator Y to a hard-coded pose
"""


import time


import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint




class MoveArmToHardcodedPose(Node):
   """Node that sends the arm to a hard-coded target pose"""


   def __init__(self):
       super().__init__('move_arm_to_hardcoded')


       # Publisher for arm joint control
       self.arm_publisher = self.create_publisher(
           JointTrajectory, '/arm_controller/joint_trajectory', 10
       )


       self.arm_joint_names = [
           'joint1',
           'joint2',
           'joint3',
           'joint4',
           'joint5',
           'joint6',
       ]


       # Hard-coded target pose (in radians)
       self.target_pose = [
           0.0,      # joint1
           -0.5,     # joint2
           0.5,      # joint3
           0.5,      # joint4
           0.0,      # joint5
           0.0,      # joint6
       ]


       self.get_logger().info(f'Node initialized with target pose: {self.target_pose}')
    


     # Create a timer to send command after connection
       self.timer = self.create_timer(1.0, self.timer_callback)
       self.command_sent = False


   def timer_callback(self):
       if not self.command_sent:
           self.send_to_target_pose(duration_sec=2.0)
           self.command_sent = True
           # Optionally destroy the timer after sending
        #    self.timer.cancel()

   def send_to_target_pose(self, duration_sec: float = 2.0):
       """
       Send arm to the hard-coded target pose
      
       Args:
           duration_sec: Time in seconds to reach the target
       """
       arm_msg = JointTrajectory()
       arm_msg.joint_names = self.arm_joint_names
      
       arm_point = JointTrajectoryPoint()
       arm_point.positions = self.target_pose
       arm_point.time_from_start.sec = int(duration_sec)
       arm_point.time_from_start.nanosec = int((duration_sec % 1.0) * 1e9)
      
       arm_msg.points.append(arm_point)
       self.arm_publisher.publish(arm_msg)
      
       self.get_logger().info(
           f'Arm command sent to target pose: {[f"{p:.3f}" for p in self.target_pose]} '
           f'(Duration: {duration_sec}s)'
       )




def main():
   rclpy.init()
   node = MoveArmToHardcodedPose()


   # Give the publisher time to connect
   time.sleep(1.0)


   # Send the arm to the hard-coded pose
   node.send_to_target_pose(duration_sec=2.0)


   # Keep the node running for a bit
   time.sleep(3.0)


   node.destroy_node()
   rclpy.shutdown()




if __name__ == '__main__':
   main()
