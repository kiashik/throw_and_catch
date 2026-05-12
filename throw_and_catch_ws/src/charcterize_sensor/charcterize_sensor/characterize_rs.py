"""
--- Purpose:
This node will characterize the noise of the Intel RealSense D455. The 
characterization process will be done after converting ball pose to robot frame.

The ball is to be placed a known 3D position in robot frame. Then x, y, and z
axis measurements will be characterized by finding the sampled mean and variance 
of the sensor. The mu printed to the terminal is not the bias. The bias if the 
(mu - the true position) of the ball in robot frame. The variance is also 
printed to the terminal.



Taking lots of inspiration from EE 471 lab 2.

--- Parameters/ Configurations:
- Ball pose in robot frame: /vision/ball_pose_robot (geometry_msgs/PoseStamped)
    Used to characterize the noise of the sensor.


    
--- Created on 05-09-2026: works successfully. 
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


import matplotlib.pyplot as plt
import numpy as np


class CharacterizeRealSenseD455(Node):
    def __init__(self):
        super().__init__('characterize_realsense_d455')


        # number of samples to collect while characterizing.
        self.declare_parameter('specifiedSamples', 2000)
        samples = self.get_parameter("specifiedSamples").get_parameter_value().integer_value

        # accumulated ball position readings for plotting and calculating stats.
        # positionData's columns are x, y, and z axis measurements respectively.
        # each row is a new sample at time = self.time[i]
        self.positionData = np.zeros((samples,3))   
        self.time = np.zeros((samples))
        
        # index to keep track of which row of positionData and time to write new samples to.
        # Kill node when sampleIndex reaches specifiedSamples since we have enough data to characterize the sensor noise.
        self.sampleIndex = 0    

        self.startTime = None   #  time when we start collecting samples. Used to calculate elapsed time for each sample.

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.ball_pose_subscription = self.create_subscription(
            PoseStamped,
            '/vision/ball_pose_robot',
            self.ball_pose_callback,
            qos_profile
        )


    def ball_pose_callback(self, msg):
        now = self.get_clock().now()

        if self.startTime is None:  # only true for the first callback
            self.startTime = now
        
        # if we still need more samples i.e. sampleIndex is less than specifiedSamples
        if self.sampleIndex < (self.positionData.shape[0]): 
            self.get_logger().info(f"Collecting self.sampleIndex = {self.sampleIndex} at time {(now - self.startTime).nanoseconds / 1e9:.2f} seconds")

            # save ball position data and timestamp
            self.positionData[self.sampleIndex,:] = [
                msg.pose.position.x, 
                msg.pose.position.y, 
                msg.pose.position.z
            ]
            self.time[self.sampleIndex] = (now - self.startTime).nanoseconds / 1e9

            self.sampleIndex += 1
        else:
            self.sensor_stats()
            self.plot_results()
            plt.show()

            # we have enough samples to characterize the sensor noise, so we can stop the node.
            self.get_logger().info("Collected enough samples to characterize sensor noise. Shutting down node.")
            rclpy.shutdown()


    
    def sensor_stats(self):
        self.get_logger().info("Calculating sensor noise statistics...")
        
        mu = np.zeros(3)
        var = np.zeros(3)

        for i in range(self.positionData.shape[1]): # for each axis x, y, and z
            mu[i] = np.mean(self.positionData[:, i])            # get mean for each variable
            var[i] = np.var(self.positionData[:, i], ddof=1)    # calc unbiased var
        
        self.get_logger().warn(f"mu: {mu}")
        self.get_logger().warn(f"var: {var}")

        num_bins = 300
        fig, axes =plt.subplots(3,1, layout="constrained")
        axes[0].hist(self.positionData[:,0], bins=num_bins)
        axes[0].set_ylabel("samples")
        axes[0].set_xlabel("x [m]")
        axes[0].grid(True)

        axes[1].hist(self.positionData[:,1], bins=num_bins)
        axes[1].set_ylabel("samples")
        axes[1].set_xlabel("y [m]")
        axes[1].grid(True)

        axes[2].hist(self.positionData[:,2], bins=num_bins)
        axes[2].set_ylabel("samples")
        axes[2].set_xlabel("z [m]")
        axes[2].grid(True)

        fig.suptitle('Histogram Plots')
        plt.tight_layout()


    def plot_results(self):
        """
        Plots the time series of the ball position measurements in x, y, and z axes.
        x, y, z are from the positionData array where each column corresponds to 
        an axis and each row corresponds to a sample at a specific time in self.time.
        """
        fig, axes = plt.subplots(3, 1, sharex=True, layout="constrained")
        axes[0].scatter(self.time, self.positionData[:,0])
        axes[0].set_ylabel("x [m]")
        axes[0].grid(True)

        axes[1].scatter(self.time, self.positionData[:,1])
        axes[1].set_ylabel("y [m]")
        axes[1].grid(True)
    
        axes[2].scatter(self.time, self.positionData[:,2])
        axes[2].set_ylabel("z [m]")
        axes[2].grid(True)
        axes[2].set_xlabel("Time [s]")

        fig.suptitle('Time Series Plots')
        plt.tight_layout()


def main():
    rclpy.init()
    node = CharacterizeRealSenseD455()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
