#!/usr/bin/env python

import rospy
import math
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler


# Constants
WHEELBASE =  1.46  # Distance between the front and rear wheels of the forklift
FRAME_ID =  799  # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE =  1  # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR =  0.01  # Steering factor to get the correct steering angle *DataSheet*
STEERING_ANGLE_CORRECTION_FACTOR =  1.098  # Steering angle correction factor driven by trial and error  

class ForkliftOdometryAM92:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x =  0.0  # X-coordinate of the forklift's position
        self.y =  0.0  # Y-coordinate of the forklift's position
        self.theta =  0.0  # Orientation angle of the forklift
        self.wheel_travel =  0.0  # Distance traveled by the wheels
        self.steering_angle_convert =  0.0  # Steering angle of the forklift after conversion
        self.steering_angle =  0.0  # Steering angle of the forklift
        self.rotation_angle =  0.0  # Rotation angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=100)  # Publisher for odometry data
        self.path_pub = rospy.Publisher('path', Path, queue_size=100)  # Publisher for path data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)  # Subscriber for frame messages
        self.time = rospy.Time.now()  # Time of the last frame received

    def handle_frame(self, msg, br = tf2_ros.TransformBroadcaster()):
        # Check if the received frame ID matches the desired frame ID
        if msg.id != FRAME_ID:
            return

        # Store the timestamp of the last received frame
        last_time = self.time

        # Get the current time
        current_time = rospy.Time.now()

        # Calculate the time difference
        dt = (current_time - last_time).to_sec()

        # Store the current time as the timestamp of the last received frame
        self.time = current_time

        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])

        prev_wheel_travel = self.wheel_travel

        # Extract wheel travel value from frame in meters
        self.wheel_travel = (byte1 | (byte2 <<  8))
        self.wheel_travel /=  1000.0

        # Extract steering angle value from frame in degrees
        self.steering_angle = ((byte3 | (byte4 <<  8)) * STEERING_ANGLE_FACTOR * STEERING_ANGLE_CORRECTION_FACTOR)/2

        # Convert steering angle to radians
        self.steering_angle_convert = math.radians(self.steering_angle)

        # Ignore large changes in wheel travel  
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
            return

        # Calculate linear and angular velocities
        linear_velocity = ((self.wheel_travel - prev_wheel_travel) / dt)
        angular_velocity = linear_velocity * (math.tan(self.steering_angle_convert) / WHEELBASE)

        # Update position and orientation using kinematics equations
        delta_theta = angular_velocity * dt
        self.theta += delta_theta
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = -linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = self.time
        t.header.frame_id = "/odom"
        t.child_frame_id = "/base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y 
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*quaternion_from_euler(0,  0, self.theta))
   
        br.sendTransform(t)

        # Publish the odometry message
        self.publish_odometry()
        
        # publish the path message
        self.publish_path()

    def publish_odometry(self):
        current_time = self.time

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_linḱ'
        odom.pose.pose = Pose(Point(self.x, self.y,  0.0), Quaternion(*quaternion_from_euler(0,  0, self.theta)))
        
        # Publish the odometry message
        self.odom_pub.publish(odom)

        

    def publish_path(self, path = Path()):
        
        
        path.header.stamp = self.time
        path.header.frame_id = 'odom'  # Assuming 'odom' is the frame of reference
        if path.poses is None:
            path.poses = []
        # Create a PoseStamped message for the current position
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'path'
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        pose_stamped.pose.position.z =  0.0  # Assuming no z-coordinate
        pose_stamped.pose.orientation = Quaternion(*quaternion_from_euler(0,  0, self.theta))

        # Add the PoseStamped message to the Path
        path.poses.append(pose_stamped)

        # Publish the path
        self.path_pub.publish(path)

    def publish_tf(self, br = tf2_ros.TransformBroadcaster()):
        
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "/odom"
        t.child_frame_id = "/base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(*quaternion_from_euler(0,  0, self.theta))
        br.sendTransform(t)
        

    def run(self):
        rate = rospy.Rate(10)  #  10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry = ForkliftOdometryAM92()
        odometry.run()
    except rospy.ROSInterruptException:
        pass
