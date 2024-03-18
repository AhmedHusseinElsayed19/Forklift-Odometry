##########first code trial##############


#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

class ForkliftOdometry:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        
        # Wheelbase length in meters
        self.WHEELBASE = 1.46
        
        # Initialize variables for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        # 'can_messages' should be replaced with the actual topic name that is produced when SocketCAN is running.
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != 0x31f:
            return

        # Extract wheel travel and steering angle from frame data
        wheel_travel = ((ord(msg.data[1]) << 8) | ord(msg.data[0])) / 1000.0  # convert from mm to m
        steering_angle = ((ord(msg.data[3]) << 8) | ord(msg.data[2])) / 100.0 * math.pi / 180  # convert to radians

        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Update position and orientation using kinematic equations
        self.x += wheel_travel * math.cos(self.theta) * dt
        self.y += wheel_travel * math.sin(self.theta) * dt
        self.theta += ((wheel_travel * math.tan(steering_angle))/ self.WHEELBASE) * dt

        # Broadcast the odometry transform
        self.broadcast_odometry_transform()

        # Prepare and publish odometry message
        odom = self.prepare_odometry_message(wheel_travel, steering_angle, dt, current_time)
        self.odom_pub.publish(odom)

    def broadcast_odometry_transform(self):
        # Create odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Set translation and rotation values
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, wheel_travel, steering_angle, dt, current_time):
        # Create quaternion from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the velocity
        velocity = wheel_travel / dt
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.theta), velocity * math.sin(self.theta), 0),
                                 Vector3(0, 0, ((velocity * math.tan(steering_angle)) / self.WHEELBASE)))

        return odom

if __name__ == '__main__':
    try:
        ForkliftOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


######## code trial with a flag #############


#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

class ForkliftOdometry1:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        # Wheelbase length in meters
        self.WHEELBASE = 1.46
        # Initialize variables for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        # 'can_messages' should be replaced with the actual topic name that is produced when SocketCAN is running.
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

        # Flag to track if a new frame has been received
        self.new_frame_received = False

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != 0x31f:
            return

        # Extract wheel travel and steering angle from frame data
        self.wheel_travel = ((ord(msg.data[1]) << 8) | ord(msg.data[0])) / 1000.0  # convert from mm to m
        self.steering_angle = ((ord(msg.data[3]) << 8) | ord(msg.data[2])) / 100.0 * math.pi / 18000  # convert to radians

        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Update position and orientation using kinematic equations
        self.x += self.wheel_travel * math.cos(self.theta) * dt
        self.y += self.wheel_travel * math.sin(self.theta) * dt
        self.theta += ((self.wheel_travel * math.tan(self.steering_angle))/ self.WHEELBASE) * dt

        # Broadcast the odometry transform
        self.broadcast_odometry_transform()

        # Set the new_frame_received flag to True
        self.new_frame_received = True

    def broadcast_odometry_transform(self):
        # Create odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Set translation and rotation values
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, dt, current_time):
        # Create quaternion from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the velocity
        velocity = self.wheel_travel / dt
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.theta), velocity * math.sin(self.theta), 0),
                                 Vector3(0, 0, ((velocity * math.tan(self.steering_angle)) / self.WHEELBASE)))

        return odom

    def publish_odometry(self):
        if self.new_frame_received:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            odom = self.prepare_odometry_message(dt, current_time)
            self.odom_pub.publish(odom)  # Publish odometry
            self.last_time = current_time
            self.new_frame_received = False  # Reset the flag

if __name__ == '__main__':
    try:
        forklift_odometry = ForkliftOdometry1()
        while not rospy.is_shutdown():
            forklift_odometry.publish_odometry()
            rospy.sleep(0.01)  # Sleep to control the loop rate
    except rospy.ROSInterruptException:
        pass


################# Code trial with change in CAN frames (with better understanding *bits in frame change*) #####################


#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

class ForkliftOdometry2:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        # Wheelbase length in meters
        self.WHEELBASE = 1.46
        # Initialize variables for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        # 'can_messages' should be replaced with the actual topic name that is produced when SocketCAN is running.
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

        # Flag to track if a new frame has been received
        self.new_frame_received = False

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != 0x31f:
            return

        # Extract wheel travel and steering angle from frame data
        self.wheel_travel = ((ord(msg.data[2]) << 8) | ord(msg.data[1])) / 1000.0  # convert from mm to m
        self.steering_angle = ((ord(msg.data[4]) << 8) | ord(msg.data[3])) / 100.0 * math.pi / 18000  # convert to radians

        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # Update position and orientation using kinematic equations
        self.x += self.wheel_travel * math.cos(self.theta) * dt
        self.y += self.wheel_travel * math.sin(self.theta) * dt
        self.theta += ((self.wheel_travel * math.tan(self.steering_angle))/ self.WHEELBASE) * dt

        # Broadcast the odometry transform
        self.broadcast_odometry_transform()

        # Set the new_frame_received flag to True
        self.new_frame_received = True

    def broadcast_odometry_transform(self):
        # Create odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Set translation and rotation values
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, dt, current_time):
        # Create quaternion from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the velocity
        velocity = self.wheel_travel / dt
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.theta), velocity * math.sin(self.theta), 0),
                                 Vector3(0, 0, ((velocity * math.tan(self.steering_angle)) / self.WHEELBASE)))

        return odom

    def publish_odometry(self):
        if self.new_frame_received:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            odom = self.prepare_odometry_message(dt, current_time)
            self.odom_pub.publish(odom)  # Publish odometry
            self.last_time = current_time
            self.new_frame_received = False  # Reset the flag

if __name__ == '__main__':
    try:
        forklift_odometry = ForkliftOdometry2()
        while not rospy.is_shutdown():
            forklift_odometry.publish_odometry()
            rospy.sleep(0.01)  # Sleep to control the loop rate
    except rospy.ROSInterruptException:
        pass
#################################### My Trial ######################################

#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

class ForkliftOdometry3:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        
        # Wheelbase length in meters
        self.WHEELBASE = 1.46

        # Frame ID in hexadecimal
        self.FRAME_ID = 0x31f
        
        # Initialize variables for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        # 'can_messages' should be replaced with the actual topic name that is produced when SocketCAN is running.
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

        # Flag to track if a new frame has been received
        self.new_frame_received = False

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != self.FRAME_ID:
            return

        byte1 = ord(msg.data[0])
        byte2 = ord(msg.data[1])
        byte3 = ord(msg.data[2])
        byte4 = ord(msg.data[3])

        self.wheel_travel = (byte1 + (byte2 << 8)) % 65535 / 1000.0  # Convert to meters
        self.steering_angle = ((byte3 + (byte4 << 8)) - 18000) / 100.0 * (math.pi / 180)  # Convert to radians
        print('wheel travel = ',self.wheel_travel)
        print('Steering angle = ',self.steering_angle)
        print('byte1 = ',byte1)
        print('byte2 = ',byte2)
        print('byte3 = ',byte3)
        print('byte4 = ',byte4)
        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # Update position and orientation using kinematic equations
        self.x_dot = self.wheel_travel * math.cos(self.steering_angle)
        self.y_dot = self.wheel_travel * math.sin(self.steering_angle)
        self.theta_dot = ((self.wheel_travel * math.tan(self.steering_angle)) / self.WHEELBASE)
        self.x = self.x_dot * dt
        self.y = self.y_dot * dt
        self.theta = self.theta_dot *dt

        # Broadcast the odometry transform
        self.broadcast_odometry_transform()

        # Set the new_frame_received flag to True
        self.new_frame_received = True

    def broadcast_odometry_transform(self):
        # Create odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Set translation and rotation values
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, dt, current_time):
        # Create quaternion from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the velocity
        velocity = self.wheel_travel / dt  #value is in m/s
        print('Velocity is = ',velocity)
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.steering_angle), velocity * math.sin(self.steering_angle), 0),
                                 Vector3(0, 0, ((velocity * math.tan(self.steering_angle)) / self.WHEELBASE)))

        return odom

    def publish_odometry(self):
        if self.new_frame_received:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            odom = self.prepare_odometry_message(dt, current_time)
            # Publish odometry
            self.odom_pub.publish(odom)  
            self.last_time = current_time
            # Reset the flag
            self.new_frame_received = False  

if __name__ == '__main__':
    try:
        forklift_odometry = ForkliftOdometry3()
        while not rospy.is_shutdown():
            forklift_odometry.publish_odometry()
            # Sleep to control the loop rate
            rospy.sleep(0.01)  
    except rospy.ROSInterruptException:
        pass

###################### One before last trial ##############################

#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

class ForkliftOdometry7:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        
        # Wheelbase length in meters
        self.WHEELBASE = 1.46

        # Frame ID in hexadecimal
        self.FRAME_ID = 0x31f
        
        # Initialize variables for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

        # Flag to track if a new frame has been received
        self.new_frame_received = False

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != self.FRAME_ID:
            return

        byte1 = ord(msg.data[0])
        byte2 = ord(msg.data[1])
        byte3 = ord(msg.data[2])
        byte4 = ord(msg.data[3])
        
        # Decode CAN frame data from hexadecimals to decimals
        self.wheel_travel = (byte1 + (byte2 << 8)) % 65535 / 1000.0  # Convert to meters
        self.steering_angle = ((byte3 + (byte4 << 8)) - 18000) / 100.0 * (math.pi / 180)  # Convert to radians
        
        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # Update position and orientation using kinematic equations
        self.x_dot = self.wheel_travel * math.cos(self.steering_angle)
        self.y_dot = self.wheel_travel * math.sin(self.steering_angle)
        self.theta_dot = ((self.wheel_travel * math.tan(self.steering_angle)) / self.WHEELBASE)
        self.x = self.x_dot * dt
        self.y = self.y_dot * dt
        self.theta = self.theta_dot * dt

        # Broadcast the odometry transform
        self.broadcast_odometry_transform()

        # Set the new_frame_received flag to True
        self.new_frame_received = True

    def broadcast_odometry_transform(self):
        # Create odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Set translation and rotation values
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, dt, current_time):
        # Create quaternion from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # Set the velocity
        velocity = self.wheel_travel / dt  #value is in m/s
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.steering_angle), velocity * math.sin(self.steering_angle), 0),
                                 Vector3(0, 0, ((velocity * math.tan(self.steering_angle)) / self.WHEELBASE)))

        return odom

    def publish_odometry(self):
        if self.new_frame_received:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            odom = self.prepare_odometry_message(dt, current_time)
            # Publish odometry
            self.odom_pub.publish(odom)  
            self.last_time = current_time
            # Reset the flag
            self.new_frame_received = False  

if __name__ == '__main__':
    try:
        forklift_odometry = ForkliftOdometry7()
        while not rospy.is_shutdown():
            forklift_odometry.publish_odometry()
            # Sleep to control the loop rate
            rospy.sleep(0.01)  
    except rospy.ROSInterruptException:
        pass

##################### Last Trial ##########################################

#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped

class ForkliftOdometry4:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        
        # Distance between the two wheels of the forklift
        self.WHEELBASE = 1.46

        # ID of the frame for which to calculate odometry
        self.FRAME_ID = 0x31f

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Previous value of wheel travel
        self.prev_wheel_travel = 0.0  

        # Previous value of steering angle
        self.prev_steering_angle = 0.0  

        # Publisher for odometry data
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscriber for frame messages
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)
        
        # TF broadcaster for transforming coordinates
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Time of the last frame received
        self.last_time = rospy.Time.now()
        
        # Flag to indicate if a new frame is received
        self.new_frame_received = False

    def handle_frame(self, msg):
        if msg.id != self.FRAME_ID:
            return
        
        # Decode CAN frame data from hexadecimals to decimals
        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])
        
        # Extract wheel travel value from frame
        self.wheel_travel = ((byte1 | (byte2 << 8)) % 216) / 1000.0  

        # Extract steering angle from frame
        self.steering_angle = ((byte3 | (byte4 << 8))) * 0.01 * 0.545 * (math.pi / 180)  
     

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # Introduce values to the kinematics equations
        self.x_dot = self.wheel_travel * math.cos(self.steering_angle)
        self.y_dot = self.wheel_travel * math.sin(self.steering_angle)
        self.theta_dot = ((self.wheel_travel * math.tan(self.steering_angle)) / self.WHEELBASE)
    
        if self.prev_wheel_travel == self.wheel_travel and self.prev_steering_angle == self.steering_angle:
            # If wheel travel and steering angle haven't changed, only update position based on time
            self.x = self.x_dot * dt
            self.y = self.y_dot * dt
            self.theta = self.theta_dot * dt
        else:
            # If wheel travel or steering angle has changed, update position based on accumulated travel
            self.x += self.x_dot * dt
            self.y += self.y_dot * dt
            self.theta += self.theta_dot * dt
        
        # Publish the odometry transform
        self.broadcast_odometry_transform()
        self.new_frame_received = True
        
        # Assign the wheel travel and steering angle data to the previous variables when a new frame is received
        self.prev_wheel_travel = self.wheel_travel
        self.prev_steering_angle = self.steering_angle

    def broadcast_odometry_transform(self):
        # Create a TransformStamped message to broadcast the odometry transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Get translation motion from X-Y co-ordinates
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        # Get rotation motion from Quaternion transformation
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, dt, current_time):
        # Prepare an Odometry message with the calculated odometry data
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))
       
        # Linear velocity of the forklift
        velocity = self.wheel_travel # velocity in m/s and it is not divided by dt because dt is too small and it will effect the value.
       
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.steering_angle), velocity * math.sin(self.steering_angle), 0),
                                 Vector3(0, 0, ((velocity * math.tan(self.steering_angle)) / self.WHEELBASE)))
        return odom

    def publish_odometry(self):
        if self.new_frame_received:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            odom = self.prepare_odometry_message(dt, current_time)
            self.odom_pub.publish(odom)
            self.last_time = current_time
            self.new_frame_received = False

if __name__ == "__main__":
    forklift_odometry = ForkliftOdometry4()
    rospy.spin()


################################ Runge-Kutta Trial ###########################

#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

class ForkliftOdometry8:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        
        # Wheelbase length in meters
        self.WHEELBASE = 1.46

        # Frame ID in hexadecimal
        self.FRAME_ID = 0x31f
        
        # Initialize variables for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Previous value of wheel travel
        self.prev_wheel_travel = None 

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

        # Flag to track if a new frame has been received
        self.new_frame_received = False

    def x_dot(self, velocity, steering_angle):
        return velocity * math.cos(steering_angle)

    def y_dot(self, velocity, steering_angle):
        return velocity * math.sin(steering_angle)

    def theta_dot(self, velocity, steering_angle):
        return (velocity * math.tan(steering_angle)) / self.WHEELBASE

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != self.FRAME_ID:
            return

        byte1 = ord(msg.data[0])
        byte2 = ord(msg.data[1])
        byte3 = ord(msg.data[2])
        byte4 = ord(msg.data[3])
        
        # Decode CAN frame data from hexadecimals to decimals
        self.wheel_travel = (byte1 + (byte2 << 8)) % 216 / 1000.0  # Convert to meters
        self.steering_angle = ((byte3 + (byte4 << 8)) - 18000) / 100.0 * (math.pi / 180)  # Convert to radians
        
        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Calculate velocity
        if self.prev_wheel_travel is not None:
            velocity = (self.wheel_travel - self.prev_wheel_travel) / dt
        else:
            velocity = 0.0

        self.prev_wheel_travel = self.wheel_travel

        # Calculate the k1 coefficients
        k1_x = dt * self.x_dot(velocity, self.steering_angle)
        k1_y = dt * self.y_dot(velocity, self.steering_angle)
        k1_theta = dt * self.theta_dot(velocity, self.steering_angle)

        # Calculate the k2 coefficients
        k2_x = dt * self.x_dot(velocity + 0.5 * k1_x, self.steering_angle)
        k2_y = dt * self.y_dot(velocity + 0.5 * k1_y, self.steering_angle)
        k2_theta = dt * self.theta_dot(velocity + 0.5 * k1_theta, self.steering_angle)

        # Calculate the k3 coefficients
        k3_x = dt * self.x_dot(velocity + 0.5 * k2_x, self.steering_angle)
        k3_y = dt * self.y_dot(velocity + 0.5 * k2_y, self.steering_angle)
        k3_theta = dt * self.theta_dot(velocity + 0.5 * k2_theta, self.steering_angle)

        # Calculate the k4 coefficients
        k4_x = dt * self.x_dot(velocity + k3_x, self.steering_angle)
        k4_y = dt * self.y_dot(velocity + k3_y, self.steering_angle)
        k4_theta = dt * self.theta_dot(velocity + k3_theta, self.steering_angle)

        
        # Update position and orientation using the Runge-Kutta method
        self.x += (k1_x + 2 * k2_x + 2 * k3_x + k4_x) / 6
        self.y += (k1_y + 2 * k2_y + 2 * k3_y + k4_y) / 6
        self.theta += (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta) / 6

        # Broadcast the odometry transform
        self.broadcast_odometry_transform()

        # Set the new_frame_received flag to True
        self.new_frame_received = True

    def broadcast_odometry_transform(self):
        # Create odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Set translation and rotation values
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, dt, current_time):
      
        # Create quaternion from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

        # Set the velocity
        velocity = (self.wheel_travel - self.prev_wheel_travel) / dt # type: ignore
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.steering_angle), velocity * math.sin(self.steering_angle), 0),
                                 Vector3(0, 0, ((velocity * math.tan(self.steering_angle)) / self.WHEELBASE)))

        return odom

    def publish_odometry(self):
        if self.new_frame_received:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            odom = self.prepare_odometry_message(dt, current_time)
            # Publish odometry
            self.odom_pub.publish(odom)  
            self.last_time = current_time
            # Reset the flag
            self.new_frame_received = False  

if __name__ == '__main__':
    try:
        forklift_odometry = ForkliftOdometry8()
        while not rospy.is_shutdown():
            forklift_odometry.publish_odometry()
            # Sleep to control the loop rate
            rospy.sleep(0.01)  
    except rospy.ROSInterruptException:
        pass


################################### Final Trial #######################################


#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

class ForkliftOdometry9:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        
        # Wheelbase length in meters
        self.WHEELBASE = 1.46

        # Frame ID in hexadecimal
        self.FRAME_ID = 0x31f
        
        # Initialize variables for position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Previous value of wheel travel
        self.prev_wheel_travel = None 

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

        # Flag to track if a new frame has been received
        self.new_frame_received = False

    def x_dot(self, velocity, steering_angle):
        return velocity * math.cos(steering_angle)

    def y_dot(self, velocity, steering_angle):
        return velocity * math.sin(steering_angle)

    def theta_dot(self, velocity, steering_angle):
        return (velocity * math.tan(steering_angle)) / self.WHEELBASE

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != self.FRAME_ID:
            return

        byte1 = ord(msg.data[0])
        byte2 = ord(msg.data[1])
        byte3 = ord(msg.data[2])
        byte4 = ord(msg.data[3])
        
        # Decode CAN frame data from hexadecimals to decimals
        self.wheel_travel = (byte1 + (byte2 << 8)) % 216 / 1000.0  # Convert to meters
        self.steering_angle = ((byte3 + (byte4 << 8)) - 18000) / 100.0 * (math.pi / 180)  # Convert to radians
        
        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Calculate velocity
        
        velocity = self.wheel_travel / dt

        # Calculate the k1 coefficients
        k1_x = dt * self.x_dot(velocity, self.steering_angle)
        k1_y = dt * self.y_dot(velocity, self.steering_angle)
        k1_theta = dt * self.theta_dot(velocity, self.steering_angle)

        # Calculate the k2 coefficients
        k2_x = dt * self.x_dot(velocity + 0.5 * k1_x, self.steering_angle)
        k2_y = dt * self.y_dot(velocity + 0.5 * k1_y, self.steering_angle)
        k2_theta = dt * self.theta_dot(velocity + 0.5 * k1_theta, self.steering_angle)

        # Calculate the k3 coefficients
        k3_x = dt * self.x_dot(velocity + 0.5 * k2_x, self.steering_angle)
        k3_y = dt * self.y_dot(velocity + 0.5 * k2_y, self.steering_angle)
        k3_theta = dt * self.theta_dot(velocity + 0.5 * k2_theta, self.steering_angle)

        # Calculate the k4 coefficients
        k4_x = dt * self.x_dot(velocity + k3_x, self.steering_angle)
        k4_y = dt * self.y_dot(velocity + k3_y, self.steering_angle)
        k4_theta = dt * self.theta_dot(velocity + k3_theta, self.steering_angle)

        
        # Update position and orientation using the Runge-Kutta method
        self.x += (k1_x + 2 * k2_x + 2 * k3_x + k4_x) / 6
        self.y += (k1_y + 2 * k2_y + 2 * k3_y + k4_y) / 6
        self.theta += (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta) / 6

        # Broadcast the odometry transform
        self.broadcast_odometry_transform()

        # Set the new_frame_received flag to True
        self.new_frame_received = True

    def broadcast_odometry_transform(self):
        # Create odometry transform message
        odom_trans = TransformStamped()
        odom_trans.header.stamp = rospy.Time.now()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"

        # Set translation and rotation values
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)
        odom_trans.transform.rotation = Quaternion(*odom_quat)

        # Broadcast the odometry transform
        self.tf_broadcaster.sendTransform(odom_trans)

    def prepare_odometry_message(self, dt, current_time):
      
        # Create quaternion from yaw
        odom_quat = transformations.quaternion_from_euler(0, 0, self.theta)

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # Set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*odom_quat))

        # Set the velocity
        velocity =(self.wheel_travel - (self.prev_wheel_travel) )/ dt  # type: ignore
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(velocity * math.cos(self.steering_angle), velocity * math.sin(self.steering_angle), 0),
                                 Vector3(0, 0, ((velocity * math.tan(self.steering_angle)) / self.WHEELBASE)))

        return odom

    def publish_odometry(self):
        if self.new_frame_received:
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            odom = self.prepare_odometry_message(dt, current_time)
            
            # Publish odometry
            self.odom_pub.publish(odom)  
            self.last_time = current_time
           
            # Reset the flag
            self.new_frame_received = False  

if __name__ == '__main__':
    try:
        forklift_odometry = ForkliftOdometry9()
        while not rospy.is_shutdown():
            forklift_odometry.publish_odometry()
            
            # Sleep to control the loop rate
            rospy.sleep(0.01)  
    except rospy.ROSInterruptException:
        pass


    ###################### Last Trial Before Testing ######################

#!/usr/bin/env python

import math
import rospy
import tf2_ros
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from tf import transformations
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped 

# Constants
WHEELBASE = 1.46 # Distance in meters
FRAME_ID = 799

class ForkliftOdometry12:
    
    def __init__(self):
        rospy.init_node('forklift_odometry')
        
        # Initialize variables for position, orientation, and wheel travel
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_theta = 0.0
        self.wheel_travel = 0.0
        self.steering_angle = 0.0

        # Create publisher for odometry messages
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Subscribe to CAN frame messages 
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)

        # Create a broadcaster for the odometry transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize the last callback time
        self.last_time = rospy.Time.now()

        # Flag to track if a new frame has been received
        self.new_frame_received = False

    def x_dot(self, velocity, steering_angle):
        return velocity * math.cos(steering_angle)

    def y_dot(self, velocity, steering_angle):
        return velocity * math.sin(steering_angle)

    def theta_dot(self, velocity, steering_angle):
        return (velocity * math.tan(steering_angle)) / WHEELBASE

    def handle_frame(self, msg):
        # Only handle frame with ID 0x31F
        if msg.id != FRAME_ID:
            return

        # Check if msg.data has at least four bytes
        if len(msg.data) < 4:
            rospy.logwarn("Received frame with less than four bytes. Ignoring frame.")
            return

        byte1 = ord(msg.data[0])
        byte2 = ord(msg.data[1])
        byte3 = ord(msg.data[2])
        byte4 = ord(msg.data[3])
        
        # Decode CAN frame data from hexadecimals to decimals
        self.wheel_travel = (byte1 + (byte2 << 8)) % 216 / 1000.0  # Convert to meters
        self.steering_angle = ((byte3 + (byte4 << 8)) - 18000) / 100.0 * (math.pi / 180)  # Convert to radians
        
        # Calculate time difference
        current_time = msg.header.stamp 
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Calculate velocity
        velocity = self.wheel_travel / dt
        
        print('wheel travel = ',self.wheel_travel)
        print('Steering angle = ',self.steering_angle)
        print('byte1 = ',byte1)
        print('byte2 = ',byte2)
        print('byte3 = ',byte3)
        print('byte4 = ',byte4)
        
        # Introduce Runge-Kutta method calculation
        # Calculate the k1 coefficients
        k1_x = dt * self.x_dot(velocity, self.steering_angle)
        k1_y = dt * self.y_dot(velocity, self.steering_angle)
        k1_theta = dt * self.theta_dot(velocity, self.steering_angle)

        # Calculate the k2 coefficients
        k2_x = dt * self.x_dot(velocity + 0.5 * k1_x, self.steering_angle)
        k2_y = dt * self.y_dot(velocity + 0.5 * k1_y, self.steering_angle)
        k2_theta = dt * self.theta_dot(velocity + 0.5 * k1_theta, self.steering_angle)

        # Calculate the k3 coefficients
        k3_x = dt * self.x_dot(velocity + 0.5 * k2_x, self.steering_angle)
        k3_y = dt * self.y_dot(velocity + 0.5 * k2_y, self.steering_angle)
        k3_theta = dt * self.theta_dot(velocity + 0.5 * k2_theta, self.steering_angle)
        
        # Calculate the k4 coefficients
        k4_x = dt * self.x_dot(velocity + k3_x, self.steering_angle)
        k4_y = dt * self.y_dot(velocity + k3_y, self.steering_angle)
        k4_theta = dt * self.theta_dot(velocity + k3_theta, self.steering_angle)

        # Update position and orientation using the Runge-Kutta method
        self.position_x += (k1_x + 2 * k2_x + 2 * k3_x + k4_x) / 6
        self.position_y += (k1_y + 2 * k2_y + 2 * k3_y + k4_y) / 6
        self.orientation_theta += (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta) / 6

        self.new_frame_received = True

    def publish_odometry(self):
        if not self.new_frame_received:
            return

        # Create a new odometry message
        odom = Odometry()

        # Set the header
        odom.header.stamp = self.last_time
        odom.header.frame_id = 'odom'

        # Set the pose (position and orientation)
        odom.pose.pose = Pose(Point(self.position_x, self.position_y, 0.), Quaternion(*transformations.quaternion_from_euler(0, 0, self.orientation_theta)))

        # Set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist = Twist(Vector3(self.wheel_travel, 0, 0), Vector3(0, 0, self.theta_dot(self.wheel_travel, self.steering_angle)))

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Also publish the transform over tf
        transform = TransformStamped()
        transform.header.stamp = odom.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.position_x
        transform.transform.translation.y = self.position_y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        self.new_frame_received = False

if __name__ == '__main__':
    try:
        forklift_odometry = ForkliftOdometry12()
        while not rospy.is_shutdown():
            forklift_odometry.publish_odometry()
            
            # Sleep to control the loop rate
            rospy.sleep(0.01)  
    except rospy.ROSInterruptException:
        pass


########################### Best Version till now #########################################

#!/usr/bin/env python

import math
import rospy
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, TransformStamped
from tf.transformations import quaternion_from_euler

WHEELWIDTH = 1.238 # Distance between two rear wheels of the forklift

class ForkliftOdometryBonus:
    def __init__(self):
        rospy.init_node('forklift_odometry')
        self.WHEELBASE = 1.46  # Distance between the front and rear wheels of the forklift
        self.FRAME_ID = 799  # ID of the frame for which to calculate odometry
        self.x = 0.0  # X-coordinate of the forklift's position
        self.y = 0.0  # Y-coordinate of the forklift's position
        self.theta = 0.0  # Orientation angle of the forklift
        self.wheel_travel = 0.0  # Distance traveled by the wheels
        self.steering_angle = 0.0  # Steering angle of the forklift
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)  # Publisher for odometry data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)  # Subscriber for frame messages
        self.last_time = rospy.Time.now()  # Time of the last frame received

    def handle_frame(self, msg):
        # Check if the received frame ID matches the desired frame ID
        if msg.id != self.FRAME_ID:
            return

        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])

        prev_wheel_travel = self.wheel_travel
        prev_steering_angle = self.steering_angle

        # Extract wheel travel value from frame in meters
        self.wheel_travel = ((byte1 + (byte2 << 8))) / 1000.0

        # Extract steering angle from frame in radians
        self.steering_angle = ((byte3 + (byte4 << 8) - 18000) / 100.0) * (math.pi / 180)

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Check if the wheel travel or steering angle has changed
        if prev_wheel_travel != self.wheel_travel or prev_steering_angle != self.steering_angle:
            # Calculate linear and angular velocities
            linear_velocity = self.wheel_travel / dt
            angular_velocity = (linear_velocity * math.tan(self.steering_angle)) / self.WHEELBASE

            # Update position and orientation using forward kinematics
            delta_theta = angular_velocity * dt
            delta_x = linear_velocity * math.cos(self.theta + delta_theta / 2) * dt
            delta_y = linear_velocity * math.sin(self.theta + delta_theta / 2) * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Publish the odometry message
            self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
        odom.twist.twist = Twist(Vector3(linear_velocity, 0.0, 0.0), Vector3(0.0, 0.0, angular_velocity))

        # Publish the odometry message
        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry = ForkliftOdometryBonus()
        odometry.run()
    except rospy.ROSInterruptException:
        pass

####################################################### Most Realistic ##################################################
    
#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two frames
STEERING_ANGLE_FACTOR = 0.01 # Maximum expected change in steering angle between two frames

class ForkliftOdometryB:
   def __init__(self):
       rospy.init_node('forklift_odometry')
       
       self.x = 0.0 # X-coordinate of the forklift's position
       self.y = 0.0 # Y-coordinate of the forklift's position
       self.theta = 0.0 # Orientation angle of the forklift
       self.wheel_travel = 0.0 # Distance traveled by the wheels
       self.steering_angle = 0.0 # Steering angle of the forklift
       self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
       self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
       self.last_time = rospy.Time.now() # Time of the last frame received

   def handle_frame(self, msg):
       # Check if the received frame ID matches the desired frame ID
       if msg.id != FRAME_ID:
           return

       byte1 = (msg.data[0])
       byte2 = (msg.data[1])
       byte3 = (msg.data[2])
       byte4 = (msg.data[3])
      

       prev_wheel_travel = self.wheel_travel
       prev_steering_angle = self.steering_angle

       # Extract wheel travel value from frame in meters
       self.wheel_travel = ((byte1 + (byte2 << 8)) % 65536) / 1000.0
       if self.wheel_travel < prev_wheel_travel:
           # Handle rollover
           self.wheel_travel += 65536 / 1000.0

       # Extract steering angle from frame in radians
       self.steering_angle = (byte3 + (byte4 << 8)) * STEERING_ANGLE_FACTOR * (math.pi / 180)

       current_time = rospy.Time.now()
       dt = (current_time - self.last_time).to_sec()
       self.last_time = current_time

       # Ignore large changes in wheel travel or steering angle
       if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
           return

       # Calculate linear and angular velocities
       linear_velocity = (self.wheel_travel-prev_wheel_travel) / dt
       angular_velocity = (linear_velocity * math.tan(self.steering_angle)) / WHEELBASE

       # Update position and orientation using forward kinematics
       delta_theta = angular_velocity * dt
       delta_x = linear_velocity * math.cos(self.theta ) * dt
       delta_y = linear_velocity * math.sin(self.theta ) * dt

       
       if self.wheel_travel != prev_wheel_travel or self.steering_angle != prev_steering_angle:
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
    
       else:
        self.x = delta_x
        self.y = delta_y
        self.theta = delta_theta

       # Publish the odometry message
       self.publish_odometry(linear_velocity, angular_velocity)

   def publish_odometry(self, linear_velocity, angular_velocity):
       current_time = rospy.Time.now()

       # Prepare an Odometry message with the calculated odometry data
       odom = Odometry()
       odom.header.stamp = current_time
       odom.header.frame_id = 'odom'
       odom.child_frame_id = 'base_link'
       odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
       odom.twist.twist = Twist(Vector3(linear_velocity, 0.0, 0.0), Vector3(0.0, 0.0, angular_velocity))

       # Publish the odometry message
       self.odom_pub.publish(odom)

   def run(self):
       rate = rospy.Rate(10) # 10 Hz
       while not rospy.is_shutdown():
           rate.sleep()

if __name__ == '__main__':
   try:
       odometry = ForkliftOdometryB()
       odometry.run()
   except rospy.ROSInterruptException:
       pass
   

########################################################## Most Accurate ###########################################################

#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two frames
STEERING_ANGLE_FACTOR = 0.01 # Steering factor to get the correct steering angle *DataSheet*


class ForkliftOdometryD:
   def __init__(self):
       rospy.init_node('forklift_odometry')
       
       self.x = 0.0 # X-coordinate of the forklift's position
       self.y = 0.0 # Y-coordinate of the forklift's position
       self.theta = 0.0 # Orientation angle of the forklift
       self.wheel_travel = 0.0 # Distance traveled by the wheels
       self.steering_angle_convert = 0.0 # Steering angle of the forklift before conversion
       self.steering_angle = 0.0 # Steering angle of the forklift
       self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
       self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
       self.last_time = rospy.Time.now() # Time of the last frame received

   def handle_frame(self, msg):
       # Check if the received frame ID matches the desired frame ID
       if msg.id != FRAME_ID:
           return
       
      # Store the timestamp of the last received frame
       last_time = self.last_time

      # Get the current time
       current_time = rospy.Time.now()

      # Calculate the time difference
       dt = (current_time - last_time).to_sec()

      # Store the current time as the timestamp of the last received frame
       self.last_time = current_time


       byte1 = (msg.data[0])
       byte2 = (msg.data[1])
       byte3 = (msg.data[2])
       byte4 = (msg.data[3])
      

       prev_wheel_travel = self.wheel_travel
      

       # Extract wheel travel value from frame in meters
       self.wheel_travel = (byte1 + (byte2 << 8)) / 1000.0
      

       # Extract steering angle from frame in radians
       self.steering_angle_convert = (byte3 + (byte4 << 8)) * STEERING_ANGLE_FACTOR * (math.pi / 180)


       # Ignore large changes in wheel travel or steering angle
       if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
           return

      # Calculate linear and angular velocities
       linear_velocity = (self.wheel_travel-prev_wheel_travel) / dt
       angular_velocity = (linear_velocity * math.atan(self.steering_angle)) / WHEELBASE

      # Update position and orientation using forward kinematics
       delta_theta = angular_velocity * dt 
       delta_x = linear_velocity * math.cos(self.theta) * dt
       delta_y = linear_velocity * math.sin(self.theta) * dt

       self.x += delta_x
       self.y += delta_y
       self.theta += delta_theta

       # Publish the odometry message
       self.publish_odometry(linear_velocity, angular_velocity)

   def publish_odometry(self, linear_velocity, angular_velocity):
       current_time = rospy.Time.now()

       # Prepare an Odometry message with the calculated odometry data
       odom = Odometry()
       odom.header.stamp = current_time
       odom.header.frame_id = 'odom'
       odom.child_frame_id = 'base_link'
       odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
       odom.twist.twist = Twist(Vector3(linear_velocity, 0.0, 0.0), Vector3(0.0, 0.0, angular_velocity))

       # Publish the odometry message
       self.odom_pub.publish(odom)

   def run(self):
       rate = rospy.Rate(10) # 10 Hz
       while not rospy.is_shutdown():
           rate.sleep()

if __name__ == '__main__':
   try:
       odometry = ForkliftOdometryD()
       odometry.run()
   except rospy.ROSInterruptException:
       pass
   

################################################## El7mdullah ########################################
#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR = 0.01 # Steering factor to get the correct steering angle *DataSheet*
THRESHOLD = 0.001 # Threshold value for updating position

class ForkliftOdometryX:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x = 0.0 # X-coordinate of the forklift's position
        self.y = 0.0 # Y-coordinate of the forklift's position
        self.theta = 0.0 # Orientation angle of the forklift
        self.wheel_travel = 0.0 # Distance traveled by the wheels
        self.steering_angle_convert = 0.0 # Steering angle of the forklift after conversion
        self.steering_angle = 0.0 # Steering angle of the forklift
        self.rotation_angle = 0.0 # Rotation angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
        self.last_time = rospy.Time.now() # Time of the last frame received

    def handle_frame(self, msg):
        # Check if the received frame ID matches the desired frame ID
        if msg.id != FRAME_ID:
            return

        # Store the timestamp of the last received frame
        last_time = self.last_time

        # Get the current time
        current_time = rospy.Time.now()

        # Calculate the time difference
        dt = (current_time - last_time).to_sec()

        # Store the current time as the timestamp of the last received frame
        self.last_time = current_time

        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])

        prev_wheel_travel = self.wheel_travel

        # Extract wheel travel value from frame in meters
        self.wheel_travel = (byte1 + (byte2 << 8))
        self.wheel_travel /= 1000.0

        # Extract steering angle value from frame in degrees
        self.steering_angle = (byte3 + (byte4 << 8)) * STEERING_ANGLE_FACTOR

        # Convert steering angle to radians
        self.steering_angle_convert = self.steering_angle * (math.pi / 18000)

        # Ignore large changes in wheel travel or steering angle
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
            return

        # Calculate linear and angular velocities
        linear_velocity = (self.wheel_travel - prev_wheel_travel) / dt
        angular_velocity = (linear_velocity * math.tan(self.steering_angle_convert)) / WHEELBASE

        # Update position and orientation using kinematics equations
        delta_theta = angular_velocity * dt
        delta_x = linear_velocity * math.cos(self.steering_angle_convert) * dt
        delta_y = linear_velocity * math.sin(self.steering_angle_convert) * dt

        # Only update x position if delta_x is greater than the threshold
        if abs(delta_x) > THRESHOLD:
            self.x += delta_x

        # Only update y position if delta_y is greater than the threshold
        elif abs(delta_y) > THRESHOLD:
            self.y += delta_y

        self.theta += delta_theta

        # Publish the odometry message
        self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
        odom.twist.twist = Twist(Vector3(linear_velocity, 0.0, 0.0), Vector3(0.0, 0.0, angular_velocity))

        # Publish the odometry message
        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep
    
if __name__ == '__main__':
  try:
      odometry = ForkliftOdometryX()
      odometry.run()
  except rospy.ROSInterruptException:
      pass

############################################# Failed Trial ######################################################
  
#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46  # Distance between the front and rear wheels of the forklift
FRAME_ID = 799  # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1  # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR = 0.01  # Steering factor to get the correct steering angle *DataSheet*
TURNING_RADIUS_FACTOR = 1.46 / 2  # Factor to calculate the turning radius from the wheel base


class ForkliftOdometryF:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x = 0.0  # X-coordinate of the forklift's position
        self.y = 0.0  # Y-coordinate of the forklift's position
        self.theta = 0.0  # Orientation angle of the forklift
        self.wheel_travel = 0.0  # Distance traveled by the wheels
        self.steering_angle_convert = 0.0  # Steering angle of the forklift after conversion
        self.v_x = 0.0  # X-velocity of the forklift
        self.v_y = 0.0  # Y-velocity of the forklift
        self.kappa = 0.0  # Turn radius of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)  # Publisher for odometry data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)  # Subscriber for frame messages
        self.last_time = rospy.Time.now()  # Time of the last frame received

    def handle_frame(self, msg):
        # Check if the received frame ID matches the desired frame ID
        if msg.id != FRAME_ID:
            return

        # Store the timestamp of the last received frame
        last_time = self.last_time

        # Get the current time
        current_time = rospy.Time.now()

        # Calculate the time difference
        dt = (current_time - last_time).to_sec()

        # Store the current time as the timestamp of the last received frame
        self.last_time = current_time

        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])

        prev_wheel_travel = self.wheel_travel
        x_prev = self.x
        y_prev = self.y

        # Extract wheel travel value from frame in meters
        self.wheel_travel = (byte1 | (byte2 << 8)) / 1000.0

        # Extract steering angle value from frame in degrees
        self.steering_angle = (byte3 | (byte4 << 8)) * STEERING_ANGLE_FACTOR
        self.steering_angle_convert = self.steering_angle * (math.pi / 180)

        # Ignore large changes in wheel travel or steering angle
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
            return

        # Calculate linear velocity
        v_r = WHEELBASE * math.tan(self.steering_angle_convert) / 2

        # calculate vx and vy based on turning radius and angular velocity
        v_x = v_r * math.cos(self.kappa + self.theta)
        v_y = v_r * math.sin(self.kappa + self.theta)

        # calculate kappa based on vx and vy
        self.kappa = math.atan2(v_y, v_x)

        # Update position and orientation
        delta_x = v_x * dt
        delta_y = v_y * dt

        # Calculate angular velocity
        angular_velocity = v_r * (TURNING_RADIUS_FACTOR) / dt  # use the turning radius factor

        # Update forklift's X and Y coordinates
        self.x += x_prev + delta_x
        self.y += y_prev + delta_y

        # Calculate new orientation
        new_theta = self.theta + angular_velocity * dt

        # Update forklift's orientation
        self.theta = new_theta

        # Create PoseWithCovariance object
        pose_cov = PoseWithCovariance()
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        pose.orientation = Quaternion(*q)
        pose_cov.pose = pose

        # Create TwistWithCovariance object
        twist_cov = TwistWithCovariance()
        twist = Twist()
        twist.linear.x = v_x
        twist.linear.y = v_y
        twist.angular.z = angular_velocity
        twist_cov.twist = twist

        # Create Odometry message
        odometry_msg = Odometry()
        odometry_msg.header.stamp = current_time
        odometry_msg.header.frame_id = 'odom'
        odometry_msg.pose = pose_cov
        odometry_msg.twist = twist_cov

        # Publish odometry message
        self.odom_pub.publish(odometry_msg)

if __name__ == '__main__':
    try:
        node = ForkliftOdometryF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

##################################################### Accurate ###################################################
    
#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR = 0.01 # Steering factor to get the correct steering angle *DataSheet*
THRESHOLD = 0.001 # Threshold value for updating position
STEERING_ANGLE_CORRECTION_FACTOR = 0.5454

class ForkliftOdometryXs:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x = 0.0 # X-coordinate of the forklift's position
        self.y = 0.0 # Y-coordinate of the forklift's position
        self.theta = 0.0 # Orientation angle of the forklift
        self.wheel_travel = 0.0 # Distance traveled by the wheels
        self.steering_angle_convert = 0.0 # Steering angle of the forklift after conversion
        self.steering_angle = 0.0 # Steering angle of the forklift
        self.rotation_angle = 0.0 # Rotation angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
        self.last_time = rospy.Time.now() # Time of the last frame received

    def handle_frame(self, msg):
        # Check if the received frame ID matches the desired frame ID
        if msg.id != FRAME_ID:
            return

        # Store the timestamp of the last received frame
        last_time = self.last_time

        # Get the current time
        current_time = rospy.Time.now()

        # Calculate the time difference
        dt = (current_time - last_time).to_sec()

        # Store the current time as the timestamp of the last received frame
        self.last_time = current_time

        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])

        
        prev_wheel_travel = self.wheel_travel
        

        # Extract wheel travel value from frame in meters
        self.wheel_travel = (byte1 | (byte2 << 8))
        self.wheel_travel /= 1000.0

        # Extract steering angle value from frame in degrees
        self.steering_angle = (byte3 | (byte4 << 8)) * STEERING_ANGLE_FACTOR * STEERING_ANGLE_CORRECTION_FACTOR

        # Convert steering angle to radians
        self.steering_angle_convert = self.steering_angle * (math.pi / 180)

        # Ignore large changes in wheel travel or steering angle
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
            return
        

        # Calculate linear and angular velocities
        linear_velocity =  ((self.wheel_travel - prev_wheel_travel) / dt) * math.cos(self.steering_angle_convert)
        angular_velocity = linear_velocity * (math.tan(self.steering_angle_convert) / WHEELBASE)
            
        # Update position and orientation using kinematics equations

        delta_theta = angular_velocity * dt
        delta_x = linear_velocity * math.cos(self.steering_angle_convert) * math.cos(delta_theta) * dt
        delta_y = linear_velocity * math.sin(self.steering_angle_convert) * math.sin(delta_theta) * dt
        
        # Only update x position if delta_x is greater than the threshold
        if abs(delta_x) > THRESHOLD:
            self.x += delta_x

        # Only update y position if delta_y is greater than the threshold
        elif abs(delta_y) > THRESHOLD:
            self.y += delta_y

        self.x += delta_x
        self.y += delta_y

        # Publish the odometry message
        self.publish_odometry(linear_velocity, angular_velocity)

    def publish_odometry(self, linear_velocity, angular_velocity):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, 0)))
        odom.twist.twist = Twist(Vector3(linear_velocity, 0.0, 0.0), Vector3(0.0, 0.0, angular_velocity))

        # Publish the odometry message
        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep
    
if __name__ == '__main__':
  try:
      odometry = ForkliftOdometryXs()
      odometry.run()
  except rospy.ROSInterruptException:
      pass

###################################################################### Nice ###################################################################
  

#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR = 0.01 # Steering factor to get the correct steering angle *DataSheet*
STEERING_ANGLE_CORRECTION_FACTOR = 0.5454

class ForkliftOdometryXm:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x = 0.0 # X-coordinate of the forklift's position
        self.y = 0.0 # Y-coordinate of the forklift's position
        self.theta = 0.0 # Orientation angle of the forklift
        self.wheel_travel = 0.0 # Distance traveled by the wheels
        self.steering_angle_convert = 0.0 # Steering angle of the forklift after conversion
        self.steering_angle = 0.0 # Steering angle of the forklift
        self.rotation_angle = 0.0 # Rotation angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
        self.last_time = rospy.Time.now() # Time of the last frame received

    def handle_frame(self, msg):
        # Check if the received frame ID matches the desired frame ID
        if msg.id != FRAME_ID:
            return

        # Store the timestamp of the last received frame
        last_time = self.last_time

        # Get the current time
        current_time = rospy.Time.now()

        # Calculate the time difference
        dt = (current_time - last_time).to_sec()

        # Store the current time as the timestamp of the last received frame
        self.last_time = current_time

        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])

        
        prev_wheel_travel = self.wheel_travel
        

        # Extract wheel travel value from frame in meters
        self.wheel_travel = (byte1 | (byte2 << 8))
        self.wheel_travel /= 1000.0

        # Extract steering angle value from frame in degrees
        self.steering_angle = (byte3 | (byte4 << 8)) * STEERING_ANGLE_FACTOR * STEERING_ANGLE_CORRECTION_FACTOR

        print('Steering',self.steering_angle)

        # Convert steering angle to radians
        self.steering_angle_convert = math.radians(self.steering_angle)

        # Ignore large changes in wheel travel or steering angle
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
            return
        

        # Calculate linear and angular velocities
        linear_velocity =  ((self.wheel_travel - prev_wheel_travel) / dt) 
        angular_velocity = linear_velocity * (math.tan(self.steering_angle_convert) / WHEELBASE)
            
        # Update position and orientation using kinematics equations
        delta_theta = math.radians(angular_velocity * dt)
        self.theta += delta_theta
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        # Publish the odometry message
        self.publish_odometry()

    def publish_odometry(self):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
        

        # Publish the odometry message
        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep
    
if __name__ == '__main__':
  try:
      odometry = ForkliftOdometryXm()
      odometry.run()
  except rospy.ROSInterruptException:
      pass

############################################# zart ##################################################
  
#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two CAN frames
MAX_STEERING_VALUE = 32767 # Maximum Steering Value to prevent overflow and underflow  
MIN_STEERING_VALUE = -32768 # Minimum Steering Value to prevent overflow and underflow  
STEERING_ANGLE_FACTOR = 0.01 # Steering factor to correct the steering angle *DataSheet*
STEERING_ANGLE_CORRECTION_FACTOR = 0.5454 # Steering factor to calibrate the steering angle value

class ForkliftOdometryzeft:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x = 0.0 # X-coordinate of the forklift's position
        self.y = 0.0 # Y-coordinate of the forklift's position
        self.theta = 0.0 # Orientation angle of the forklift
        self.wheel_travel = 0.0 # Distance traveled by the wheels
        self.internal_steering_angle = 0.0 # Steering angle raw data
        self.steering_angle_convert = 0.0 # Steering angle of the forklift after conversion
        self.steering_angle = 0.0 # Steering angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
        self.Steering_pub = rospy.Publisher ('Steering', Float32, queue_size=1 )
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
        self.time = rospy.Time.now() # Time of the last frame received

    def handle_frame(self, msg):
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

        # Extract the wheel travel and steering angle data from the CAN frame 
        byte1 = (msg.data[0])
        byte2 = (msg.data[1])
        byte3 = (msg.data[2])
        byte4 = (msg.data[3])

        # Store the current wheel travel 
        prev_wheel_travel = self.wheel_travel
        
        # Extract wheel travel value from frame in meters
        self.wheel_travel = (byte1 | (byte2 << 8))
        self.wheel_travel /= 1000.0

        # Extract steering angle value from frame in degrees
        raw_steering_angle = (byte3 | (byte4 << 8))
        
        # Store the current steering angle 
        prev_steering_angle = self.steering_angle

        # Check for wrap-around and adjust the value accordingly
        if prev_steering_angle < MIN_STEERING_VALUE and raw_steering_angle >= MIN_STEERING_VALUE:
        # We've wrapped around from negative to positive
        # Adjust the internal representation
           self.internal_steering_angle -= MAX_STEERING_VALUE
        elif prev_steering_angle > MAX_STEERING_VALUE and raw_steering_angle <= MAX_STEERING_VALUE:
        # We've wrapped around from positive to negative
        # Adjust the internal representation
            self.internal_steering_angle += MAX_STEERING_VALUE

        # Apply the correction factor and update the internal steering angle
        self.steering_angle = (raw_steering_angle + self.internal_steering_angle) 
   
        print('Steering',self.steering_angle)
        self.Steering_pub.publish(self.steering_angle)

        # Convert steering angle to radians
        self.steering_angle_convert = math.radians(self.steering_angle*STEERING_ANGLE_FACTOR)

        # Ignore large changes in wheel travel or steering angle
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE :
            return
        
        # Calculate linear and angular velocities
        linear_velocity = ((self.wheel_travel - prev_wheel_travel) / dt) 
        angular_velocity = linear_velocity * (math.tan(self.steering_angle_convert) / WHEELBASE)

        # Update position and orientation using kinematics equations
        delta_theta = math.radians(angular_velocity * dt)
        self.theta += delta_theta

        # Store the current orientation angle at the beginning of the movement
        initial_theta = self.theta

        delta_x = linear_velocity * math.cos(initial_theta) * dt
        delta_y = linear_velocity * math.sin(initial_theta) * dt
        self.x += delta_x 
        self.y += delta_y 

        # Publish the odometry message
        self.publish_odometry()

    def publish_odometry(self):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
        
        # Publish the odometry message
        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.publish_odometry()
            rate.sleep()
    
if __name__ == '__main__':
    try:
      odometry = ForkliftOdometryzeft()
      odometry.run()
    except rospy.ROSInterruptException:
      pass
################################################## Perfect #####################################################################

#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR = 0.01 # Steering factor to get the correct steering angle *DataSheet*
STEERING_ANGLE_CORRECTION_FACTOR = 1.098 # Steering angle correction factor driven by trial and error 

class ForkliftOdometryAM92:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x = 0.0 # X-coordinate of the forklift's position
        self.y = 0.0 # Y-coordinate of the forklift's position
        self.theta = 0.0 # Orientation angle of the forklift
        self.wheel_travel = 0.0 # Distance traveled by the wheels
        self.steering_angle_convert = 0.0 # Steering angle of the forklift after conversion
        self.steering_angle = 0.0 # Steering angle of the forklift
        self.rotation_angle = 0.0 # Rotation angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
        self.time = rospy.Time.now() # Time of the last frame received

    def handle_frame(self, msg):
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
        self.wheel_travel = (byte1 | (byte2 << 8))
        self.wheel_travel /= 1000.0

        # Extract steering angle value from frame in degrees
        self.steering_angle = ((byte3 | (byte4 << 8)) * STEERING_ANGLE_FACTOR * STEERING_ANGLE_CORRECTION_FACTOR)/2

        print('Steering',self.steering_angle)

        # Convert steering angle to radians
        self.steering_angle_convert = math.radians(self.steering_angle)

        # Ignore large changes in wheel travel 
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
            return
        
        # Calculate linear and angular velocities
        linear_velocity =  ((self.wheel_travel - prev_wheel_travel) / dt) 
        angular_velocity = linear_velocity * (math.tan(self.steering_angle_convert) / WHEELBASE)
            
        # Update position and orientation using kinematics equations
        delta_theta = angular_velocity * dt
        self.theta += delta_theta
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y
        

        # Publish the odometry message
        self.publish_odometry()

    def publish_odometry(self):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
        

        # Publish the odometry message
        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep
    
if __name__ == '__main__':
  try:
      odometry = ForkliftOdometryAM92()
      odometry.run()
  except rospy.ROSInterruptException:
      pass
########################################################## Mario ######################################
  
#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE = 1.46 # Distance between the front and rear wheels of the forklift
FRAME_ID = 799 # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE = 1 # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR = 0.01 # Steering factor to get the correct steering angle *DataSheet*
STEERING_ANGLE_CORRECTION_FACTOR = 1.098 # Steering angle correction factor driven by mapping data from CAN bus from (0 to 655.35) to (0 to 360) 

# Global variables
path = Path()
temp = PoseStamped()
path.poses = []
pose = PoseStamped()

class ForkliftOdometryXSS:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x = 0.0 # X-coordinate of the forklift's position
        self.y = 0.0 # Y-coordinate of the forklift's position
        self.theta = 0.0 # Orientation angle of the forklift
        self.wheel_travel = 0.0 # Distance traveled by the wheels
        self.steering_angle_convert = 0.0 # Steering angle of the forklift after conversion
        self.steering_angle = 0.0 # Steering angle of the forklift
        self.rotation_angle = 0.0 # Rotation angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1) # Publisher for odometry data
        self.path_pub = rospy.Publisher('path', Path, queue_size=10)
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame) # Subscriber for frame messages
        self.time = rospy.Time.now() # Time of the last frame received

    def handle_frame(self, msg):
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
        self.wheel_travel = (byte1 | (byte2 << 8))
        self.wheel_travel /= 1000.0

        # Extract steering angle value from frame in degrees
        self.steering_angle = ((byte3 | (byte4 << 8)) * STEERING_ANGLE_FACTOR * STEERING_ANGLE_CORRECTION_FACTOR)/2

        print('Steering',self.steering_angle)

        # Convert steering angle to radians
        self.steering_angle_convert = math.radians(self.steering_angle)

        # Ignore large changes in wheel travel 
        if abs(self.wheel_travel - prev_wheel_travel) > MAX_WHEEL_TRAVEL_CHANGE:
            return
        
        # Calculate linear and angular velocities
        linear_velocity =  ((self.wheel_travel - prev_wheel_travel) / dt) 
        angular_velocity = linear_velocity * (math.tan(self.steering_angle_convert) / WHEELBASE)
            
        # Update position and orientation using kinematics equations
        delta_theta = angular_velocity * dt
        self.theta += delta_theta
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y
        

        # Publish the odometry message
        self.publish_odometry()
        self.publish_path()

    def publish_odometry(self):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        
       
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))

        # Publish the odometry message
        self.odom_pub.publish(odom)

    def publish_path(self):
        current_time = rospy.Time.now()

        # path = Path()
        # path.poses = []
        
        path.header.stamp = current_time
        path.header.frame_id = 'odom'
        pose.header.frame_id = "odom"
        pose.header.stamp = current_time
        pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*quaternion_from_euler(0, 0, self.theta)))
         
        #path.poses.append(pose)
        

        # print("X {}, Y, {}".format(input_path[0], input_path[1]))
        self.path_pub.publish(path)

  
    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep
    
if __name__ == '__main__':
  try:
      odometry = ForkliftOdometryXSS()
      odometry.run()
  except rospy.ROSInterruptException:
      pass


  

########################################################## Mario & PHIND (rviz PATH WORKING!!) ####################################
  
  #!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler

# Constants
WHEELBASE =  1.46  # Distance between the front and rear wheels of the forklift
FRAME_ID =  799  # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE =  1  # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR =  0.01  # Steering factor to get the correct steering angle *DataSheet*
STEERING_ANGLE_CORRECTION_FACTOR =  1.098  # Steering angle correction factor driven by trial and error  

class ForkliftOdometryAM922:
    def __init__(self):
        rospy.init_node('forklift_odometry')

        self.x =  0.0  # X-coordinate of the forklift's position
        self.y =  0.0  # Y-coordinate of the forklift's position
        self.theta =  0.0  # Orientation angle of the forklift
        self.wheel_travel =  0.0  # Distance traveled by the wheels
        self.steering_angle_convert =  0.0  # Steering angle of the forklift after conversion
        self.steering_angle =  0.0  # Steering angle of the forklift
        self.rotation_angle =  0.0  # Rotation angle of the forklift
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)  # Publisher for odometry data
        self.path_pub = rospy.Publisher('path', Path, queue_size=1)  # Publisher for path data
        self.frame_sub = rospy.Subscriber('received_messages', Frame, self.handle_frame)  # Subscriber for frame messages
        self.time = rospy.Time.now()  # Time of the last frame received

    def handle_frame(self, msg):
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

        print('Steering', self.steering_angle)

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
        delta_y = linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        # Publish the odometry message
        self.publish_odometry()
        self.publish_path()  # Call this method to publish the path

    def publish_odometry(self):
        current_time = rospy.Time.now()

        # Prepare an Odometry message with the calculated odometry data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(Point(self.x, self.y,  0.0), Quaternion(*quaternion_from_euler(0,  0, self.theta)))

        # Publish the odometry message
        self.odom_pub.publish(odom)

    def publish_path(self, path = Path()):
        # Create a new Path message
        
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = 'odom'  # Assuming 'odom' is the frame of reference
        #path.poses = []
        # Create a PoseStamped message for the current position
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        pose_stamped.pose.position.z =  0.0  # Assuming no z-coordinate
        pose_stamped.pose.orientation = Quaternion(*quaternion_from_euler(0,  0, self.theta))

        # Add the PoseStamped message to the Path
        #path.poses.append(pose_stamped)

        # Publish the path
        self.path_pub.publish(path)

    def run(self):
        rate = rospy.Rate(10)  #  10 Hz
        while not rospy.is_shutdown():
            
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry = ForkliftOdometryAM922()
        odometry.run()
    except rospy.ROSInterruptException:
        pass


########################################################## Mario & PHIND (rviz totally WORKING!!) ####################################

#!/usr/bin/env python

import rospy
import math
from can_msgs.msg import Frame
from nav_msgs.msg import Odometry, Path
# from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TransformStamped
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
# transformations
import tf

import tf_conversions
import tf2_ros

# Constants
WHEELBASE =  1.46  # Distance between the front and rear wheels of the forklift
FRAME_ID =  799  # ID of the frame for which to calculate odometry
MAX_WHEEL_TRAVEL_CHANGE =  1  # Maximum expected change in wheel travel between two CAN frames
STEERING_ANGLE_FACTOR =  0.01  # Steering factor to get the correct steering angle *DataSheet*
STEERING_ANGLE_CORRECTION_FACTOR =  1.098  # Steering angle correction factor driven by trial and error  

class ForkliftOdometryAM921:
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
        # self.publish_tf = rospy.Publisher('tf', tf, queue_size=1)  # Publisher for tf data
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

        print('Steering', self.steering_angle)

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
        delta_y = linear_velocity * math.sin(self.theta) * dt
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



        # self.publish_tf()

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

        # self.publish_tf()

    def publish_path(self, path = Path()):
        # Create a new Path message
        
        path.header.stamp = self.time
        path.header.frame_id = 'odom'  # Assuming 'odom' is the frame of reference
        if path.poses is None:
            path.poses = []
        # Create a PoseStamped message for the current position
        pose_stamped = PoseStamped()
        # pose_stamped.header.stamp = self.time
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
        t.header.frame_id = "/odom" # Add the PoseStamped message to the Path
        #path.poses.append(pose_stamped)
        # q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # print(br)
        br.sendTransform(t)
        # self.publish_tf.publish(t)

    def run(self):
        rate = rospy.Rate(100)  #  10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        odometry = ForkliftOdometryAM921()
        odometry.run()
        # odometry.publish_tf()
    except rospy.ROSInterruptException:
        pass
