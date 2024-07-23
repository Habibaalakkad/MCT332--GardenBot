import rospy
import math
from std_msgs.msg import Float32, Float32MultiArray

WHEEL_RADIUS = 0.065
WHEEL_DISTANCE = 0.4

def pi_2_pi(angle):
    # a function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + math.pi) % (2 * math.pi) - math.pi

def forward_model(wl, wr):
    # units are m/s and rad/s
    xdot = (wr * WHEEL_RADIUS + wl * WHEEL_RADIUS) / 2
    thetadot = (wr * WHEEL_RADIUS - wl * WHEEL_RADIUS) / WHEEL_DISTANCE
    return xdot, thetadot

def left_wheel_speed_callback(msg):
    global wl
    wl = msg.data

def right_wheel_speed_callback(msg):
    global wr
    wr = msg.data


if __name__ == "__main__":

    # Initialize the ROS node
    rospy.init_node("wheel_odometry_localizer_node")

    # Initialize wheel speed values 
    wl, wr = 0, 0

    # Publisher for the pose topic 
    pose_pub = rospy.Publisher("/wheel_odometry_localizer/pose", Float32MultiArray, queue_size= 10)


    # Subscriber to the velocity commanding topic
    rospy.Subscriber("/sim_ros_interface/left_motor/actual_speed", Float32, left_wheel_speed_callback)
    rospy.Subscriber("/sim_ros_interface/right_motor/actual_speed", Float32, right_wheel_speed_callback)

    # Initialize time for integration
    t_start = rospy.get_time()
    t_prev = rospy.get_time()

    # Initialize variables for trapezoidal integration
    xdot_prev, thetadot_prev = 0, 0

    # Initialize integration values 
    x, y, theta = 0, 0, 0

    rospy.loginfo("Connecting to CoppeliaSim topics")
    rospy.loginfo("Accessing wheel speeds")
    
    while not rospy.is_shutdown():
        xdot, thetadot = forward_model(wl, wr)

        # calculate dt
        t_start = rospy.get_time()
        dt = t_start - t_prev

        # do the integration (trapezoidal)
        x += (xdot + xdot_prev)/2 * math.cos(theta) * dt
        y += (xdot + xdot_prev)/2 * math.sin(theta) * dt
        theta += (thetadot + thetadot_prev)/2 * dt 
        theta = pi_2_pi(theta) # wrap-to-pi
        t_prev = t_start
        
        # equate variables for next iteration
        xdot_prev = xdot
        thetadot_prev = thetadot

        # initialize ros message to be published
        msg = Float32MultiArray()
        msg.data.append(x)
        msg.data.append(y)
        msg.data.append(theta)
        pose_pub.publish(msg)
        
