import rospy
import math
from std_msgs.msg import Float32, Float32MultiArray, Int32

WHEEL_RADIUS = 0.065
WHEEL_DISTANCE = 0.4

DIST_THRESHOLD = 0.001
YAW_THRESHOLD = 0.001
THETADOT_NAV = 15
XDOT_NAV = 9

TRAJECTORY=[ 
[0.25 , 0],
[0.25 , 0.25],
[0.25 , 0.5],
[0.5  , 0.5],
[0.75 , 0.65],
[1    , 0.65],
[1.25 , 1],
[1.25 , 1.5],
[1.45 , 1.75],
[1.45 , 2],
[1.45 , 2.25],
[1.45 , 2.75],
[1.75 , 2.75],
[1.75 , 3],
[1.75 , 3.5],
[2    , 3.5],
[2.5  , 3.5],
[2.75 , 3.75],
[3    , 4],
[3.5  , 4],
[3.5  , 4.5],
[3.5  , 4.8],
[3.5  , 5]
 
      
]


def inverse_model(xdot, thetadot):
    # units are m/s and rad/s
    wl = (xdot - thetadot * WHEEL_DISTANCE / 2) / WHEEL_RADIUS
    wr = (xdot + thetadot * WHEEL_DISTANCE / 2) / WHEEL_RADIUS
    return wl, wr

def pose_clbk(msg):
    global x, y, theta
    x = msg.data[0]
    y = msg.data[1]
    theta = msg.data[2]

def euclidean_distance(x_, y_, goal_point_):
    dist_x = goal_point_[0] - x_
    dist_y = goal_point_[1] - y_
    dist = math.sqrt(dist_x**2 + dist_y**2) 
    return dist

def sign(x):
    if x >= 0:
        return 1
    else: 
        return -1

def pi_2_pi(angle):
    # a function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + math.pi) % (2 * math.pi) - math.pi

def steering_angle(x_, y_, theta_, goal_point_):
    return pi_2_pi(math.atan2(goal_point_[1] - y_, goal_point_[0] - x_) - theta_)

if __name__ == "__main__":

    rospy.init_node("navigator_node")

    # Initialize pose values
    x, y, theta = 1.825 , -1.775 , 0

    # Subscriber to the velocity commanding topic
    rospy.Subscriber("/wheel_odometry_localizer/pose", Float32MultiArray, pose_clbk)
    
    # Setup wheel speed publishers
    left_wheel_speed_pub = rospy.Publisher("/sim_ros_interface/left_motor/setpoint_speed", Float32, queue_size=10)
    right_wheel_speed_pub = rospy.Publisher("/sim_ros_interface/right_motor/setpoint_speed", Float32, queue_size=10)

    # Setup ROS rate 
    rate = rospy.Rate(10) # 30 Hz 

    rospy.loginfo("Navigation node successfully spawned")

    while not rospy.is_shutdown():
        
        rospy.loginfo("Starting to traverse the path")
        
        for goal_point in TRAJECTORY:
            while euclidean_distance(x, y, goal_point) > DIST_THRESHOLD and not rospy.is_shutdown():
                
                # Read proximity sensor
                proximity_msg = rospy.wait_for_message("/sim_ros_interface/proximity_sensor/state", Int32)
                
                if proximity_msg.data != 0:     # proximity sees something
                    xdot = 0
                    thetadot = 0
                    
                # Safe to navigate    
                elif abs(steering_angle(x, y, theta, goal_point)) > YAW_THRESHOLD:
                    xdot = 0
                    thetadot = THETADOT_NAV * sign(steering_angle(x, y, theta, goal_point)) 
                else:
                    xdot = XDOT_NAV
                    thetadot = 0

                lw_speed, rw_speed = inverse_model(xdot, thetadot)
                left_wheel_speed_pub.publish(Float32(lw_speed))
                right_wheel_speed_pub.publish(Float32(rw_speed))

                rate.sleep()

        rospy.loginfo("Robot navigation finished or terminated!")
        left_wheel_speed_pub.publish(Float32(0))    # publish zero speeds
        right_wheel_speed_pub.publish(Float32(0))
        break
        
