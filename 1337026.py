#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
import math
from tf import transformations

waypoint=None

#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(4.0); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************

        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.

            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue


        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print('Current waypoint (x,y): (',waypoint.translation.x,",",waypoint.translation.y,')')

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print('Current waypoint (theta): (',waypoint_theta,')\n')

        #***************************************
        #***          DRIVE THE ROBOT HERE (same as with assignment 1
        #**           - except you are driving towards a goal not away from an obstacle)
        #***************************************

        #for containing the motor commands to send to the robot
        motor_command=Twist()
        x_diff = waypoint.translation.x - translation[0]
        y_diff = waypoint.translation.y - translation[1]

        theta_diff = waypoint_theta - robot_theta;  #how much to turn
        if math.fabs(theta_diff) > math.pi:         #for negative angles, not to turn more than one round
            if waypoint_theta < -1*(math.pi/2) and robot_theta>math.pi/2:
                theta_diff = math.pi*2 - math.fabs(waypoint_theta) - robot_theta
            if waypoint_theta > math.pi/2 and robot_theta<-1*(math.pi/2):
                theta_diff = -1 * (math.pi*2 - waypoint_theta - math.fabs(robot_theta))



        print('x_diff' + str(x_diff) + '\n')
        print('y_diff' + str(y_diff) + '\n')
        dist = math.sqrt(x_diff*x_diff + y_diff * y_diff)
        motor_command.linear.x=dist     #default speed

        if math.fabs(theta_diff)<0.1:   #if our orientation is ok
            if dist<0.3:                #dont go too slow
                motor_command.linear.x = 0.3
            if dist>1:                  #can speed up here
                motor_command.linear.x=dist * 1.5
        stopflag = 0
        print('dist: ' + str(dist))
        if dist<0.1 and math.fabs(theta_diff)>0.1:     #too close to go. Turn first!
            motor_command.linear.x = 0
            stopflag = 1
            print('dont go\n')

        # if math.fabs(theta_diff)<0.01:
        #     if dist<0.3:
        #         motor_command.linear.x = 0.3
        #     if dist>0.8:
        #         motor_command.linear.x=dist *1.5

        backwards_flag = 0

        if robot_theta<math.pi/2 and robot_theta>-1*math.pi/2 and x_diff<-0.1 and y_diff<0.1 and stopflag==0:  #go backwards
            backwards_flag = 1
            motor_command.linear.x=-0.5 *math.sqrt(dist)
            motor_command.angular.z = 0

        angle = math.atan2(y_diff,x_diff)
        if math.fabs(waypoint_theta-angle)>0.2 and dist>0.2 and backwards_flag==0 and stopflag == 0 and math.fabs(angle)<math.pi:
            if waypoint_theta < -1*(math.pi/2) and angle>math.pi/2:
                waypoint_theta = math.pi*2 - math.fabs(waypoint_theta)
            if waypoint_theta > math.pi/2 and angle<-1*(math.pi/2):
                angle = (math.pi*2 - math.fabs(angle))

            fixed_angle = (angle+waypoint_theta)/2
            fixed_angle = 0
            if dist>1:
                fixed_angle = (2*angle+waypoint_theta)/3
            elif dist>0.5:
                fixed_angle = (angle+waypoint_theta)/2
            else:
                fixed_angle = (angle+2*waypoint_theta)/3
            if fixed_angle > math.pi:
                fixed_angle -= math.pi
                fixed_angle = -1*(math.pi-fixed_angle)

            theta_diff = fixed_angle - robot_theta
            print('theta_diff: ' + str(theta_diff))
            print('negative angle issue \n')
            print('angle: ' + str(angle))
            print('\nfixed angle: ' + str(fixed_angle))
            if math.fabs(theta_diff) > math.pi:         #for negative angles, not to turn more than one round

                if fixed_angle < -1*(math.pi/2) and robot_theta>math.pi/2:
                    theta_diff = math.pi*2 - math.fabs(fixed_angle) - robot_theta
                if fixed_angle > math.pi/2 and robot_theta<-1*(math.pi/2):
                    theta_diff = -1 * (math.pi*2 - (fixed_angle - math.fabs(robot_theta)))
                print('theta_diff after process: ' + str(theta_diff))
            print('angle fix\n')
        motor_command.angular.z = theta_diff
        #
        #
        #
        #
        # angle = math.atan2(y_diff,x_diff)
        # if math.fabs(waypoint_theta-angle)>0.1 and dist>0.5:
        #     theta_diff = (angle+waypoint_theta)/ - robot_theta
        #     print('angle fix\n')
        # if robot_theta<math.pi/2 and robot_theta>-1*math.pi/2 and x_diff<-0.1:  #go backwards
        #     backwards_flag = 1
        #     motor_command.linear.x=-0.5 *math.sqrt(dist)
        #     motor_command.angular.z = 0
        #I don't know what to do because nobody has programmed me with any smartness,
        #so I'll do what everybody does, and drive very fast straight forwards.
        # if math.fabs(x_diff)<0.01 or math.fabs(y_diff)<0.01:
        #     if math.fabs(theta_diff)>0.01:
        #         motor_command.linear.x = 0
        #         print('dont go\n')


        motor_command_publisher.publish(motor_command)

        #######################################################################
        #FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME
        #FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME
        #FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME -- FIX ME
        #######################################################################

        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads


    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
