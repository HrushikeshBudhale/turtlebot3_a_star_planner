#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import planner
import sys

def main():
    msg=Twist()
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    rospy.init_node('tb_node',anonymous=True)

    # rpm values have been kept lower to eliminate wheel slip
    actions = planner.main(sys.argv[1:])
    rospy.loginfo("Path Found!")
    rospy.loginfo("Waiting for Gazebo to start")
    rospy.sleep(5)  # Todo: handle starting of gazebo in closed loop
    for (l_rpm, r_rpm) in actions:
        l_rpm += 0.0101  # correction for error in gazebo model's wheel radius

        # division by 60 to convert from rpm to rps
        msg.linear.x = (l_rpm + r_rpm)/(2*60.0)
        msg.angular.z = ((r_rpm/60.0 - l_rpm/60.0) / planner.mm.b)
        
        pub.publish(msg)
        rospy.sleep(2.05)   # sleep time adjusted experimentally

    # stop the robot and node
    msg=Twist()
    pub.publish(msg)
    rospy.spin()
    rospy.signal_shutdown("done")
		
if __name__=='__main__':
    main()
	
