#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState as JointStateMoveIt
from dynamixel_msgs.msg import JointState as JointStateDynamixel


class JointStatePublisher():
    def __init__(self):
        rospy.init_node('holy_joint_state_publisher')
	self.joints={ 'R_SAA': {'pos':0,'vel':0,'load':0}, 'R_SFE': {'pos':0,'vel':0,'load':0}, 'R_EB': {'pos':0,'vel':0,'load':0}, 'R_HAA': {'pos':0,'vel':0,'load':0}, 'R_HR': {'pos':0,'vel':0,'load':0}, 'R_HFE': {'pos':0,'vel':0,'load':0}, 'R_KFE': {'pos':0,'vel':0,'load':0}, 'R_AFE': {'pos':0,'vel':0,'load':0}, 'R_AR': 		{'pos':0,'vel':0,'load':0}, 'L_SAA': {'pos':0,'vel':0,'load':0}, 'L_SFE': {'pos':0,'vel':0,'load':0}, 'L_EB': {'pos':0,'vel':0,'load':0}, 'L_HAA': {'pos':0,'vel':0,'load':0}, 'L_HR': {'pos':0,'vel':0,'load':0}, 'L_HFE': {'pos':0,'vel':0,'load':0}, 'L_KFE': {'pos':0,'vel':0,'load':0}, 'L_AFE': {'pos':0,'vel':0,'load':0}, 'L_AR': {'pos':0,'vel':0,'load':0}}
        rate = 20 # 20Hz
        r = rospy.Rate(rate)

        # Start controller state subscribers
	for joint in self.joints:
        	rospy.Subscriber('/'+joint+'_controller/state', JointStateDynamixel, self.controller_state_handler)

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStateMoveIt)

        rospy.loginfo("Publishing joint_state at " + str(rate) + "Hz")

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def controller_state_handler(self, msg):
        self.joints[msg.name]['pos']=msg.current_pos
	self.joints[msg.name]['vel']=msg.velocity
	self.joints[msg.name]['load']=msg.load

    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStateMoveIt()

        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
	for joint in self.joints:
		msg.name.append(joint)
		msg.position.append(self.joints[joint]['pos'])
		msg.velocity.append(self.joints[joint]['vel'])
		msg.effort.append(self.joints[joint]['load'])

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        self.joint_states_pub.publish(msg)

if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
