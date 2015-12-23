import rospy
from sensor_msgs.msg import JointState

pub_init = rospy.Publisher ('initConfig', JointState, queue_size=1)
pub_goal = rospy.Publisher ('goalConfig', JointState, queue_size=1)
rospy.init_node ('config_pulisher')
rate = rospy.rate (1)

q_init = JointState ()
q_goal = JointState ()

while not rospy.is_shutdown ():
    pub_init.publish (q_init)
    pub_goal.publish (q_goal)
    rate.sleep ()
