
import unittest
import rospy, rostest
import time

from maniros.msg import EncoderOdometry
from sensor_msgs.msg import JointState

class EncoderPubTest(unittest.TestCase):
	def __init__(self, *args):
		super(EncoderPubTest, self).__init__(*args)
		rospy.init_node("test_input", anonymous=True)
		rospy.Subscriber("/encoder_odometry", EncoderOdometry, self.callback)
		self.enc_pub = rospy.Publisher("manisim/joint_states", JointState, queue_size=10)

	def callback(self, data):
		self.success = True

	def test_basic_function(self):
		self.success = False
		msg = JointState()
		msg.velocity = [1,1,1,1,1,1,1,1]

		timeout_t = time.time() + 10.0
		while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
			self.enc_pub.publish(msg)
			time.sleep(0.1)

		self.assertTrue(self.success, 'Callback has not been called')

if __name__ == '__main__':
	rostest.rosrun('maniros','test_encoder_pub', EncoderPubTest)

