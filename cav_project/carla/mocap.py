import rospy
from std_msgs.msg import Float32MultiArray

class MoCap(): ##
    def mocap_callback(self, data): ##
        self.data = data.data
        self.odom_x = self.data[0]
        self.odom_y = self.data[1] * 20
        self.odom_yaw = self.data[2]

    def get_mocap(self):
        subscriber = rospy.Subscriber('mocap_info', Float32MultiArray, self.mocap_callback) ##
        rospy.sleep(0.1)
        mocap_x = -1* self.odom_x
        mocap_y = self.odom_y
        mocap_yaw = -1 * self.odom_yaw
        subscriber.unregister()
        return mocap_x,mocap_y,mocap_yaw
	
