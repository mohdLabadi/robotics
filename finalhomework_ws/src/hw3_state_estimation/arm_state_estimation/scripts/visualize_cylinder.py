import rospy
from arm_state_estimation.init_env import Env

if __name__ == "__main__":
    env = Env()
    try:
        env.publish_cylinder()
    except rospy.ROSInterruptException:
        pass
