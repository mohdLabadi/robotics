import random
from operator import pos

import cv2
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from arm_state_estimation.tracker import CylinderTracker

matplotlib.use('Agg')

plt.ylim(top=100)

def check_pf_parameters(detector):
    
    init_cov = np.min(np.abs(np.diag(detector.pf._init_cov)))
    if init_cov < 1:
        rospy.logwarn("Initial covariance is too small to realistically model noise in this setting.")
        raise ValueError("Initial covariance is too small to realistically model noise in this setting. Consider choosing bigger values for covariance.")

    std_u = abs(detector.pf.std_u)
    if std_u < 1:
        rospy.logwarn("Motion model noise is too small to realistically model noise in this setting.")
        raise ValueError("Motion model noise is too small to realistically model noise in this setting. Consider choosing bigger values for std_u.")

if __name__ == '__main__':
    rospy.init_node('cylinder_tracking', disable_signals=True)
    detector = CylinderTracker()
    check_pf_parameters(detector)
    rospy.spin()
    cv2.destroyAllWindows()
