#!/usr/bin/env python

import sys
import rospy
import yaml
from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.msg import CameraInfo


def loadCalibrationFile(filename):
    ci = CameraInfo()
    try:
        f = open(filename)
        calib = yaml.safe_load(f)
        if calib is not None:
            ci.width = calib['width']
            ci.height = calib['height']
            ci.distortion_model = calib['distortion_model']
            ci.D = calib['D']
            ci.K = calib['K']
            ci.R = calib['R']
            ci.P = calib['P']

    except IOError:                     # OK if file did not exist
        pass

    return ci


def set_camera_info_client(cname, filename):
    sci_service = f'/{cname}/set_camera_info'
    rospy.wait_for_service(sci_service)
    try:
        sci = rospy.ServiceProxy(sci_service, SetCameraInfo)
        camera_info = loadCalibrationFile(filename)
        return sci(camera_info)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def usage():
    return f"{sys.argv[0]} [camera name] [camera_info.yaml]"


if __name__ == "__main__":
    if len(sys.argv) == 3:
        cname = sys.argv[1]
        camera_info = sys.argv[2]
    else:
        print(usage())
        sys.exit(1)
    res = set_camera_info_client(cname, camera_info)
    print(res)
