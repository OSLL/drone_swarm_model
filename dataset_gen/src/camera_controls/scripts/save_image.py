#!/usr/bin/env python

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion


def save(image, filename):
    try:
        cv2_img = CvBridge().imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imwrite(filename, cv2_img)


def get_filename(cname):
    gms_service = '/gazebo/get_model_state'
    rospy.wait_for_service(gms_service)
    try:
        gms = rospy.ServiceProxy(gms_service, GetModelState)
        model_state = gms(cname, '')
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

    orientation_list = [model_state.pose.orientation.x,
                        model_state.pose.orientation.y,
                        model_state.pose.orientation.z,
                        model_state.pose.orientation.w]
    pose = [model_state.pose.position.x,
            model_state.pose.position.y,
            model_state.pose.position.z,
            *euler_from_quaternion(orientation_list)]
    base = '_'.join([f'{x:.2f}' for x in pose])
    return f'{base}.png'


def get_image(cname):
    image_topic = f'/{cname}/camera/image_raw/r'
    image = rospy.wait_for_message(image_topic, Image)
    return image


def save_image_client(cname, prefix):
    rospy.init_node('save_image')
    filename = f'{prefix}{get_filename(cname)}'
    image = get_image(cname)
    save(image, filename)


def usage():
    return f"{sys.argv[0]} [camera name] [[filename prefix]]"


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        cname = sys.argv[1]
        prefix = sys.argv[2] if len(sys.argv) == 3 else ''
    else:
        print(usage())
        sys.exit(1)
    save_image_client(cname, prefix)
