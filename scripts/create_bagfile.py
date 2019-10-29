#!/usr/bin/python
# coding: UTF-8

from ros import rosbag
import numpy as np
import cv2
import re
import rospy
import tf
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

path_to_img = '/home/liu/Desktop/img/'
path_odom = '/home/liu/Desktop/img/odom.csv'
num_of_frame = 498


def readlog(fname):
    log = []
    f = open(fname)
    lines = f.readlines()
    for line in lines:
        element = line.splitlines()[0].split(',')
        try:
            log.append([float(i) for i in element ])
        except:
            pass
    return log

def readimg(path_name, img_name, num):
    imgs = []
    for i in range(num):
        fname = path_name + img_name + '_'+('%04d'% i )+'.png'
        img = cv2.imread(fname)
        imgs.append(img)
    return imgs

def CreateStereoBag(imgs0, imgs1, odoms, hz, bagname):
    '''Creates a bag file containing stereo image pairs'''
    bag =rosbag.Bag(bagname, 'w')
    bridge = CvBridge()
    t = 100.
    for i in range(len(imgs0)):
        t += 1.0/float(hz)
        stamp = rospy.Time.from_seconds(t)
        img0 = cv2.cvtColor(imgs0[i], cv2.COLOR_BGR2GRAY)
        img1 = cv2.cvtColor(imgs1[i], cv2.COLOR_BGR2GRAY)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "base_link"
        odom_quat = tf.transformations.quaternion_from_euler(0, -(np.pi/2 + odoms[i][2]), 0)
        odom.pose.pose = Pose(Point(-odoms[i][0]/1000., 0., odoms[i][1]/1000.), Quaternion(*odom_quat))
        msg_img0 = bridge.cv2_to_imgmsg(img0, "mono8")
        msg_img0.header.frame_id = "front_camera_link"
        msg_img1 = bridge.cv2_to_imgmsg(img1, "mono8")
        msg_img1.header.frame_id = "front_camera_link"

        bag.write('front/image_raw', msg_img0, stamp)
        bag.write('rear/image_raw', msg_img1, stamp)
        bag.write('odom', odom, stamp)
    bag.close()
        


odoms = readlog(path_odom)
front_imgs = readimg(path_to_img,'front',num_of_frame)
rear_imgs = readimg(path_to_img,'rear',num_of_frame)
CreateStereoBag(front_imgs,rear_imgs, odoms, 10, '/home/liu/bag/t.bag')



#img = cv2.imread('/home/liu/Desktop/',0)