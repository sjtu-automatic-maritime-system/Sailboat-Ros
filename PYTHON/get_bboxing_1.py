import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from tld_msgs.msg import BoundingBox
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

BALL_DIAMETER = 0.4  # m
FOCAL_LENGTH = 820.0 # pixel

def get_circle_pos(img, bbox):
    img_w, img_h = img.shape[1], img.shape[0]
    crop = img[bbox.y:(bbox.y + bbox.height), bbox.x:(bbox.x + bbox.width)]
    img_resize = cv2.resize(crop, (0, 0), fx=10, fy=10)
    cv2.imshow('Crop image', img_resize)
    cv2.waitKey(5)
    # img_hls = cv2.cvtColor(img_resize, cv2.COLOR_BGR2HLS)
    # img_h = img_hls[:, :, 0]
    # cv2.imshow('img_h', img_h)
    # cv2.waitKey(5)
    # img_l = img_hls[:, :, 1]
    # cv2.imshow('img_l', img_l)
    # cv2.waitKey(5)
    # img_s = img_hls[:, :, 2]
    # cv2.imshow('img_s', img_s)
    # cv2.waitKey(5)
    edge = cv2.cvtColor(img_resize, cv2.COLOR_BGR2GRAY)
    # img_l = cv2.GaussianBlur(img_l, ksize=(3, 3), sigmaX=2, sigmaY=2)
    # edge = cv2.Canny(img_l, 30, 100)
    # cv2.imshow('edge', edge)
    # cv2.waitKey(5)
    w, h = edge.shape[1], edge.shape[0]
    # circles = cv2.HoughCircles(edge, method=cv2.cv.CV_HOUGH_GRADIENT, dp=1,
    #                            minDist=50, param1=65, param2=40, minRadius=w / 4, maxRadius=w / 2)
    circles = cv2.HoughCircles(edge, method=cv2.cv.CV_HOUGH_GRADIENT, dp=1,
                               minDist=50, param1=50, param2=40, minRadius=w / 4, maxRadius=w / 2)
    print('circle: {}'.format(circles))
    if circles is not None:
        circles = circles.reshape(-1, 3)
        for cir in circles:
            cv2.circle(img_resize, (cir[0], cir[1]), cir[2], color=(255, 0, 0), thickness=3)

        cv2.imshow('circle', img_resize)
        cv2.waitKey(5)

        center = (int(bbox.x+circles[0][0]/10.0), int(bbox.y+circles[0][1]/10.0))
        radius = circles[0][2]/10.0
        print('circle_center: {}'.format(center))

        distance = BALL_DIAMETER * FOCAL_LENGTH / (2 * radius)
        print('distance: {}'.format(distance))
        direction = math.atan((center[0]-img_w/2.0)/FOCAL_LENGTH)

        return distance, direction
    return 0, 0


def draw_bbox(img, pts, color=(0, 255, 0), thickness=2):
    cv2.line(img, pts[0], pts[1], thickness=thickness, color=color)
    cv2.line(img, pts[1], pts[2], thickness=thickness, color=color)
    cv2.line(img, pts[2], pts[3], thickness=thickness, color=color)
    cv2.line(img, pts[3], pts[0], thickness=thickness, color=color)
    return img


def callback(img_msg, bbox):
    img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    print('bbox x: {}, y {}'.format(bbox.x, bbox.y))

    if bbox.confidence > 0:
        distance, direction = get_circle_pos(img, bbox)

        pts = []
        pt1 = (bbox.x, bbox.y)
        pts.append(pt1)
        pt2 = (bbox.x + bbox.width, bbox.y)
        pts.append(pt2)
        pt3 = (bbox.x + bbox.width, bbox.y + bbox.height)
        pts.append(pt3)
        pt4 = (bbox.x, bbox.y + bbox.height)
        pts.append(pt4)
        img = draw_bbox(img, pts)
    cv2.imshow("img_bbox", img)
    cv2.waitKey(5)
    img_bbox_pub.publish(bridge.cv2_to_imgmsg(img))


def main():
    rospy.init_node('get_bbox', anonymous=True)
    global img_bbox_pub, bridge
    img_bbox_pub = rospy.Publisher('/img_with_bbox_1', Image, queue_size=2)
    bridge = CvBridge()

    # img_sub = message_filters.Subscriber('/camera/image_raw', Image)
    # img_sub = message_filters.Subscriber('/camera/image_undistorted', Image)
    img_sub = message_filters.Subscriber('/camera/image_undistorted_rotated', Image)
    bbox_sub = message_filters.Subscriber('/tld_tracked_object', BoundingBox)

    ts = message_filters.ApproximateTimeSynchronizer([img_sub, bbox_sub], queue_size=10,
                                                     slop=0.01)  ## slop defines the delay (in seconds) with which messages can be synchronized
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    main()
