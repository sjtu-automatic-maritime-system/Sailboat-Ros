import rospy
from sensor_msgs.msg import Image
from tld_msgs.msg import BoundingBox
from sailboat_message.msg import Detected_obs
import message_filters
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import camera_info

BALL_DIAMETER = 0.3  # m
FOCAL_LENGTH = 455.0  # pixel
RESIZE_RATIO = 10.0
SHOW_SIZE = 200  # pixel


def project_reverse_intrinsic(u, v):
    pt_boat = camera_info.img_to_boat(u, v)
    depth = math.fabs(pt_boat[0])
    distance = math.sqrt(pt_boat[0] ** 2 + pt_boat[1] ** 2)
    direction = math.atan(pt_boat[1] / pt_boat[0])
    print 'intrinsic method: {}, {}, {}'.format(depth, distance, direction)
    if 1:
        row = '{depth} {distance} {direction}\n'.format(depth=depth, distance=distance, direction=direction)
        f2.write(row)
    return depth, distance, direction


def project_reverse_geometry_1(u, v):
    h = math.fabs(v - camera_info.IMG_CENTER_Y)
    depth = camera_info.DISTANCE_CAM_TO_WATER * camera_info.FOCAL_LENGTH / h
    direction = math.atan((u - camera_info.IMG_CENTER_X) / camera_info.FOCAL_LENGTH)
    distance = depth / math.cos(direction)
    print 'geometry 1 : {}, {}, {}'.format(depth, distance, direction)
    if 1:
        row = '{depth} {distance} {direction}\n'.format(depth=depth, distance=distance, direction=direction)
        f3.write(row)
    return depth, distance, direction


def get_circle_pos(img, bbox):
    img_w, img_h = img.shape[1], img.shape[0]
    crop = img[bbox.y:(bbox.y + bbox.height), bbox.x:(bbox.x + bbox.width)]
    img_resize = cv2.resize(crop, (0, 0), fx=RESIZE_RATIO, fy=RESIZE_RATIO)
    # cv2.imshow('Crop image', img_resize)
    # cv2.waitKey(5)
    gray = cv2.cvtColor(img_resize, cv2.COLOR_BGR2GRAY)
    w, h = gray.shape[1], gray.shape[0]
    # circles = cv2.HoughCircles(gray, method=cv2.cv.CV_HOUGH_GRADIENT, dp=1,
    #                            minDist=50, param1=65, param2=40, minRadius=w / 4, maxRadius=w / 2)
    circles = cv2.HoughCircles(gray, method=cv2.cv.CV_HOUGH_GRADIENT, dp=1,
                               minDist=50, param1=50, param2=40, minRadius=h / 4, maxRadius=h / 2)
    print('circle: {}'.format(circles))
    if circles is not None:
        gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        circles = circles.reshape(-1, 3)
        for cir in circles:
            cv2.circle(gray, (cir[0], cir[1]), cir[2], color=(0, 0, 255), thickness=6)
            break

        # cv2.imshow('circle', gray)
        # cv2.waitKey(5)

        img_resize = cv2.resize(img_resize, (SHOW_SIZE, SHOW_SIZE))
        gray = cv2.resize(gray, (SHOW_SIZE, SHOW_SIZE))
        y_offset = int(img_h / 15)
        x_offset = int(img_w / 1.6)
        img[y_offset:(y_offset + SHOW_SIZE), x_offset:(x_offset + SHOW_SIZE)] = img_resize
        img[(y_offset):(y_offset + SHOW_SIZE), (x_offset + SHOW_SIZE + 10):(x_offset + 2 * SHOW_SIZE + 10)] = gray

        center = (int(bbox.x + circles[0][0] / RESIZE_RATIO), int(bbox.y + circles[0][1] / RESIZE_RATIO))
        radius = circles[0][2] / RESIZE_RATIO
        # print('circle_center: {}'.format(center))

        depth = BALL_DIAMETER * camera_info.FOCAL_LENGTH / (2 * radius)
        # print('depth: {}'.format(depth))
        direction = math.atan((center[0] - img_w / 2.0) / camera_info.FOCAL_LENGTH)
        distance = depth / math.cos(direction)
        print 'geometry 2 : {}, {}, {}'.format(depth, distance, direction)

        # set background
        background = np.zeros(shape=(190, 550, 3))
        img[60:250, 60:610] = background
        cv2.putText(img, 'depth   : {0:.2f} m.'.format(depth),
                    (img_w / 15, img_h / 8), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, color=(0, 255, 0), thickness=4)
        cv2.putText(img, 'distance: {0:.2f} m.'.format(distance),
                    (img_w / 15, img_h / 8 + 50), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, color=(0, 255, 0),
                    thickness=4)
        cv2.putText(img, 'direction: {1:.2f} deg.'.format(depth, direction * 57.3),
                    (img_w / 15, img_h / 8 + 100), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5, color=(0, 255, 0),
                    thickness=4)

        return center, radius, depth, distance, direction
    return (0, 0), 0, 0, 0, 0


def draw_bbox(img, pts, color=(0, 255, 0), thickness=4):
    cv2.line(img, pts[0], pts[1], thickness=thickness, color=color)
    cv2.line(img, pts[1], pts[2], thickness=thickness, color=color)
    cv2.line(img, pts[2], pts[3], thickness=thickness, color=color)
    cv2.line(img, pts[3], pts[0], thickness=thickness, color=color)
    return img


def callback(img_msg, bbox):
    img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    print('bbox x: {}, y {}'.format(bbox.x, bbox.y))

    detectd_obs = Detected_obs()
    detectd_obs.confidence = bbox.confidence
    detectd_obs.header = img_msg.header
    detectd_obs.header.frame_id = 'camera'
    if bbox.confidence > 0:
        center, radius, depth, distance, direction = get_circle_pos(img, bbox)
        u, v = center[0], center[1] + radius
        project_reverse_intrinsic(u, v)
        project_reverse_geometry_1(u, v)

        detectd_obs.header.stamp = img_msg.header.stamp
        detectd_obs.depth = depth
        detectd_obs.direction = direction
        detectd_obs.distance = distance
        if 1:
            row = '{depth} {distance} {direction}\n'.format(depth=depth, distance=distance, direction=direction)
            f1.write(row)

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
    # cv2.imshow("img_bbox", img)
    # cv2.waitKey(5)
    img_bbox_pub.publish(bridge.cv2_to_imgmsg(img))
    obs_pub.publish(detectd_obs)


def main():
    rospy.init_node('get_bbox', anonymous=True)
    global img_bbox_pub, bridge, obs_pub
    img_bbox_pub = rospy.Publisher('/img_with_bbox', Image, queue_size=2)
    bridge = CvBridge()
    obs_pub = rospy.Publisher('/detected_obs', Detected_obs, queue_size=2)

    # img_sub = message_filters.Subscriber('/camera/image_raw', Image)
    # img_sub = message_filters.Subscriber('/camera/image_undistorted', Image)
    img_sub = message_filters.Subscriber('/camera/image_undistorted_rotated', Image)
    bbox_sub = message_filters.Subscriber('/tld_tracked_object', BoundingBox)

    ts = message_filters.ApproximateTimeSynchronizer([img_sub, bbox_sub], queue_size=10,
                                                     slop=0.01)  ## slop defines the delay (in seconds) with which messages can be synchronized
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    f1 = open('obs_distance_direction_geo2.csv', 'wb')
    f2 = open('obs_distance_direction_intr.csv', 'wb')
    f3 = open('obs_distance_direction_geo1.csv', 'wb')
    main()
