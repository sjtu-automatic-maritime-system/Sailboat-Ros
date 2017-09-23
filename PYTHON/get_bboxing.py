import rospy
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from tld_msgs.msg import BoundingBox
import message_filters
from cv_bridge import CvBridge
import cv2


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
    img_bbox_pub = rospy.Publisher('/img_with_bbox', Image, queue_size=2)
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
