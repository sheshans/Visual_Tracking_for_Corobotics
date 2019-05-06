#! /usr/bin/python
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image as KinectImage
from cv_bridge import CvBridge
from math import sqrt, atan2, sin, cos, exp, degrees
import termios, fcntl, sys, os, rospy, roslib
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from matplotlib import pyplot as plt
import message_filters
from corobot_common.msg import Pose, Goal
from sensor_msgs.msg import Image, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

# Range for which Kinect v1 worked
CAMERA_RANGE = 48

class Polar:  # commands
    __slots__ = ['d', 'a']

    def __init__(self, distance, angle):
        self.d = distance
        self.a = angle

    def __str__(self):
        return '(d={},a={})'.format(self.d, self.a)

    def valid(self):
        return self.d > 0


class Robot:

    def __init__(self, tracker):
        self.bridge = CvBridge()  # OpenCV API
        self.image = None  # Start with no image
        self.depth_image = None  # Start with no image
        self.tracker = tracker  # tacker OBJECT
        self.tracker_success = False  # tacker read flag set to False
        self.tracker_bbox = None  # No initial bounding box
        self.tracker_init = False
        self.count_num_linear = -1
        self.count_num_angular = -1

        self.prev_center_x = None
        self.prev_center_y = None
        self.prev_area = None
        self.controller = Controller()
        self.rgb_image_queue = []
        self.d_image_queue = []
        self.object_centers_queue = []
        self.bouding_box_queue = []
        self.pose = None
        self.scan = None
        self.laser_projector = LaserProjection()
        self.detecting = True
        self.disable_continuous_motion = False
        self.wp_publisher = rospy.Publisher('waypoints', Point)
        self.screens_count = 0
        # Polar points
        self.cache = []
        self.last_success_depth = None
        self.roll = self.pitch = self.yaw = 0.0
        self.target = 90
        self.kp = 0.5

    def histogram_equalize(self, img):
        b, g, r = cv2.split(img)
        red = cv2.equalizeHist(r)
        green = cv2.equalizeHist(g)
        blue = cv2.equalizeHist(b)
        return cv2.merge((blue, green, red))

    def save_images(self, rgb_image, depth_image):
        self.image = self.bridge.imgmsg_to_cv2(rgb_image, "passthrough")
        self.image = self.histogram_equalize(self.image)
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_image, "32FC1")

    def convertRobotCoorToGlobalCoor(self, polarPoint):
        sp = Pose()
        sp.x = self.pose.x + polarPoint.d * cos(self.pose.theta + math.radians(polarPoint.a))
        sp.y = self.pose.y + polarPoint.d * sin(self.pose.theta + math.radians(polarPoint.a))
        sp.theta = 0
        return sp

    def main_orchestrator(self, rgb_image, depth_image):
        # Solve all of perception here...
        # get both rgb and depth image and save
        self.save_images(rgb_image, depth_image)
        self.tracker_success, self.tracker_bbox = self.tracker.update(self.image)
        if (self.tracker_init):
            if self.tracker_success:
                curr_center_x = int(self.tracker_bbox[0] + (self.tracker_bbox[2] // 2))
                curr_center_y = int(self.tracker_bbox[1] + (self.tracker_bbox[3] // 2))
                temp_d = np.array(self.depth_image, dtype=np.float32)
                temp_i = np.array(self.image, dtype=np.float32)

                if( self.tracker_bbox[0] + (self.tracker_bbox[2] // 2) >= temp_i.shape[1] ):
                    cv2.putText(self.image, "Tracking Failure", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
                    cv2.imshow("Tracking ...", self.image)
                    cv2.waitKey(1)
                    return
                if( self.tracker_bbox[1] + (self.tracker_bbox[3] // 2) >= temp_i.shape[0] ):
                    cv2.putText(self.image, "Tracking Failure", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
                    cv2.imshow("Tracking ...", self.image)
                    cv2.waitKey(1)
                    return

                depth = temp_d[curr_center_y][curr_center_x]

                calculated_depth = depth / 1000.0

                if( temp_i.shape[0] > temp_i.shape[1] ):
                    max_dimension = temp_i.shape[0] * 1.0
                else:
                    max_dimension = temp_i.shape[1] * 1.0

                actual_center = max_dimension // 2

                difference = actual_center - curr_center_x

                angle_per_pixel = CAMERA_RANGE / max_dimension
                total_angle = angle_per_pixel * difference
                destination = self.convertRobotCoorToGlobalCoor(Polar(calculated_depth, total_angle))

                if abs(destination.y - self.pose.y) <= 0.099 and abs(destination.x - self.pose.x) <= 0.099:
                    pass
                else:
                    if self.detecting or not self.disable_continuous_motion:
                        self.wp_publisher.publish(Point(destination.x, destination.y, 0))
                self.detecting = False

                p1 = (int(self.tracker_bbox[0]), int(self.tracker_bbox[1]))
                p2 = (int(self.tracker_bbox[0] + self.tracker_bbox[2]),
                      int(self.tracker_bbox[1] + self.tracker_bbox[3]))
                cv2.rectangle(self.image, p1, p2, (255, 0, 0), 2, 1)

                if self.image is not None:
                    print("in image Tracking")
                    cv2.putText(self.image, "Tracking ...", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
                    cv2.imshow("Tracking ...", self.image)
                    cv2.waitKey(1)
                else:
                    cv2.putText(self.image, "Failure ...", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
                    cv2.imshow("Tracking ...", self.image)
                    cv2.waitKey(1)
                    rospy.loginfo("Nothing to visualize!")

            else:
                print("Tracking Failure")
                pass

    def waypointsReached(self, point):
        print("reached" + str(point))
        self.detecting = True

    # to select Object
    def selectROI(self, raw_image):
        # convert raw image to cv2 image format
        self.image = self.bridge.imgmsg_to_cv2(raw_image, "passthrough")
        if not self.tracker_init:
            # initialize parameters for robot to start moving
            self.controller.start_moving()
            # get tracker/bounding box
            self.tracker_bbox = cv2.selectROI(self.image, False)
            # boolean variable indicating if object was found
            self.tracker_success = self.tracker.init(self.image, self.tracker_bbox)
            # tracker initialized
            self.tracker_init = True

    def reduce_covariance(self, cov):
        """Convert a flat 6x6 covariance matrix into a flat 3x3."""
        """ 6 variances are for linear x, y, z and rotate x, y, z """
        return (cov[0], cov[1], cov[5],
                cov[6], cov[7], cov[11],
                cov[30], cov[31], cov[35])

    def odom_to_pose(self, odom):
        """Utility function to convert an Odometry message into a Pose message."""
        pose = Pose()
        pose.header = odom.header
        pose.x = odom.pose.pose.position.x
        pose.y = odom.pose.pose.position.y
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w
        pose.theta = atan2(2 * qw * qz, 1 - 2 * qz * qz)
        pose.cov = self.reduce_covariance(odom.pose.covariance)
        return pose

    # Pose -> None
    def poseCallback(self, odom):
        self.pose = self.odom_to_pose(odom)
        print(self.pose.theta)



def main():
    if len(sys.argv) != 2:
        raise AssertionError("Usage: rosrun project_name tracker.py tracker_type")

    # Create the tracker
    tracker = getattr(cv2, "Tracker{}_create".format(sys.argv[1]))()

    # Initialize node
    rospy.init_node("sy4923_tracker", anonymous=False)
    robot = Robot(tracker)  # create the robot object

    # to get robots current pose
    rospy.Subscriber('odom', Odometry, robot.poseCallback)

    # Synchronous callbacks of rgb and depth images
    image_rgb_sub = message_filters.Subscriber('/camera/rgb/image_color', KinectImage)
    image_depth_sub = message_filters.Subscriber('/camera/depth_registered/image_raw', KinectImage)
    ts = message_filters.ApproximateTimeSynchronizer([image_rgb_sub, image_depth_sub],
                                                     queue_size=2, slop=0.1)
    ts.registerCallback(robot.main_orchestrator)

    # Callback to select ROI
    rospy.Subscriber("/camera/rgb/image_color", KinectImage, robot.selectROI, queue_size=1)

    rospy.Subscriber('waypoints_reached', Point, robot.waypointsReached)
    rospy.Subscriber('waypoints_failed', Point, robot.waypointsReached)

    print("Done")
    rospy.spin()


class Controller:
    __slots__ = ['commandVelocityPublisher', 'wp_publisher']

    def __init__(self):
        self.commandVelocityPublisher = rospy.Publisher('mobile_base/commands/velocity', Twist)
        rospy.Subscriber('handle_twist', Twist, self.handle_twist)

    def handle_twist(self, twist_callback):
        self.commandVelocityPublisher.publish(twist_callback)

    def start_moving(self):
        fd = sys.stdin.fileno()

        oldterm = termios.tcgetattr(fd)
        newattr = termios.tcgetattr(fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, newattr)

        oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)
        try:
            self.twist = Twist()
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

        pass

    def move_Left_By(self, amount):
        print('move left {} m/s'.format(amount))
        self.twist.angular.z = amount
        # self.commandVelocityPublisher.publish(self.twist)

    def move_Right_By(self, amount):
        print('move right {} m/s'.format(amount))
        self.twist.angular.z = -amount
        # self.commandVelocityPublisher.publish(self.twist)

    def move_Forward_By(self, amount):
        print('move forward {} m/s'.format(amount))
        self.twist.linear.x = amount
        # self.commandVelocityPublisher.publish(self.twist)

    def move_Backward_By(self, amount):
        print('move backward {} m/s'.format(amount))
        self.twist.linear.x = -amount
        # self.commandVelocityPublisher.publish(self.twist)

    def test(self):
        p = Point(0.0, 0.5, 0.0)
        p.x = 1900

        while (True):
            self.wp_publisher.publish(p)

    def run(self):

        fd = sys.stdin.fileno()

        oldterm = termios.tcgetattr(fd)
        newattr = termios.tcgetattr(fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, newattr)

        oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

        speed = .5

        try:
            while True:
                twist = Twist()
                try:
                    c = sys.stdin.read(1)
                    if c == '\x1b':
                        c = sys.stdin.read(2)
                    if c == '[A' or c == 'w':
                        print('move forward {} m/s'.format(speed))
                        twist.linear.x = speed
                    elif c == 's' or c == '[B':
                        print('move backward {} m/s'.format(-speed))
                        twist.linear.x = -speed
                    elif c == '[C' or c == 'd':
                        print('rotate right {} m/s'.format(speed))
                        twist.angular.z = -speed
                    elif c == '[D' or c == 'a':
                        print('rotate left {} m/s'.format(speed))
                        twist.angular.z = speed
                    elif c == '-':
                        speed = max(.1, speed - .1)
                        print('decreasing linear speed to ', speed)
                    elif c == '+':
                        speed = min(.5, speed + .1)
                        print('increasing linear speed to ', speed)
                    elif c == 'q':
                        return
                    else:
                        print('Controls:')
                        print('          w or up    : move forward')
                        print('          a or left  : rotate counterclockwise')
                        print('          d or right : rotate clockwise')
                        print('          -          : decrease linear speed by .1 m/s (.1 m/s minimum)')
                        print('          +          : increase linear speed by .1 m/s (.5 m/s maximum)')
                        print('          q          : quit')
                    # more seamless with multiple requests
                    self.commandVelocityPublisher.publish(twist)
                    self.commandVelocityPublisher.publish(twist)
                    self.commandVelocityPublisher.publish(twist)
                except IOError:
                    pass
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
            fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Thank you for using the tracker! Exiting ...")

