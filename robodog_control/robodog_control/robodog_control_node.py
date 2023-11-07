# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    robodog_control_node.py                            :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: Paul Joseph <paul.joseph@pbl.ee.ethz.ch    +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2023/10/30 08:05:28 by Paul Joseph       #+#    #+#              #
#    Updated: 2023/11/07 09:11:05 by Paul Joseph      ###   ########.fr        #
#                                                                              #
# **************************************************************************** #


from queue import Queue
import numpy as np
import tf

# ROS imports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from gaze_msgs.msg import GazeStamped
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient

import rclpy


class RobodogCtrl(Node):
    #      ____                _              _
    #     / ___|___  _ __  ___| |_ _   _  ___| |_ ___  _ __
    #    | |   / _ \| '_ \/ __| __| | | |/ __| __/ _ \| '__|
    #    | |__| (_) | | | \__ \ |_| |_| | (__| || (_) | |
    #     \____\___/|_| |_|___/\__|\__,_|\___|\__\___/|_|
    def __init__(self) -> None:
        super().__init__('RobodogCtrl')
        # customizable params
        self.robodogNode = '/camera'    # node name for the robodog cam
        self.max_queue_size = 1     # max images saved in queue
        self.depth_cam_info = None
        self.client = ActionClient(self,
                                   NavigateToPose,
                                   '/move_base')

        # init data queues
        self.init_queues()
        # init ROS stuff
        self.init_ros()

    #    ____      _ _ _                _
    #   / ___|__ _| | | |__   __ _  ___| | _____
    #  | |   / _` | | | '_ \ / _` |/ __| |/ / __|
    #  | |__| (_| | | | |_) | (_| | (__|   <\__ \
    #   \____\__,_|_|_|_.__/ \__,_|\___|_|\_\___/
    def robodog_gaze_sub_callback(self, msg) -> None:
        self.queue_msg(msg, self.robodog_gaze_queue)

    def robodog_depth_sub_callback(self, msg) -> None:
        self.queue_msg(msg, self.robodog_depth_queue)

    def robodog_depth_info_sub_callback(self, msg) -> None:
        self.set_depth_cam_info(msg)

    def robodog_ctrl_pub_callback(self) -> None:
        # self.calc_cmd_from_gaze()
        self.test()

    #    _____                 _   _
    #   |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
    #   | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
    #   |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
    #   |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
    def test(self):
        pose_frame = PoseStamped()
        pose_frame.header.frame_id = 'map'
        pose_frame.header.stamp = self.get_clock().now().to_msg()
        pose_frame.pose.position.x = -0.5
        pose_frame.pose.position.y = 0.0
        pose_frame.pose.position.z = 0.0
        # pose_map = self.transform_pose(pose_frame, "map")
        # print(pose_map)
        self.send_goal_pose(pose_frame)

    def calc_cmd_from_gaze(self) -> None:
        ''' 
        Calculate the neccessary ROS Twist input message to 
        follow the gaze in the robots image frame 
        '''
        if (not self.robodog_gaze_queue.empty() and
            not self.robodog_gaze_queue.empty() and
                self.depth_cam_info != None):
            # get data from queue
            gaze_msg = self.robodog_gaze_queue.get()
            depth_msg = self.robodog_depth_queue.get()
            # calc gaze offset from image center (relative to image size)
            gaze_offset = self.calc_gaze_offset(gaze_msg)
            # now calc control command.
            twist = self.calc_twist_from_offset(gaze_offset)
            # calculate the 3D coordinates of the gaze
            pose_map = self.calc_gaze_to_world(depth_msg, gaze_msg)
            # finally publish this commad to robot
            self.publish_cmd_vel_msg(twist, self.robodog_ctrl_pub)

    def calc_gaze_offset(self, gaze: GazeStamped) -> np.ndarray:
        # calc center point of image
        center = {
            "x": gaze.image_size.width/2,
            "y": gaze.image_size.height/2
        }
        # get gaze in relation to center point
        return [(gaze.gaze.x - center['x'])/gaze.image_size.width,
                (gaze.gaze.y - center['y'])/gaze.image_size.height]

    def calc_twist_from_offset(self, offset: np.array) -> Twist:
        # (yaw control needs to be positiv when turning counter clockwise).
        # also make the control speed dependent on the proximity of gaze
        # to center
        twist = Twist()
        # for now only set angular val
        twist.angular.z = -offset[0]
        return twist

    def calc_gaze_to_world(self, depth_msg: Image, gaze_msg: GazeStamped) -> tuple[float, float, float]:
        # convert depth image to usable format
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        # first retrive depth info at gaze location (do some smoothing with a bounding
        # box around that pixel)
        w = 10  # width of the bounding box
        small_box = depth_img[
            round(gaze_msg.gaze.x-w/2):round(gaze_msg.gaze.x+w/2),
            round(gaze_msg.gaze.y-w/2):round(gaze_msg.gaze.y+w/2)]

        # and take the average of all valid (non-zero) points
        depth = float(np.mean(small_box[np.nonzero(small_box)]))

        pose_frame = self.pixel_to_pose(gaze_msg.gaze.x,
                                        gaze_msg.gaze.y,
                                        depth)
        print(pose_frame)
        # transform to map frame
        pose_map = self.transform_pose(pose_frame, "map")
        print(pose_map)
        return pose_map

    #    ___       _ _
    #   |_ _|_ __ (_) |_
    #    | || '_ \| | __|
    #    | || | | | | |_
    #   |___|_| |_|_|\__|

    def init_queues(self) -> None:
        self.robodog_gaze_queue = Queue(maxsize=self.max_queue_size)
        self.robodog_depth_queue = Queue(maxsize=self.max_queue_size)

    def init_ros(self) -> None:
        #   use cv bridge to handle cv2 to ROS convertion
        self.cv_bridge = CvBridge()
        #   publisher for robodog contorl (TODO)
        self.robodog_ctrl_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robodog_ctrl_pose_pub = self.create_publisher(PoseStamped, '~/goal_pose', 10)
        self.robodog_ctrl_pub_timer = self.create_timer(
            0.01, self.robodog_ctrl_pub_callback)

        #   subscriber for the gaze data
        self.robodog_gaze_sub = self.create_subscription(
            GazeStamped,
            self.robodogNode + '/gaze',
            self.robodog_gaze_sub_callback,
            10)
        self.robodog_gaze_sub  # prevent unused variable warning

        #   subscriber for the depth data
        self.robodog_depth_sub = self.create_subscription(
            Image,
            self.robodogNode + '/aligned_depth_to_color/image_raw',
            self.robodog_depth_sub_callback,
            10)
        self.robodog_depth_sub  # prevent unused variable warning

        #   subscriber for the depth cam info data
        self.robodog_depth_info_sub = self.create_subscription(
            CameraInfo,
            self.robodogNode + '/aligned_depth_to_color/camera_info',
            self.robodog_depth_info_sub_callback,
            10)
        self.robodog_depth_info_sub  # prevent unused variable warning
    #    _   _ _   _ _
    #   | | | | |_(_) |___
    #   | | | | __| | / __|
    #   | |_| | |_| | \__ \
    #    \___/ \__|_|_|___/

    def queue_msg(self, msg, queue) -> None:
        '''
        Queue incoming ROS messages in queues for later use
        '''
        # check if we are full
        if (queue.qsize() >= self.max_queue_size):
            # pop oldest item
            queue.get()
        # add new item
        queue.put(msg)

    def set_depth_cam_info(self, msg) -> None:
        if (self.depth_cam_info == None):
            self.depth_cam_info = msg

    def get_gaze_from_msg(self, msg: GazeStamped) -> np.ndarray:
        gaze = [msg.gaze.x, msg.gaze.y]
        return gaze

    def publish_cmd_vel_msg(self, cmd_msg, publisher) -> None:
        publisher.publish(cmd_msg)

    def pixel_to_point(self, px, py, depth) -> tuple[float, float, float]:
        """Converts a specific pixel to a metric 3D point in the camera frame

        Args:
            px: value between 0 and image width from left to right
            py: value between 0 and image height from top to bottom
            depth: distance in mm

        Returns:
            (X,Y,Z) coordinates


        See the following links for implementation details:
        https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/box_dimensioner_multicam/helper_functions.py
        https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#intrinsic-camera-parameters
        http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
        """
        fx = self.depth_cam_info.k[0]
        fy = self.depth_cam_info.k[4]
        ppx = self.depth_cam_info.k[2]
        ppy = self.depth_cam_info.k[5]
        Z = depth / 1000

        X = float((px - ppx) / fx * Z)
        Y = float((py - ppy) / fy * Z)
        Z = float(Z)

        return X, Y, Z

    def pixel_to_pose(self, px, py, depth) -> PoseStamped:
        point = self.pixel_to_point(px, py, depth)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        return pose

    def transform_pose(self, input_pose: PoseStamped, to_frame) -> PoseStamped:
        """
        Args:
            input_pose (Pose): pose to be transformed.
            to_frame: name of the desired frame
            header: input pose header

        Returns:
            pose_map (Pose): transformed Pose in the new coordinate frame"""
        try:
            pose_map = self.tf_listener.transformPose(to_frame, input_pose)
            return pose_map
        except:
            self.get_logger().warning("Transformation failed")

    # def go_to_pose(self, pose):
    #     # Sends a `NavToPose` action request and waits for completion
    #     self.get_logger().debug("Waiting for 'NavigateToPose' action server")
    #     # while not self.client.wait_for_server(timeout_sec=1.0):
    #     #     self.get_logger().info("'NavigateToPose' action server not available, waiting...")

    #     goal_msg = NavigateToPose.Goal()
    #     goal_msg.pose = pose

    #     self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
    #               str(pose.pose.position.y) + '...')
    #     send_goal_future = self.client.send_goal_async(goal_msg,
    #                                                    self._feedbackCallback)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' +
    #                    str(pose.pose.position.y) + ' was rejected!')
    #         return False

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True

    # def _feedbackCallback(self, msg):
    #     self.feedback = msg.feedback
    #     return

    def send_goal_pose(self, pose: PoseStamped) -> None:
        '''
        Because the great wizzards of ROS decided to completly redo the packeg that 
        we use for navigation we can not use ROS actions directly. We will send the 
        calculated pose to a new ROS1 node that then handles navigation! YEAH! This
        did not totally take me a couple of days to figure out ...
        '''
        self.robodog_ctrl_pose_pub.publish(pose)

def main(args=None):
    '''
    Main! What more explanation do you need?
    '''
    # init for all ROS things
    rclpy.init(args=args)

    # init glasses (give an IP address if necessary!
    #  check in neon companion android app)
    robodogCtrl = RobodogCtrl()

    # Spin ROS
    rclpy.spin(robodogCtrl)

    # clean up
    rclpy.shutdown()


if __name__ == "__main__":
    main()
