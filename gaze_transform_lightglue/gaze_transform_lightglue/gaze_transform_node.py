from lightglue import LightGlue, SuperPoint, DISK, match_pair, viz2d
from lightglue.utils import load_image, rbd, numpy_image_to_torch
import torch
import cv2
import numpy as np
from queue import Queue 

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
 
class GazeTransform(Node):
    #      ____                _              _
    #     / ___|___  _ __  ___| |_ _   _  ___| |_ ___  _ __
    #    | |   / _ \| '_ \/ __| __| | | |/ __| __/ _ \| '__|
    #    | |__| (_) | | | \__ \ |_| |_| | (__| || (_) | |
    #     \____\___/|_| |_|___/\__|\__,_|\___|\__\___/|_|
    def __init__(self) -> None:
        super().__init__('gaze_transform_node')
        # customizable params
        self.viz_results = False    # save matching results in data folder
        self.use_grayscale = False   # use grayscale images for faster matching (maybe)
        self.max_num_features = 256 # max number of features for detection (lower should be faster)
        self.use_ransac = False     # use ransac for filtering
        self.glassesNode = '/smart_glasses'    # node name for the smartglasses
        self.robodogNode = '/robodog_camera/color'    # node name for the robodog cam
        self.max_queue_size = 1     # max images saved in queue

        # init data queues
        self.glasses_gaze_queue = Queue(maxsize=self.max_queue_size)
        self.glasses_cam_queue = Queue(maxsize=self.max_queue_size)
        self.robodog_cam_queue = Queue(maxsize=self.max_queue_size)

        # init lightglue 
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 'mps', 'cpu'
        self.init_lightglue()

        # init ROS stuff
        self.init_ros()
 
    #    ____      _ _ _                _        
    #   / ___|__ _| | | |__   __ _  ___| | _____ 
    #  | |   / _` | | | '_ \ / _` |/ __| |/ / __|
    #  | |__| (_| | | | |_) | (_| | (__|   <\__ \
    #   \____\__,_|_|_|_.__/ \__,_|\___|_|\_\___/
    def glasses_gaze_sub_callback(self, msg) -> None:
        self.queue_msg(msg, self.glasses_gaze_queue)

    def glasses_cam_sub_callback(self, msg) -> None:
        self.queue_msg(msg, self.glasses_cam_queue)

    def robodog_cam_sub_callback(self, msg) -> None:
        self.queue_msg(msg, self.robodog_cam_queue)

    def robodog_gaze_pub_callback(self):
        self.match_gaze()
    #    _____                 _   _                 
    #   |  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
    #   | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
    #   |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
    #   |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
    def match_gaze(self):
        if (not self.glasses_gaze_queue.empty() and 
            not self.glasses_cam_queue.empty() and 
            not self.robodog_cam_queue.empty()):
            # get images from ROS message
            glasses_img = self.get_img_from_msg(self.glasses_cam_queue.get())
            robodog_img = self.get_img_from_msg(self.robodog_cam_queue.get())
            glasses_gaze = self.get_gaze_from_msg(self.glasses_gaze_queue.get())
            # convert to tensor (because lightglue wants that)
            robodog_img_tensor = numpy_image_to_torch(robodog_img)
            glasses_img_tensor = numpy_image_to_torch(glasses_img)
            # now actually match
            kpts0, kpts1, matches01 = match_pair(self.extractor, 
                                                 self.matcher, 
                                                 glasses_img_tensor,
                                                 robodog_img_tensor)

            matches = matches01['matches']  # indices with shape (K,2)
            m_kpts0 = kpts0['keypoints'][matches[..., 0]]  # coordinates in image #0, shape (K,2)
            m_kpts1 = kpts1['keypoints'][matches[..., 1]]  # coordinates in image #1, shape (K,2)

            # skip this turn if we didn't find enough matches
            if(m_kpts0.size(dim=0) < 4):
                print("could not find enough matches! Try again next turn")
                return

            # find homography to transform pixels from img to img
            if(self.use_ransac):
                # apply ransac
                h, status = cv2.findHomography(m_kpts0.detach().to(self.device).numpy(), 
                                               m_kpts1.detach().to(self.device).numpy(),
                                               cv2.RANSAC,
                                               1.0)
            else:
                h, status = cv2.findHomography(m_kpts0.detach().to(self.device).numpy(), 
                                               m_kpts1.detach().to(self.device).numpy())

            m_kpts0_good = torch.empty((0, 2))
            m_kpts1_good = torch.empty((0, 2))

            count=0
            for stat in status:
                if stat==1:
                    m_kpts0_good = torch.cat((m_kpts0_good, torch.index_select(m_kpts0, 0, torch.tensor([count]))), 0)
                    m_kpts1_good = torch.cat((m_kpts1_good, torch.index_select(m_kpts1, 0, torch.tensor([count]))), 0)
                count = count+1

            if(self.viz_results):
                axes = viz2d.plot_images([glasses_img_tensor, robodog_img_tensor])
                viz2d.plot_matches(m_kpts0, m_kpts1, color="lime", lw=0.2)
                viz2d.add_text(0, f'Stop after {matches01["stop"]} layers', fs=20)
                viz2d.save_plot("/home/ws/src/robodog_control/gaze_transform_lightglue/data/matches.png")

                if(self.use_ransac):
                    axes1 = viz2d.plot_images([glasses_img_tensor, robodog_img_tensor])
                    viz2d.plot_matches(m_kpts0_good, m_kpts1_good, color="red", lw=0.2)
                    viz2d.add_text(0, f'Stop after {matches01["stop"]} layers + RANSAC', fs=20)
                    viz2d.save_plot("/home/ws/src/robodog_control/gaze_transform_lightglue/data/matches_RANSAC.png")
            
            # transform gaze to robodog frame
            robodog_gaze = self.transform_gaze(h, glasses_gaze)
            # publish gaze as ros message
            self.publish_gaze_msg(robodog_gaze, self.robodog_gaze_pub)

        
    def transform_gaze(self, h: np.array, gaze: np.array) -> np.array:
        # use computer vision magic 😎
        p = np.array((gaze[0],gaze[1],1)).reshape((3,1))
        temp_p = h.dot(p)
        sum = np.sum(temp_p ,1)
        px = int(round(sum[0]/sum[2]))
        py = int(round(sum[1]/sum[2]))
        return [px, py]
 
    #    ___       _ _   
    #   |_ _|_ __ (_) |_ 
    #    | || '_ \| | __|
    #    | || | | | | |_ 
    #   |___|_| |_|_|\__|
    def init_lightglue(self):                      
        # DISK+LightGlue
        # self.extractor = DISK(max_num_keypoints=self.max_num_features).eval().to(self.device)  # load the extractor
        # or SuperPoint+LightGlue
        self.extractor = SuperPoint(max_num_keypoints=self.max_num_features).eval().to(self.device)  # load the extractor
        self.matcher = LightGlue(features='superpoint', 
                                 depth_confidence=0.9, 
                                 width_confidence=0.95, 
                                 mode='reduce-overhead').eval().to(self.device)  # load the matcher

    def init_ros(self):
        #   use cv bridge to handle cv2 to ROS convertion
        self.cv_bridge = CvBridge()
        #   publisher for pretty pictures + gaze estimate
        self.robodog_gaze_pub = self.create_publisher(PointStamped, self.robodogNode + '/gaze', 10)
        self.robodog_gaze_pub_timer = self.create_timer(0.2, self.robodog_gaze_pub_callback)

        #   subscriber for the gaze data
        self.glasses_gaze_sub = self.create_subscription(
            PointStamped,
            self.glassesNode + '/gaze',
            self.glasses_gaze_sub_callback,
            10)
        self.glasses_gaze_sub # prevent unused variable warning
        #   subscriber for the cam data
        self.glasses_cam_sub = self.create_subscription(
            Image,
            self.glassesNode + '/cam_outward',
            self.glasses_cam_sub_callback,
            10)
        self.glasses_cam_sub # prevent unused variable warning
        #   subscriber for the cam data
        self.robodog_cam_sub = self.create_subscription(
            Image,
            self.robodogNode + '/image_raw',
            self.robodog_cam_sub_callback,
            10)
        self.robodog_cam_sub # prevent unused variable warning

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
        if(queue.qsize() >= self.max_queue_size):
            # pop oldest item
            queue.get()
        # add new item 
        queue.put(msg)

    def get_img_from_msg(self, msg: Image) -> torch.Tensor:
        if(self.use_grayscale):
            return cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(msg), cv2.COLOR_BGR2GRAY)
        else:
            return self.cv_bridge.imgmsg_to_cv2(msg)

    def get_gaze_from_msg(self, msg: PointStamped) -> np.array:
        gaze = [msg.point.x, msg.point.y]
        return gaze

    def publish_gaze_msg(self, gaze, publisher) -> None:
        timestamp = self.get_clock().now().to_msg() # for now lets use current system time
        gaze_msg = PointStamped()
        gaze_msg.header.stamp = timestamp
        gaze_msg.point.x = gaze[0]*1.0
        gaze_msg.point.y = gaze[1]*1.0
        publisher.publish(gaze_msg)

def main(args=None):
    '''
    Main! What more explanation do you need?
    '''
    # init for all ROS things
    rclpy.init(args=args)

    # init glasses (give an IP address if necessary! 
    #  check in neon companion android app)
    gazeTransform = GazeTransform()

    # Spin ROS
    rclpy.spin(gazeTransform)

    # clean up
    rclpy.shutdown()

if __name__ == "__main__":
    main()
