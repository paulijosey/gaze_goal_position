# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    pupil_labs_stream_node.py                          :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: Paul Joseph <paul.joseph@pbl.ee.ethz.ch    +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2023/10/04 09:00:11 by Paul Joseph       #+#    #+#              #
#    Updated: 2024/04/04 15:35:56 by Paul Joseph      ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

from pupil_labs.realtime_api import Device, Network, models, receive_video_frames, receive_gaze_data, receive_imu_data
import asyncio
import typing as T
import scipy
import math 
# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, Imu, CameraInfo
from gaze_msgs.msg import GazeStamped
from cv_bridge import CvBridge

class SmartGlasses(Node):
    #      ____                _              _
    #     / ___|___  _ __  ___| |_ _   _  ___| |_ ___  _ __
    #    | |   / _ \| '_ \/ __| __| | | |/ __| __/ _ \| '__|
    #    | |__| (_) | | | \__ \ |_| |_| | (__| || (_) | |
    #     \____\___/|_| |_|___/\__|\__,_|\___|\__\___/|_|
    def __init__(self) -> None:
        super().__init__('PupilLabsStream')
        self.inti_ros()

        # init variables
        self.ip = self.get_parameter('ip_addr').get_parameter_value().string_value
        self.port = 8080  # this might change ... but keep it hardcoded for now
        self.recording_id = ''

        print(self.ip)

        # init connection (this will init self.device)
        asyncio.run(self.neon_companion_network_conf())
        # Get device status and info (this will init self.cam_outward & self.gaze)
        asyncio.run(self.get_neon_companion_info())
        asyncio.run(self.set_calibration())
 
    #    ___       _ _   
    #   |_ _|_ __ (_) |_ 
    #    | || '_ \| | __|
    #    | || | | | | |_ 
    #   |___|_| |_|_|\__|
    def inti_ros(self):
        '''
        Init everything ROS related here. (pub/sub/parameters)
        '''
        # init ROS stuff
        #   use cv bridge to handle cv2 to ROS convertion
        self.cv_bridge = CvBridge()
        #   publisher for pretty pictures
        self.cam_outward_pub = self.create_publisher(Image, 'cam_outward', 10)
        #   publisher for the camera info
        self.cam_outward_info_pub = self.create_publisher(CameraInfo, 'cam_outward_info', 10)
        #   publisher for the gaze data
        self.gaze_pub = self.create_publisher(GazeStamped, 'gaze', 10)
        #   publisher for the imu data
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        #   parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('ip_addr', '10.5.51.42')
            ]
        )

    #    _   _      _                      _    _
    #   | \ | | ___| |___      _____  _ __| | _(_)_ __   __ _
    #   |  \| |/ _ \ __\ \ /\ / / _ \| '__| |/ / | '_ \ / _` |
    #   | |\  |  __/ |_ \ V  V / (_) | |  |   <| | | | | (_| |
    #   |_| \_|\___|\__| \_/\_/ \___/|_|  |_|\_\_|_| |_|\__, |
    #                                                   |___/
    async def neon_companion_network_conf(self) -> None:
        '''
        Connect this python instance to the host device (aka phone)
        This function will try to detect a pupil labs device on 
        the network. If it fails it will revert back to the manual 
        ip address given.
        '''
        async with Network() as network:
            # self.device = await network.wait_for_new_device(timeout_seconds=5)
            self.device = None

        if self.device is None:
            print("No device found. Using given IP for manual override")
            self.device = models.DiscoveredDeviceInfo('test', 'neon.local', 
                                                      self.port, [self.ip])
            # Device(address=self.ip, port=self.port)

    def close_neon_companion_connection(self) -> None:
        '''
        Close network connection to host device (aka phone)
        '''
        self.device.close()  # explicitly stop auto-update

    #  ____  _        _             
    # / ___|| |_ __ _| |_ _   _ ___ 
    # \___ \| __/ _` | __| | | / __|
    #  ___) | || (_| | |_| |_| \__ \
    # |____/ \__\__,_|\__|\__,_|___/
    async def get_neon_companion_info(self) -> None:
        '''
        Print all information about the host device (aka phone).
        Needs to be called during init! 
        '''
        async with Device.from_discovered_device(self.device) as device:
            status = await device.get_status()
            self.get_logger().info(f"Phone IP address: {status.phone.ip}")
            self.get_logger().info(f"Battery level: {status.phone.battery_level}%")

            self.get_logger().info(f"Connected glasses: SN {status.hardware.glasses_serial}")
            self.get_logger().info(f"Connected scene camera: SN {status.hardware.world_camera_serial}")
    
            self.cam_outward = status.direct_world_sensor()
            self.get_logger().info(f"World sensor: connected={self.cam_outward.connected} url={self.cam_outward.url}")
    
            self.cam_eyes = status.direct_eyes_sensor()
            self.get_logger().info(f"Eye sensor: connected={self.cam_eyes.connected} url={self.cam_eyes.url}")

            self.imu = status.direct_imu_sensor()
            self.get_logger().info(f"IMU sensor: connected={self.imu.connected} url={self.imu.url}")

            self.gaze = status.direct_gaze_sensor()
            self.get_logger().info(f"Gaze sensor: connected={self.gaze.connected} url={self.gaze.url}")


    async def set_calibration(self) -> None:
        '''
        Set the calibration for the glasses
        '''
        async with Device.from_discovered_device(self.device) as device:
            self.calibration = await device.get_calibration()
            # hacky parsing of calibration data (only need to do this once)
            k = []
            for i in range(3):
                for j in range(3):
                    k.append(self.calibration['scene_camera_matrix'][0][i][j])
            d = []
            for i in range(8):
                d.append(self.calibration['scene_distortion_coefficients'][0][i])
            # transform to ROS2 camera info messages
            #   scene camera
            self.cam_outward_info = CameraInfo()
            self.cam_outward_info.header.frame_id = "scene_camera"
            self.cam_outward_info.width = 1600
            self.cam_outward_info.height = 1200
            self.cam_outward_info.distortion_model = "plumb_bob"    # might be wrong
            self.cam_outward_info.d = d
            self.cam_outward_info.k = k

        return

    #   ____        _          ____  _                                
    #  |  _ \  __ _| |_ __ _  / ___|| |_ _ __ ___  __ _ _ __ ___  ___ 
    #  | | | |/ _` | __/ _` | \___ \| __| '__/ _ \/ _` | '_ ` _ \/ __|
    #  | |_| | (_| | || (_| |  ___) | |_| | |  __/ (_| | | | | | \__ \
    #  |____/ \__,_|\__\__,_| |____/ \__|_|  \___|\__,_|_| |_| |_|___/
    async def stream_outward_cam_and_gaze(self) -> None:
        '''
        Stream outward camera and gaze data. This is where the magic happens
        '''
        # init streamer tasks
        restart_on_disconnect = True
        queue_cam_outward = asyncio.Queue()
        queue_gaze = asyncio.Queue()

        # get image and gaze data from pupil labs glasses 
        # and figure out which data entries match
        process_cam_outward = asyncio.create_task(
            self.enqueue_sensor_data(
                receive_video_frames(self.cam_outward.url, run_loop=restart_on_disconnect),
                queue_cam_outward,
            )
        )
        process_gaze = asyncio.create_task(
            self.enqueue_sensor_data(
                receive_gaze_data(self.gaze.url, run_loop=restart_on_disconnect),
                queue_gaze,
            )
        )
        try:
            # match queues
            while True:
                video_datetime, frame = await self.get_most_recent_item(queue_cam_outward)
                _, gaze = await self.get_closest_item(queue_gaze, video_datetime)
                timestamp = self.get_clock().now().to_msg() # for now lets use current system time
                # publish in ros messages
                #   frame consists of:
                #       frame.bgr_buffer 
                #       frame.timestamp_unix_sec
                cam_outward_msg = self.cv_bridge.cv2_to_imgmsg(frame.bgr_buffer(), 
                                                               encoding="passthrough")
                cam_outward_msg.header.stamp = timestamp
                self.cam_outward_pub.publish(cam_outward_msg)
                #   Gaze consists of:
                #       gaze.x
                #       gaze.y
                #       gaze.worn
                #       gaze.timestamp_unix_sec
                gaze_msg = GazeStamped()
                gaze_msg.header.stamp = timestamp
                gaze_msg.gaze.x = gaze.x
                gaze_msg.gaze.y = gaze.y
                gaze_msg.image_size.height = cam_outward_msg.height
                gaze_msg.image_size.width = cam_outward_msg.width
                self.gaze_pub.publish(gaze_msg)

                # also publish the camera info here
                if self.cam_outward_info is not None:
                    self.cam_outward_info.header.stamp = timestamp
                    self.cam_outward_info_pub.publish(self.cam_outward_info)
        finally:
            process_cam_outward.cancel()
            process_gaze.cancel()


    async def stream_imu(self) -> None:
        '''
        read the IMU stream and publish it as ROS messages
        '''
        # init queue
        restart_on_disconnect = True
        queue_imu = asyncio.Queue()
        # get data
        process_imu = asyncio.create_task(
            self.enqueue_sensor_data(
                receive_imu_data(self.imu.url, run_loop=restart_on_disconnect),
                queue_imu,
            )
        )
        try:
            # match queues
            while True:
                imu_datetime, imu= await self.get_most_recent_item(queue_imu)
                timestamp = self.get_clock().now().to_msg() # for now lets use current system time
                #   imu consists of
                #       imu.gyro_data:
                #           x
                #           y
                #           z
                #       imu.accel_data:
                #           x
                #           y
                #           z
                #       imu.quaternion:
                #           x
                #           y
                #           z
                #           w
                #       imu.timestamp_unix_seconds
                imu_msg = Imu()
                imu_msg.header.stamp = timestamp
                imu_msg.angular_velocity.x = math.radians(imu.gyro_data.x)
                imu_msg.angular_velocity.y = math.radians(imu.gyro_data.y)
                imu_msg.angular_velocity.z = math.radians(imu.gyro_data.z)
                imu_msg.linear_acceleration.x = imu.accel_data.x*scipy.constants.g
                imu_msg.linear_acceleration.y = imu.accel_data.y*scipy.constants.g
                imu_msg.linear_acceleration.z = imu.accel_data.z*scipy.constants.g
                imu_msg.orientation.x = imu.quaternion.x
                imu_msg.orientation.y = imu.quaternion.y
                imu_msg.orientation.z = imu.quaternion.z
                imu_msg.orientation.w = imu.quaternion.w
                self.imu_pub.publish(imu_msg)
        finally:
            process_imu.cancel()

    async def stream_blink_signal(self) -> None:
        '''
        detect rapid 3 blinks and publish a signal
        '''
        return

    async def stream_data(self) -> None:
        await asyncio.gather(
            asyncio.create_task(self.stream_imu()),
            asyncio.create_task(self.stream_outward_cam_and_gaze()),
        )

    #   _   _ _   _ _      
    #  | | | | |_(_) |___  
    #  | | | | __| | / __| 
    #  | |_| | |_| | \__ \ 
    #   \___/ \__|_|_|___/ 
    async def enqueue_sensor_data(self, sensor: T.AsyncIterator, queue: asyncio.Queue) -> None:
        async for datum in sensor:
            try:
                queue.put_nowait((datum.datetime, datum))
            except asyncio.QueueFull:
                print(f"Queue is full, dropping {datum}")

    async def get_most_recent_item(self, queue):
        item = await queue.get()
        while True:
            try:
                next_item = queue.get_nowait()
            except asyncio.QueueEmpty:
                return item
            else:
                item = next_item

    async def get_closest_item(self, queue, timestamp):
        item_ts, item = await queue.get()
        # assumes monotonically increasing timestamps
        if item_ts > timestamp:
            return item_ts, item
        while True:
            try:
                next_item_ts, next_item = queue.get_nowait()
            except asyncio.QueueEmpty:
                return item_ts, item
            else:
                if next_item_ts > timestamp:
                    return next_item_ts, next_item
                item_ts, item = next_item_ts, next_item

def main(args=None):
    '''
    Main! What more explanation do you need?
    '''
    # init for all ROS things
    rclpy.init(args=args)

    # init glasses
    glasses = SmartGlasses()
    # Publish cam, gaze and IMU data
    asyncio.run(glasses.stream_data())

    # Spin ROS
    rclpy.spin(glasses)

    # clean up
    glasses.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
