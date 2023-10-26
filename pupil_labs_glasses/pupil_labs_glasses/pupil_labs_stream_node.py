# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    pupil_labs_stream_node.py                          :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: Paul Joseph <paul.joseph@pbl.ee.ethz.ch    +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2023/10/04 09:00:11 by Paul Joseph       #+#    #+#              #
#    Updated: 2023/10/17 08:58:47 by Paul Joseph      ###   ########.fr        #
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
from std_msgs.msg import String
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

class SmartGlasses(Node):
    #      ____                _              _
    #     / ___|___  _ __  ___| |_ _   _  ___| |_ ___  _ __
    #    | |   / _ \| '_ \/ __| __| | | |/ __| __/ _ \| '__|
    #    | |__| (_) | | | \__ \ |_| |_| | (__| || (_) | |
    #     \____\___/|_| |_|___/\__|\__,_|\___|\__\___/|_|
    def __init__(self, ip='10.5.50.232') -> None:
        super().__init__('PupilLabsStream')
        self.ip = ip
        self.port = 8080  # this might change ... but keep it hardcoded for now
        self.recording_id = ''

        # init connection (this will init self.device)
        asyncio.run(self.neon_companion_network_conf())
        # Get device status and info (this will init self.cam_outward & self.gaze)
        asyncio.run(self.get_neon_companion_info())

        # init ROS stuff
        #   use cv bridge to handle cv2 to ROS convertion
        self.cv_bridge = CvBridge()
        #   publisher for pretty pictures
        self.cam_outward_pub = self.create_publisher(Image, 'cam_outward', 10)
        #   publisher for the gaze data
        self.gaze_pub = self.create_publisher(PointStamped, 'gaze', 10)
        #   publisher for the imu data
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

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
            self.device = await network.wait_for_new_device(timeout_seconds=5)

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
            print(f"Phone IP address: {status.phone.ip}")
            print(f"Battery level: {status.phone.battery_level}%")

            print(f"Connected glasses: SN {status.hardware.glasses_serial}")
            print(f"Connected scene camera: SN {status.hardware.world_camera_serial}")
    
            self.cam_outward = status.direct_world_sensor()
            print(f"World sensor: connected={self.cam_outward.connected} url={self.cam_outward.url}")
    
            self.imu = status.direct_imu_sensor()
            print(f"IMU sensor: connected={self.imu.connected} url={self.imu.url}")

            self.gaze = status.direct_gaze_sensor()
            print(f"Gaze sensor: connected={self.gaze.connected} url={self.gaze.url}")

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
        queue_imu = asyncio.Queue()

        # get image, imu and gaze data from pupil labs glasses 
        # and figure out which data entries match
        process_cam_outward = asyncio.create_task(
            self.enqueue_sensor_data(
                receive_video_frames(self.cam_outward.url, run_loop=restart_on_disconnect),
                queue_cam_outward,
            )
        )
        process_imu = asyncio.create_task(
            self.enqueue_sensor_data(
                receive_imu_data(self.imu.url, run_loop=restart_on_disconnect),
                queue_imu,
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
                _, imu  = await self.get_closest_item(queue_imu,  video_datetime)
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
                gaze_msg = PointStamped()
                gaze_msg.header.stamp = timestamp
                gaze_msg.point.x = gaze.x
                gaze_msg.point.y = gaze.y
                self.gaze_pub.publish(gaze_msg)
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
            process_cam_outward.cancel()
            process_gaze.cancel()
            process_imu.cancel()

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

    # init glasses (give an IP address if necessary! 
    #  check in neon companion android app)
    glasses = SmartGlasses()
    # Publish cam and gaze data
    asyncio.run(glasses.stream_outward_cam_and_gaze())

    # Spin ROS
    rclpy.spin(glasses)

    # clean up
    glasses.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
