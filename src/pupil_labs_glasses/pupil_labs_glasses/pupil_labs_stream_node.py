from pupil_labs.realtime_api.simple import Device
import time
import cv2 as cv
# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class SmartGlasses(Node):
    #      ____                _              _
    #     / ___|___  _ __  ___| |_ _   _  ___| |_ ___  _ __
    #    | |   / _ \| '_ \/ __| __| | | |/ __| __/ _ \| '__|
    #    | |__| (_) | | | \__ \ |_| |_| | (__| || (_) | |
    #     \____\___/|_| |_|___/\__|\__,_|\___|\__\___/|_|
    def __init__(self, ip) -> None:
        super().__init__('PupilLabsStream')
        self.ip = ip
        self.port = 8080  # this might change ... but keep it hardcoded for now
        self.recording_id = ''

        # init connection (this will init self.device)
        self.neon_companion_network_conf()

        # init ROS stuff
        #   use cv bridge to handle cv2 to ROS convertion
        self.cv_bridge = CvBridge()
        #   publisher for pretty pictures
        self.cam_outward_pub = self.create_publisher(Image, 'cam_outward', 10)
        #   publisher for the gaze data
        self.gaze_pub = self.create_publisher(Point, 'gaze', 10)

    #    _   _      _                      _    _
    #   | \ | | ___| |___      _____  _ __| | _(_)_ __   __ _
    #   |  \| |/ _ \ __\ \ /\ / / _ \| '__| |/ / | '_ \ / _` |
    #   | |\  |  __/ |_ \ V  V / (_) | |  |   <| | | | | (_| |
    #   |_| \_|\___|\__| \_/\_/ \___/|_|  |_|\_\_|_| |_|\__, |
    #                                                   |___/

    def neon_companion_network_conf(self) -> None:
        '''
        Connect this python instance to the host device (aka phone)
        '''
        self.device = Device(address=self.ip, port=self.port)
        if self.device is None:
            print("No device found.")
            raise SystemExit(-1)

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
    def print_neon_companion_info(self) -> None:
        '''
        Print all information about the host device (aka phone)
        '''
        print(f"Phone IP address: {self.device.phone_ip}")
        print(f"Phone name: {self.device.phone_name}")
        print(f"Phone unique ID: {self.device.phone_id}")

        print(f"Battery level: {self.device.battery_level_percent}%")
        print(f"Battery state: {self.device.battery_state}")

        print(f"Free storage: {self.device.memory_num_free_bytes / 1024**3}GB")
        print(f"Storage level: {self.device.memory_state}")

        print(f"Connected glasses: SN {self.device.serial_number_glasses}")
        print(
            f"Connected scene camera: SN {self.device.serial_number_scene_cam}")

    #   ____            _             _
    #  / ___|___  _ __ | |_ _ __ ___ | |
    # | |   / _ \| '_ \| __| '__/ _ \| |
    # | |__| (_) | | | | |_| | | (_) | |
    #  \____\___/|_| |_|\__|_|  \___/|_|
    def start_recording(self) -> None:
        '''
        Start recording on an already connected device
        '''
        self.recording_id = self.device.recording_start()

    def stop_recording(self) -> None:
        '''
        Stop recording on an already connected device
        and reset the recording id to ''
        '''
        self.device.recording_stop_and_save()
        self.recording_id = ''
    
    def record(self, timespan) -> None:
        '''
        Record for a given timespan 
        '''
        self.start_recording()
        time.sleep(timespan)
        self.stop_recording()

    def stream_gaze(self) -> None:
        '''
        Stream Gaze data
        '''
        try:
            while True:
                print(self.device.receive_gaze_datum())
        except KeyboardInterrupt:
            pass

    def stream_gaze_cam(self) -> None:
        '''
        Stream gaze camera
        '''
        try:
            while True:
                bgr_pixels, frame_datetime = self.device.receive_eyes_video_frame()
                cv.imwrite('/workspace/data/gaze_cam.png', bgr_pixels)
        except KeyboardInterrupt:
            pass

    def stream_outward_cam_and_gaze(self) -> None:
        '''
        Stream outward camera and gaze data
        '''
        try:
            while True:
                # get image data and gaze data from pupil labs glasses
                frame, gaze = self.device.receive_matched_scene_video_frame_and_gaze()
                # publish them in ros messages
                #   First handle the outward image
                #   frame consists of:
                #       frame.bgr_pixels 
                #       frame.timestamp_unix_sec
                cam_outward_msg = self.cv_bridge.cv2_to_imgmsg(frame.bgr_pixels, encoding="passthrough")
                self.cam_outward_pub.publish(cam_outward_msg)
                #   Next the gaze info
                #   Gaze consists of:
                #       gaze.x
                #       gaze.y
                #       gaze.worn
                #       gaze.timestamp_unix_sec
                gaze_msg = Point()
                gaze_msg.x = gaze.x
                gaze_msg.y = gaze.y
                self.gaze_pub.publish(gaze_msg)


        except KeyboardInterrupt:
            pass

def main(args=None):
    '''
    Main! What more explanation do you need?
    '''
    # init for all ROS things
    rclpy.init(args=args)

    # init glasses
    glasses = SmartGlasses("10.5.50.232")
    glasses.print_neon_companion_info()
    glasses.stream_outward_cam_and_gaze()

    rclpy.spin(glasses)

    glasses.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
