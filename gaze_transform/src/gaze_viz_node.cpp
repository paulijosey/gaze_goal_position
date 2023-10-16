#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/types.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <cstdio>
#include <queue>

// ROS stuff imports
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
// #include "sensor_msgs/msg/image_const_ptr.hpp"
using namespace std::literals::chrono_literals;

class GazeViz : public rclcpp::Node
{
public:
    //    ____                _                   _
    //   / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __
    //  | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__|
    //  | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |
    //   \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|
    GazeViz() : Node("GazeViz")
    {
        // init the subscribers to listen to external device control (e.g.
        // a camera and gaze sensor)
        // 		- topic name
        //		- buffer (I think)
        // 		- callback function
        glassesGazeSub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            externalDevice + "gaze",
            10,
            std::bind(&GazeViz::glasses_gaze_sub_callback, this, std::placeholders::_1));

        glassesCamSub = this->create_subscription<sensor_msgs::msg::Image>(
            externalDevice + "cam_outward",
            10,
            std::bind(&GazeViz::glasses_cam_sub_callback, this, std::placeholders::_1));
        // init the subscribers to listen to internal device (e.g.
        // a camera)
        robodogCamSub = this->create_subscription<sensor_msgs::msg::Image>(
            internalDevice + "image_raw",
            10,
            std::bind(&GazeViz::robodog_cam_sub_callback, this, std::placeholders::_1)); //

        robodogGazeSub = this->create_subscription<geometry_msgs::msg::PointStamped>(
            internalDevice + "gaze",
            10,
            std::bind(&GazeViz::robodog_gaze_sub_callback, this, std::placeholders::_1));
        // init publisher(s). For now this is just the robodogs gaze (so the gaze from
        // the glasses transformed to the robodogs image)
        // 		- topic name
        //		- buffer (I think)
        robodogCamGazePub = this->create_publisher<sensor_msgs::msg::Image>(
            "~/" + internalDevice + "cam_gaze",
            10);

        glassesCamGazePub = this->create_publisher<sensor_msgs::msg::Image>(
            "~/" + externalDevice + "cam_gaze",
            10);

        // now use a timer function to compute the transformed gaze in regular intervals
        // and publish it.
        robodogCamGazeTimer = this->create_wall_timer(
            robodogCamGazePubInterval,
            std::bind(&GazeViz::viz_robodog_gaze, this));

        glassesCamGazeTimer = this->create_wall_timer(
            glassesCamGazePubInterval,
            std::bind(&GazeViz::viz_glasses_gaze, this));
    }

private:
    //    ____      _ _ _                _
    //   / ___|__ _| | | |__   __ _  ___| | _____
    //  | |   / _` | | | '_ \ / _` |/ __| |/ / __|
    //  | |__| (_| | | | |_) | (_| | (__|   <\__ \
	//   \____\__,_|_|_|_.__/ \__,_|\___|_|\_\___/
    void glasses_gaze_sub_callback(const geometry_msgs::msg::PointStamped &gazeMsg)
    {
        // get gaze data and save in queue
        push_to_queue(gazeMsg, glassesGazeBuf);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Gaze Queue Size: " << glassesGazeBuf.size());
    }

    void glasses_cam_sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr &camMsg)
    {
        // get cam data and save in queue
        push_to_queue(camMsg, glassesCamBuf);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Glasses Cam Queue Size: " << glassesCamBuf.size());
    }

    void robodog_gaze_sub_callback(const geometry_msgs::msg::PointStamped &gazeMsg)
    {
        // get gaze data and save in queue
        push_to_queue(gazeMsg, robodogGazeBuf);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Gaze Queue Size: " << robodogGazeBuf.size());
    }

    void robodog_cam_sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr &camMsg)
    {
        // get cam data and save in queue
        push_to_queue(camMsg, robodogCamBuf);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Robodog Cam Queue Size: " << robodogCamBuf.size());
    }

    void viz_glasses_gaze()
    {
        // check if the queues are not empty
        if (!glassesCamBuf.empty() && !glassesGazeBuf.empty())
        {
            // convert image to CV opject
            cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(glassesCamBuf.front());
            // viz data
            draw_gaze_in_img(glassesGazeBuf.front(),
                             cv_img,
                             glassesCamGazePub,
                             sensor_msgs::image_encodings::BGR8);
            // remove entries from buffer
            glassesGazeBuf.pop();
            glassesCamBuf.pop();
        }
    }

    void viz_robodog_gaze()
    {
        // check if the queues are not empty
        if (!robodogCamBuf.empty() && !robodogGazeBuf.empty())
        {
            // convert image to CV opject
            cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(robodogCamBuf.front());
            // viz data
            draw_gaze_in_img(robodogGazeBuf.front(),
                             cv_img,
                             robodogCamGazePub, 
                             sensor_msgs::image_encodings::RGB8);
            // remove entries from buffer
            robodogGazeBuf.pop();
            robodogCamBuf.pop();
        }
    }
    //   _____                 _   _
    //  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
    //  | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
    //  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
	//  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/

    /**
     * @brief 	Take an incoming ros message and push it to the according queue buffer.
     * 			If the queue is full remove the oldest message beforhand.
     *
     * @tparam 		T	Template param to allow different ros message types to be used.
     * @param msg 	T 	Ros message that should be pushed to an according queue buffer.
     * 					Make sure the queue buffer exists and has the right type!
     * 					This function uses a template as an input so we can use it for
     * 					multiple message types and have less code duplication 😎
     * @param queue 	std::queue<T> 	Queue to be pushed to.
     */
    template <typename T>
    void push_to_queue(const T &msg,
                       std::queue<T> &queue)
    {
        // check if queue is full
        if (queue.size() >= maxQueueSize)
        {
            // remove oldest entry
            queue.pop();
        }
        // add new entry
        queue.push(msg);
    }

    void draw_gaze_in_img(geometry_msgs::msg::PointStamped &gaze,
                          cv_bridge::CvImageConstPtr img,
                          rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub,
                          const std::string encoding)
    {
        // init as cv bridge object for later publishing
        cv_bridge::CvImage img_bridge;
        cv::Point2f gazeCv;
        gazeCv.x = gaze.point.x;
        gazeCv.y = gaze.point.y;
        // draw circle
        cv::circle(img->image, gazeCv, 30, cv::Scalar(0, 0, 255), 10, cv::LINE_8, 0);
        // Publish
        sensor_msgs::msg::Image imgMsg;
        imgMsg.header.stamp = this->get_clock()->now();
        img_bridge = cv_bridge::CvImage(imgMsg.header, encoding, img->image);
        img_bridge.toImageMsg(imgMsg); // from cv_bridge to sensor_msgs::Image
        pub->publish(imgMsg);
    }

    //  __     __         _       _     _
    //  \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___
    //   \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
    //    \ V / (_| | |  | | (_| | |_) | |  __/\__ \
	//     \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
    // subscriber(s)
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr glassesGazeSub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr robodogGazeSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr glassesCamSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robodogCamSub;
    // publisher(s)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr robodogCamGazePub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr glassesCamGazePub;
    // timer(s) for the publishing cycle
    rclcpp::TimerBase::SharedPtr glassesCamGazeTimer;
    rclcpp::TimerBase::SharedPtr robodogCamGazeTimer;
    std::chrono::duration<float> glassesCamGazePubInterval = std::chrono::milliseconds(200ms);
    std::chrono::duration<float> robodogCamGazePubInterval = std::chrono::milliseconds(200ms);
    // node that streams the external device data
    std::string externalDevice = "PupilLabsStream/";
    std::string internalDevice = "color/";
    // buffers for incoming data
    std::queue<geometry_msgs::msg::PointStamped> glassesGazeBuf;
    std::queue<geometry_msgs::msg::PointStamped> robodogGazeBuf;
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> glassesCamBuf;
    std::queue<sensor_msgs::msg::Image::ConstSharedPtr> robodogCamBuf;
    const uint maxQueueSize = 1;
    // opencv data objects
    cv_bridge::CvImageConstPtr glassesCamCvPtr;
    cv_bridge::CvImageConstPtr robodogCamCvPtr;
    cv::Point2f glassesGazeCv;
    cv::Point2f robodogGazeCv;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GazeViz>());
    rclcpp::shutdown();
    return 0;
}
