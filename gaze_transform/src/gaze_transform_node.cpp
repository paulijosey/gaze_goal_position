/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   gaze_transform_node.cpp                            :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: Paul Joseph <paul.joseph@pbl.ee.ethz.ch    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2023/10/06 08:52:10 by Paul Joseph       #+#    #+#             */
/*   Updated: 2023/10/17 09:02:43 by Paul Joseph      ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
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
#include "image_transport/image_transport.hpp"
// #include "sensor_msgs/msg/image_const_ptr.hpp"
using namespace std::literals::chrono_literals;

bool compare_matches(std::vector<cv::DMatch> i1, std::vector<cv::DMatch> i2)
{
	return (i1[0].distance < i2[0].distance);
}

class GazeTransform : public rclcpp::Node
{
public:
	//    ____                _                   _
	//   / ___|___  _ __  ___| |_ _ __ _   _  ___| |_ ___  _ __
	//  | |   / _ \| '_ \/ __| __| '__| | | |/ __| __/ _ \| '__|
	//  | |__| (_) | | | \__ \ |_| |  | |_| | (__| || (_) | |
	//   \____\___/|_| |_|___/\__|_|   \__,_|\___|\__\___/|_|
	GazeTransform() : Node("gaze_transform_node")
	{
		// init the subscribers to listen to external device control (e.g.
		// a camera and gaze sensor)
		// 		- topic name
		//		- buffer (I think)
		// 		- callback function
		glassesGazeSub = this->create_subscription<geometry_msgs::msg::PointStamped>(
			externalDevice + "gaze",
			10,
			std::bind(&GazeTransform::glasses_gaze_sub_callback, this, std::placeholders::_1));

		glassesCamSub = this->create_subscription<sensor_msgs::msg::Image>(
			externalDevice + "cam_outward",
			10,
			std::bind(&GazeTransform::glasses_cam_sub_callback, this, std::placeholders::_1));
		// init the subscribers to listen to internal device (e.g.
		// a camera)
		robodogCamSub = this->create_subscription<sensor_msgs::msg::Image>(
			internalDevice + "image_raw",
			10,
			std::bind(&GazeTransform::robodog_cam_sub_callback, this, std::placeholders::_1)); //

		// init publisher(s). For now this is just the robodogs gaze (so the gaze from
		// the glasses transformed to the robodogs image)
		// 		- topic name
		//		- buffer (I think)
		robodogGazePub = this->create_publisher<geometry_msgs::msg::PointStamped>(
			internalDevice + "gaze",
			10);

		// now use a timer function to compute the transformed gaze in regular intervals
		// and publish it.
		robodogGazeTimer = this->create_wall_timer(
			robodogGazePubInterval,
			std::bind(&GazeTransform::match_gaze, this));
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

	void robodog_cam_sub_callback(const sensor_msgs::msg::Image::ConstSharedPtr &camMsg)
	{
		// get cam data and save in queue
		push_to_queue(camMsg, robodogCamBuf);
		RCLCPP_DEBUG_STREAM(this->get_logger(), "Robodog Cam Queue Size: " << robodogCamBuf.size());
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

	void match_gaze()
	{
		// convert ros images to opencv and get gaze info
		// but first check if the image buffers are not empty
		if (!glassesCamBuf.empty() &&
			!robodogCamBuf.empty() &&
			!glassesGazeBuf.empty())
		{
			try
			{
				// convert everything to opencv objects because they are easier to
				// handle with opencv methods/funcions.
				// Note: somehow setting an BGR8 image encoding fails on the glasses
				// 		object ...
				glassesCamCvPtr = cv_bridge::toCvShare(glassesCamBuf.front());
				robodogCamCvPtr = cv_bridge::toCvShare(robodogCamBuf.front(),
													   sensor_msgs::image_encodings::BGR8);
				glassesGazeCv.x = glassesGazeBuf.front().point.x;
				glassesGazeCv.y = glassesGazeBuf.front().point.y;
				glassesCamBuf.pop();
				robodogCamBuf.pop();
				glassesGazeBuf.pop();
			}
			catch (cv_bridge::Exception &e)
			{
				RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
				return;
			}
			// now start with the actual gaze matching.
			// We will use openCv's optical flow (lukas kanade) method.
			// We need to init input/output vectors for matching point because this
			// method is usually used to match mutliple points! HACKY WORKAROUND
			// WARNING! Finally we get the gaze from the camera in the robodogs frame.
			// Also resize the target image (robodog) because they need to be same size.
			// TODO: figure out how to handle this resizing for future control.
			std::vector<uchar> status;
			std::vector<float> err;
			std::vector<cv::Point2f> glassesGazeCvVec = {glassesGazeCv};
			cv::Mat glassesCamCvCopy = glassesCamCvPtr->image.clone();
			cv::Mat robodogCamCvCopy = robodogCamCvPtr->image.clone();
			cv::Mat glassesCamCvGray, robodogCamCvGray;

			// convert to grayscale because I like a gray world
			cv::cvtColor(glassesCamCvPtr->image, glassesCamCvGray, CV_BGR2GRAY);
			cv::cvtColor(robodogCamCvPtr->image, robodogCamCvGray, CV_BGR2GRAY);
			// use fast score if settings.cpp says so
			cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(400, 4, 3, false);
			cv::Ptr<cv::DescriptorMatcher>
				matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
			cv::Mat mask;
			std::vector<std::vector<cv::DMatch>> matches;

			//	calculate descriptors
			std::vector<cv::KeyPoint> tmplPts, imgPts;
			cv::Mat tmplDescriptor, imgDescriptor;
			std::vector<cv::Point2f> resultsImg, resultsTmpl;

			detector->detectAndCompute(glassesCamCvGray, mask, tmplPts, tmplDescriptor);
			detector->detectAndCompute(robodogCamCvGray, mask, imgPts, imgDescriptor);

			matcher->knnMatch(tmplDescriptor, imgDescriptor, matches, 2);

			// sort matches acording to distance
			// std::sort(matches.begin(), matches.end(), compare_matches);
			//-- Filter matches using the Lowe's ratio test
			const float ratio_thresh = 0.3f;
			uint featureCounter = 0;
			for (size_t i = 0; i < matches.size(); i++)
			{
				if (matches[i][0].distance < ratio_thresh * matches[i][1].distance)
				{
					// get feature points of match
					cv::Point2f resImg, resTmpl;
					resImg = imgPts.at(matches[i][0].trainIdx).pt;
					resTmpl = tmplPts.at(matches[i][0].queryIdx).pt;
					resultsImg.push_back(resImg);
					resultsTmpl.push_back(resTmpl);
					featureCounter++;
				}
			}

			if (resultsTmpl.size() >= 4 && resultsImg.size() >= 4)
			{
				// // remove outliers with RANSAC
				cv::Mat H = cv::findHomography(resultsTmpl, resultsImg, cv::RANSAC);
				// check if findhomography suceeded
				if (!H.empty())
				{
					robodogGazeCv = transform_gaze(H, glassesGazeCv);
					publish_gaze(robodogGazeCv, robodogGazePub);
				}
			}
		}
	}

	cv::Point2f transform_gaze(cv::Mat &H,
							   cv::Point2f &gaze)
	{
		std::vector<cv::Point2f> transformedPoints;
		std::vector<cv::Point2f> gazeVec = {gaze};
		cv::perspectiveTransform(gazeVec, transformedPoints, H);
		return transformedPoints.front();
	}

	void publish_gaze(cv::Point2f &gaze,
					  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr &pub)
	{
		geometry_msgs::msg::PointStamped gazeMsg;
		gazeMsg.header.stamp = this->get_clock()->now();
		gazeMsg.point.x = gaze.x;
		gazeMsg.point.y = gaze.y;
		pub->publish(gazeMsg);
	}

	//  __     __         _       _     _
	//  \ \   / /_ _ _ __(_) __ _| |__ | | ___  ___
	//   \ \ / / _` | '__| |/ _` | '_ \| |/ _ \/ __|
	//    \ V / (_| | |  | | (_| | |_) | |  __/\__ \
	//     \_/ \__,_|_|  |_|\__,_|_.__/|_|\___||___/
	// subscriber(s)
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr glassesGazeSub;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr glassesCamSub;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr robodogCamSub;
	// publisher(s)
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr robodogGazePub;
	// timer for the publishing cycle
	rclcpp::TimerBase::SharedPtr robodogGazeTimer;
	std::chrono::duration<float> robodogGazePubInterval = std::chrono::milliseconds(200ms);
	// node that streams the external device data
	std::string externalDevice = "smart_glasses/";
	std::string internalDevice = "robodog_camera/color/";
	// buffers for incoming data
	std::queue<geometry_msgs::msg::PointStamped> glassesGazeBuf;
	std::queue<sensor_msgs::msg::Image::ConstSharedPtr> glassesCamBuf;
	std::queue<sensor_msgs::msg::Image::ConstSharedPtr> robodogCamBuf;
	const uint maxQueueSize = 1;
	// opencv data objects
	cv_bridge::CvImageConstPtr glassesCamCvPtr;
	cv_bridge::CvImageConstPtr robodogCamCvPtr;
	cv::Point2f glassesGazeCv;
	cv::Point2f robodogGazeCv;
	uint maxNumFeatures = 400;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GazeTransform>());
	rclcpp::shutdown();
	return 0;
}
