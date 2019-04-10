#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <sign_detection/StopSignConfig.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

static void die(const std::string& why)
{
    ROS_ERROR_STREAM("[FATAL] Sign detection: " << why);
    exit(0);
}

class SignDetection
{

public:
    SignDetection();

private:
    // Callbacks
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void configCB(sign_detection::StopSignConfig &config, uint32_t level);

    // Run sign detection
    void detectSign(cv::Mat& frame);

    // ROS Objects
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher sign_visible_pub_;
    ros::Publisher size_pub_;

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<sign_detection::StopSignConfig> server_;
    sign_detection::StopSignConfig config_;

    uint8_t sign_visible_; // uint8_t used as boolean; 1 = sign, 0 = no sign

    int sign_max_width_; // Largest sign seen in last frame

    image_transport::Publisher debug_image_pub_;

    static constexpr int consecutive_frames_trigger_ = 2;

    cv::CascadeClassifier cascade_;
};

SignDetection::SignDetection() : nh_("~"), it_(nh_)
{
    // Subscribe to camera
    std::string cam_topic;
    if (nh_.getParam("camera_topic", cam_topic))
    {
        ROS_INFO_STREAM("Sign Detection: using video source " << cam_topic << "...");
    }
    else
    {
        die("param 'camera_topic' not defined");
    }

    std::string cascade_path;
    if (nh_.getParam("cascade_path", cascade_path))
    {
        //Load the cascades
        if (!cascade_.load(cascade_path)) {
            die(std::string("Error loading cascade_ ") + cascade_path);
        }
    }
    else
    {
        die("param 'cascade_path' not defined");
    }


    image_sub_ = it_.subscribe(cam_topic, 1, &SignDetection::imageCb, this);

    // Dynamic reconfigure
    server_.setCallback(boost::bind(&SignDetection::configCB, this, _1, _2));
    server_.getConfigDefault(config_);

    sign_visible_pub_ = nh_.advertise<std_msgs::UInt8>("stop_sign", 100);
    size_pub_ = nh_.advertise<std_msgs::UInt32>("sign_size", 100);
    debug_image_pub_ = it_.advertise("debug", 1);

    sign_visible_ = 0;
}

void SignDetection::configCB(sign_detection::StopSignConfig &config, uint32_t level)
{
    config_ = config;
}

void SignDetection::detectSign( cv::Mat& frame )
{
    std::vector<cv::Rect> signs;
    cv::Mat frame_gray;

    cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    //-- Run classifier
    cascade_.detectMultiScale(
        frame_gray,  // Input Image
        signs,  // Output bounds
        config_.scale_factor,   // Scale factor (1.05)
        config_.min_neighbors,   // Min neighbors (7)
        0|CV_HAAR_SCALE_IMAGE,  // Flags
        cv::Size(config_.min_size, config_.min_size),    // Min Size (70)
        cv::Size(config_.max_size, config_.max_size)    // Max Size (160)
    );

    int sign_max_width_ = 0;

    // Find the largest sign
    // Draw circles on all signs
    for( size_t i = 0; i < signs.size(); i++ )
    {
        // Update max width
        if(signs[i].width > sign_max_width_) sign_max_width_ = signs[i].width;

        // The center of the sign location
        cv::Point center( signs[i].x + signs[i].width*0.5,
                        signs[i].y + signs[i].height*0.5 );

        // If sign is valid (big enough), then use different color
        cv::Scalar color;
        if (signs[i].width > config_.trigger_width) {
            color = cv::Scalar(0, 250, 0);
        } else {
            color = cv::Scalar(250, 0, 0);
        }

        // Draw an ellipse on the frame where the sign is
        cv::ellipse(frame,
                    center,
                    cv::Size( signs[i].width*0.5, signs[i].height*0.5),
                    0, 0, 360, color, 4, 8, 0 );
    }

    // Count the number of valid consecutive sign frames
    //    Only publish a detection if the number of frames is
    //    above a certain threshold (frame_count_trigger_)
    static int sign_frames = 0;
    
    if (sign_max_width_ > config_.trigger_width) {
        sign_frames++;
        if (sign_frames > consecutive_frames_trigger_) {
            sign_visible_ = true;
        }
    } else {
        sign_visible_ = false;
        sign_frames = 0;
    }
}

void SignDetection::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    //Convert to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Downsize image
    cv::Mat resized;
    cv::resize(cv_ptr->image, resized, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

    // Run detection
    detectSign(resized);

    // Publish image with annotations
    if (debug_image_pub_.getNumSubscribers() > 0)
    {
        debug_image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized).toImageMsg());
    }

    // Publish sign visible?
    std_msgs::UInt8 sign_msg;
    sign_msg.data = sign_visible_;
    sign_visible_pub_.publish(sign_msg);

    // Publish size of sign
    std_msgs::UInt32 size_msg;
    size_msg.data = sign_max_width_;
    size_pub_.publish(size_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sign_detection");
    SignDetection sd;

    ros::spin();
    return 0;
}
