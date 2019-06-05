#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <ltu_actor_route_sign_detection/StopSignConfig.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

class SignDetection
{

public:
    SignDetection();

    bool hasSub();
    bool isEnabled();
    void startup();
    void shutdown();

private:
    // Callbacks
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void configCB(ltu_actor_route_sign_detection::StopSignConfig &config, uint32_t level);

    // Run sign detection
    void detectSign(cv::Mat& frame);
    void applyRedMask(const cv::Mat &in, cv::Mat &out);

    // ROS Objects
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher sign_visible_pub_;
    ros::Publisher size_pub_;

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<ltu_actor_route_sign_detection::StopSignConfig> server_;
    ltu_actor_route_sign_detection::StopSignConfig config_;

    uint8_t sign_visible_; // uint8_t used as boolean; 1 = sign, 0 = no sign
    int sign_max_width_; // Largest sign seen in last frame
    image_transport::Publisher debug_image_pub_;
    static constexpr int consecutive_frames_trigger_ = 2;
    cv::CascadeClassifier cascade_;
    bool enabled_ = false;

    std::string cam_topic;
    std::string cascade_path;
};

SignDetection::SignDetection() : nh_("~"), it_(nh_)
{
    // Dynamic reconfigure
    server_.setCallback(boost::bind(&SignDetection::configCB, this, _1, _2));
    server_.getConfigDefault(config_);

    if (nh_.hasParam("max_size")) { nh_.getParam("max_size", config_.max_size); }
    if (nh_.hasParam("min_size")) { nh_.getParam("min_size", config_.min_size); }
    if (nh_.hasParam("scale_factor")) { nh_.getParam("scale_factor", config_.scale_factor); }
    if (nh_.hasParam("min_neighbors")) { nh_.getParam("min_neighbors", config_.min_neighbors); }
    if (nh_.hasParam("trigger_width")) { nh_.getParam("trigger_width", config_.trigger_width); }
    if (nh_.hasParam("red_mask")) { nh_.getParam("red_mask", config_.red_mask); }
    if (nh_.hasParam("hue_lower")) { nh_.getParam("hue_lower", config_.hue_lower); }
    if (nh_.hasParam("saturation_lower")) { nh_.getParam("saturation_lower", config_.saturation_lower); }
    if (nh_.hasParam("value_lower")) { nh_.getParam("value_lower", config_.value_lower); }
    if (nh_.hasParam("hue_upper")) { nh_.getParam("hue_upper", config_.hue_upper); }
    if (nh_.hasParam("saturation_upper")) { nh_.getParam("saturation_upper", config_.saturation_upper); }
    if (nh_.hasParam("value_upper")) { nh_.getParam("value_upper", config_.value_upper); }
    server_.updateConfig(config_);

    // Subscribe to camera
    if (!nh_.getParam("camera_topic", cam_topic))
    {
        ROS_ERROR_STREAM("[FATAL] Sign detection: param 'camera_topic' not defined");
        exit(0);
    }

    if (!nh_.getParam("cascade_path", cascade_path))
    {
        ROS_ERROR_STREAM("[FATAL] Sign detection: param 'camera_topic' not defined");
        exit(0);
    }

    //Load the cascades
    if (!cascade_.load(cascade_path)) {
        ROS_ERROR_STREAM("[FATAL] Sign detection: error loading cascade_ " << cascade_path);
        exit(0);
    }

    //image_sub_ = it_.subscribe(cam_topic, 1, &SignDetection::imageCb, this);

    sign_visible_pub_ = nh_.advertise<std_msgs::UInt8>("stop_sign", 100);
    size_pub_ = nh_.advertise<std_msgs::UInt32>("sign_size", 100);
    debug_image_pub_ = it_.advertise("debug", 1);

    sign_visible_ = 0;
}

void SignDetection::configCB(ltu_actor_route_sign_detection::StopSignConfig &config, uint32_t level)
{
    config_ = config;
}

void SignDetection::applyRedMask(const cv::Mat &in, cv::Mat &out)
{
    // Convert input image to HSV
    cv::Mat hsv_image;
    cv::cvtColor(in, hsv_image, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels(3);
    cv::split(in, channels);

    // Threshold the HSV image, keep only the red pixels
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image,
            cv::Scalar(config_.hue_lower, config_.saturation_lower, config_.value_lower),
            cv::Scalar(config_.hue_upper, config_.saturation_upper, config_.value_upper),
            upper_red_hue_range);

    cv::Mat dilated;
    cv::Mat dilate_element =\
                cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                cv::Size(2*10 + 1, 2*10+1),
                                cv::Point(10, 10));

    cv::dilate(upper_red_hue_range, dilated, dilate_element);

    channels[0] &= dilated;
    channels[1] &= dilated;
    channels[2] &= dilated;
    cv::merge(channels, out);
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

    sign_max_width_ = 0;

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
    if(config_.red_mask){ applyRedMask(resized, resized); }
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

bool SignDetection::hasSub(){
    return (sign_visible_pub_.getNumSubscribers() || size_pub_.getNumSubscribers() || debug_image_pub_.getNumSubscribers());
}

bool SignDetection::isEnabled(){
    return enabled_;
}

void SignDetection::startup(){
    image_sub_ = it_.subscribe(cam_topic, 1, &SignDetection::imageCb, this);
    enabled_ = true;
}

void SignDetection::shutdown(){
    image_sub_ = image_transport::Subscriber();
    enabled_ =  false;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop_sign_detection");
    SignDetection stop;
    ros::Rate r(10); 

    while (ros::ok()){
        if (stop.hasSub()){
            if (!stop.isEnabled()){
                stop.startup();
            }
        } else {
            if (stop.isEnabled()){
                stop.shutdown();
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
