#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

class ImagePublisher
{
public:
    ImagePublisher()
    {
        // Initialize ROS node handle
        ros::NodeHandle nh;
        ros::NodeHandle nh_private("~");
        
        // Load the camera parameters from YAML
        // std::string package_path = ros::package::getPath("camera_cpp");
        // std::string camera_info_file = package_path + "/config/camera_params.yaml";
        
        // nh_private.param<std::string>("camera_info_file", camera_info_file, camera_info_file);
        
        // // Parse YAML
        // YAML::Node config = YAML::LoadFile(camera_info_file);
        
        // // Fill CameraInfo message
        // camera_info_msg_.width = config["image_width"].as<int>();
        // camera_info_msg_.height = config["image_height"].as<int>();
        // camera_info_msg_.distortion_model = config["distortion_model"].as<std::string>();
        
        // // Fill K (camera_matrix)
        // for (size_t i = 0; i < config["camera_matrix"]["data"].size(); ++i)
        // {
        //     camera_info_msg_.K[i] = config["camera_matrix"]["data"][i].as<double>();
        // }
        
        // // Fill D (distortion_coefficients)
        // for (const auto& val : config["distortion_coefficients"]["data"])
        // {
        //     camera_info_msg_.D.push_back(val.as<double>());
        // }
        
        // // Fill P (projection_matrix)
        // for (size_t i = 0; i < config["projection_matrix"]["data"].size(); ++i)
        // {
        //     camera_info_msg_.P[i] = config["projection_matrix"]["data"][i].as<double>();
        // }
        
        // Publishers
        // cam_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("/go_pro/camera_info", 10);
        img_pub_ = nh.advertise<sensor_msgs::Image>("go_pro/image", 10);
        
        // OpenCV video capture
        cap_.open("/dev/video42");
        
        if (!cap_.isOpened()) {
            ROS_ERROR("Could not open video device");
            return;
        }
        
        double fps;
        nh_private.param<double>("fps", fps, 5.0);
        if (fps <= 0.0) {
            ROS_WARN("FPS must be positive. Defaulting to 10.");
            fps = 10.0;
        }
        
        ROS_INFO("Using FPS: %.2f", fps);
        timer_ = nh.createTimer(ros::Duration(1.0 / fps), &ImagePublisher::timer_callback, this);
    }

private:
    void timer_callback(const ros::TimerEvent&)
    {
        cv::Mat frame;
        if (cap_.read(frame)) {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            
            camera_info_msg_.header = header;
            
            img_pub_.publish(*msg);
            cam_info_pub_.publish(camera_info_msg_);
        } else {
            ROS_WARN("Failed to read frame from camera");
            cap_.release();
        }
    }

    ros::Publisher img_pub_;
    ros::Publisher cam_info_pub_;
    sensor_msgs::CameraInfo camera_info_msg_;
    ros::Timer timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_publisher");
    ImagePublisher image_publisher;
    ros::spin();
    return 0;
}
