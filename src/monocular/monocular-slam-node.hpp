#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"
#include "Converter.h"


class MonocularSlamNode : public rclcpp::Node
{

public:

    MonocularSlamNode(ORB_SLAM3::System* pSLAM, const string &strVocFile, const string &strSettingsFile);


    ~MonocularSlamNode();


private: 

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

};

#endif