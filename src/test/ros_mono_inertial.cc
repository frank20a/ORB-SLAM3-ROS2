/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
// #include <stdio.h>
#include <algorithm>
// #include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <string>
#include <thread>
#include <mutex>

#include "System.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;

class SLAMNode : public rclcpp::Node
{
public:
    SLAMNode(ORB_SLAM3::System *pSLAM, const bool bClahe) : Node("orbslam_mono_imu"), mpSLAM(pSLAM), mbClahe(bClahe)
    {
        this->create_subscription<sensor_msgs::msg::Image>(
            "front_cam/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&SLAMNode::GrabImage, this, std::placeholders::_1));

        this->create_subscription<sensor_msgs::msg::Imu>(
            "imu",
            rclcpp::SensorDataQoS(),
            std::bind(&SLAMNode::GrabImu, this, std::placeholders::_1));

        m_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "pose",
            rclcpp::SensorDataQoS());

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        ns = this->get_namespace();
        if (ns != "")
            ns += "/";
    }

    void SyncWithImu();
    void GrabImage(const sensor_msgs::msg::Image::ConstPtr msg);
    void GrabImu(const sensor_msgs::msg::Imu::ConstPtr msg);
    void HandlePose(const Sophus::SE3f &T, const builtin_interfaces::msg::Time *stamp);

    ORB_SLAM3::System *mpSLAM;

    queue<sensor_msgs::msg::Imu::ConstPtr> imuBuf;
    queue<sensor_msgs::msg::Image::ConstPtr> img0Buf;

    std::string ns;

    std::mutex mBufMutex;

    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;
        rclcpp::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, false);

    bool bEqual = false;
    if (argv[3] == "true" || argv[3] == "True")
        bEqual = true;

    auto node = std::make_shared<SLAMNode>(&SLAM, bEqual);

    std::thread sync_thread(&SLAMNode::SyncWithImu, node);
    rclcpp::spin(node);

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}

void SLAMNode::GrabImage(const sensor_msgs::msg::Image::ConstPtr msg)
{

    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(msg);
    mBufMutex.unlock();
}

void SLAMNode::SyncWithImu()
{
    while (1)
    {
        cv_bridge::CvImageConstPtr im = NULL;
        double tIm = 0;
        if (!img0Buf.empty() && !this->imuBuf.empty())
        {
            tIm = img0Buf.front()->header.stamp.sec;
            if (tIm > this->imuBuf.back()->header.stamp.sec)
                continue;
            {
                this->mBufMutex.lock();

                cv_bridge::CvImageConstPtr im;
                try
                {
                    im = cv_bridge::toCvShare(img0Buf.front(), sensor_msgs::image_encodings::RGB8);
                }
                catch (const cv_bridge::Exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                    this->mBufMutex.unlock();
                    return;
                }

                img0Buf.pop();
                this->mBufMutex.unlock();
            }

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            this->mBufMutex.lock();
            if (!this->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!this->imuBuf.empty() && this->imuBuf.front()->header.stamp.sec <= tIm)
                {
                    double t = this->imuBuf.front()->header.stamp.sec;
                    cv::Point3f acc(this->imuBuf.front()->linear_acceleration.x, this->imuBuf.front()->linear_acceleration.y, this->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(this->imuBuf.front()->angular_velocity.x, this->imuBuf.front()->angular_velocity.y, this->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    this->imuBuf.pop();
                }
            }
            this->mBufMutex.unlock();
            if (mbClahe)
                mClahe->apply(im->image, im->image);

            HandlePose(mpSLAM->TrackMonocular(im->image, tIm, vImuMeas), &im->header.stamp);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void SLAMNode::GrabImu(const sensor_msgs::msg::Imu::ConstPtr msg)
{
    mBufMutex.lock();
    imuBuf.push(msg);
    mBufMutex.unlock();
    return;
}

void SLAMNode::HandlePose(const Sophus::SE3f &T, const builtin_interfaces::msg::Time *stamp)
{
    // auto T = mpSLAM->TrackMonocular(cv_ptr->image, msg->header.stamp.sec);

    // Sophus::SE3f Twc = pKF.GetPoseInverse();
    // Eigen::Quaternionf q = Twc.unit_quaternion();
    // Eigen::Vector3f t = Twc.translation();

    Eigen::Quaternionf q(T.rotationMatrix());

    // geometry_msgs::msg::TransformStamped pose_msg;
    // pose_msg.header.stamp = msg->header.stamp;
    // pose_msg.header.frame_id = ns + "camera_optical";
    // pose_msg.child_frame_id = ns + "estimated_pose";
    // pose_msg.transform.translation.x = T.translation().x() * 10;
    // pose_msg.transform.translation.y = T.translation().y() * 10;
    // pose_msg.transform.translation.z = T.translation().z() * 10/* + 2*/;
    // pose_msg.transform.rotation.x = q.x() /*+ 0.7071788*/;
    // pose_msg.transform.rotation.y = q.y();
    // pose_msg.transform.rotation.z = q.z();
    // pose_msg.transform.rotation.w = q.w() /*+ 0.7070348*/;
    // tf_broadcaster->sendTransform(pose_msg);

    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = T.translation().x();
    pose_msg.position.y = T.translation().y();
    pose_msg.position.z = T.translation().z();
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = *stamp;
    pose_stamped_msg.header.frame_id = "world";
    pose_stamped_msg.pose = pose_msg;
    m_pose_publisher->publish(pose_stamped_msg);

    // RCLCPP_INFO(this->get_logger(), "[%.3f, %.3f, %.3f]", pose_msg.transform.translation.x, pose_msg.transform.translation.y, pose_msg.transform.translation.z);
    RCLCPP_INFO(this->get_logger(), "[%.3f, %.3f, %.3f]", pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
}