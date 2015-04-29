/**
 * @brief	This file contains the QtROS class which contains methods
 *          for communicating with other ROS nodes
 *
 * @file 	qtros.hpp
 * @author	Lennart Claassen
 * @date    Sep 08 2014
 */
#ifndef QT_ROS_H
#define QT_ROS_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <QThread>
#include <QObject>
#include <QStringListModel>
#include <QMutex>

#include <ros/network.h>
#include <ros/package.h>

#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <sstream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>

//#include <kinpro_interaction/line.h>

#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

class QtROS: public QThread {
    Q_OBJECT

    signals:

        /**
         * @brief this signal is emitted when the ROS main loop is left
         */
        void rosShutdown();

        void pointCloudReceived(pcl::PointCloud<pcl::PointXYZRGB> pc);
        void poseReceived(nav_msgs::Odometry msg);
        void cam2projTrafoReceived(std_msgs::Float32MultiArray msg);
        //void lineReceived(kinpro_interaction::line line);
        void signalSendARTransform(std::vector<geometry_msgs::TransformStamped> transforms);
        void signalPoseRMS(float rmsVal);

    public slots:

        void slotProjectImage(cv::Mat img);
        void slotPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB> pc);
        void slotPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped pose);
        void slotCallGlobalLoc();
        void slotCallLocalLoc();
        void slotCallPauseLoc();
        void slotCallResumeLoc();
        void slotPauseVisOdom();
        void slotResumeVisOdom();

        void slotGetARTransform();
        void slotToggleARDet();

    public:
        /**
         * @brief constructor of class QtROS
         */
        QtROS(int argc, char *argv[], const char* node_name);

        /**
         * @brief destructor of class QtROS
         */
        ~QtROS();

        /**
         * @brief This method returns the current NodeHandle
         * @return NodeHandle
         */
        ros::NodeHandle getNodeHandle() {
            return *nh;
        }

        /**
         * @brief This method contains the ROS event loop
         */
        void run();

        void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);
        void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void arCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
        void poseRMSCallback(const std_msgs::Float32::ConstPtr& rmsMsg);
        void cam2projCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);


    private:

        ros::NodeHandle*                    nh;
        image_transport::ImageTransport*    it;
        image_transport::Publisher          image_publisher;
        ros::Subscriber                     pc_sub;
        ros::Subscriber                     pos_sub;
        ros::Subscriber                     cam2proj_sub;
        ros::Subscriber                     ar_sub;
        ros::Subscriber                     posRMS_sub;
        ros::Publisher                      pc_pub;
        ros::ServiceClient                  octomapClient;
        ros::ServiceClient                  globalLocClient;
        ros::ServiceClient                  localLocClient;
        ros::ServiceClient                  pauseLocClient;
        ros::ServiceClient                  resumeLocClient;
        ros::Publisher                      initPosePub;
        ros::ServiceClient                  pauseVisOdomClient;
        ros::ServiceClient                  resumeVisOdomClient;
        std_srvs::Empty                     m_e;

        pcl::PointCloud<pcl::PointXYZRGB>   pclCloud;
        sensor_msgs::Image                  projectorImg;
        std::vector<geometry_msgs::TransformStamped>     arTransforms;

        int posSigCnt;
        int posSigCntVal;
        float currRMSVal;

        void publishImage();
};
#endif
