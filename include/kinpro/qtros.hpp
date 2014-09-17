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

    public slots:

        void slotProjectImage(cv::Mat img);

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

        void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg);

    private:

        ros::NodeHandle* nh;
        image_transport::ImageTransport* it;
        image_transport::Publisher image_publisher;
        ros::Subscriber sub;

        pcl::PointCloud<pcl::PointXYZRGB> pclCloud;
        sensor_msgs::Image projectorImg;

        void publishImage();
};
#endif
