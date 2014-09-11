/**
 * @brief	This file contains the QtROS class which contains methods
 *          for communicating with other ROS nodes
 *
 * @file 	qtros.cpp
 * @author	Lennart Claassen
 * @date    Sep 08 2014
 */
#include <kinpro/qtros.hpp>

#include <boost/foreach.hpp>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

QtROS::QtROS(int argc, char *argv[], const char* node_name) {
    ros::init(argc, argv, node_name);

    this->nh        = new ros::NodeHandle;

    sub = nh->subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &QtROS::callback, this);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    start();
}


QtROS::~QtROS() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}


void QtROS::run() {
    ros::Rate rate(50);

    while(ros::ok()){

        ros::spinOnce();
        rate.sleep();
    }
    emit rosShutdown();
}

void QtROS::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
//    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    pcl::fromROSMsg(*msg, pclCloud);
//    printf ("PCL cloud generated: width = %d, height = %d\n", pclCloud.width, pclCloud.height);
//    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//      printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    emit pointCloudReceived(pclCloud);
}
