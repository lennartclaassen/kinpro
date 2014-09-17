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

QtROS::QtROS(int argc, char *argv[], const char* node_name) {
    ros::init(argc, argv, node_name);

    this->nh        = new ros::NodeHandle;
    this->it        = new image_transport::ImageTransport(*nh);

//    sub = nh->subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, &QtROS::callback, this);
    sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, &QtROS::callback, this);

    image_publisher = it->advertise("/raspberry_image", 1);

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

void QtROS::callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud = msg->makeShared();
//    Eigen::Vector3i vec = msg->at(0).getRGBVector3i();
//    printf ("r= %d, g= %d, b= %d\n", vec[0], vec[1], vec[2]);
//    pcl::fromROSMsg(*msg, pclCloud);
//    printf ("PCL cloud generated: width = %d, height = %d\n", pclCloud.width, pclCloud.height);
//    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//      printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    emit pointCloudReceived(*pclCloud);
}

void QtROS::publishImage() {

    // === PUBLISH ===
    image_publisher.publish(this->projectorImg);
}

void QtROS::slotProjectImage(cv::Mat img) {

    this->projectorImg.step = this->projectorImg.width = img.cols;
    this->projectorImg.height = img.rows;
    uchar* ptr = img.ptr<uchar>(0);
    this->projectorImg.data.assign(ptr, ptr + img.rows*img.cols*img.channels());
    this->projectorImg.encoding = sensor_msgs::image_encodings::RGB8;
    this->projectorImg.is_bigendian = false;
    this->projectorImg.step = img.cols * img.channels();

    this->publishImage();
}
