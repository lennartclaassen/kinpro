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

    pc_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, &QtROS::pointcloudCallback, this);
    pos_sub = nh->subscribe< nav_msgs::Odometry >("/odometry/filtered", 1, &QtROS::positionCallback, this);
    line_sub = nh->subscribe< kinpro_interaction::line >("/line", 1, &QtROS::lineCallback, this);

//    octomapClient = nh->serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server_node/clear_bbx");

    pc_pub = nh->advertise< pcl::PointCloud<pcl::PointXYZRGB> >("cloud_in", 1);


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
    ros::Rate rate(30);

    while(ros::ok()){

        ros::spinOnce();
        rate.sleep();
    }
    emit rosShutdown();
}

void QtROS::pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud = msg->makeShared();
    emit pointCloudReceived(*pclCloud);
}

void QtROS::positionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    emit positionReceived(*msg);
}

void QtROS::publishImage() {
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

void QtROS::slotPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB> pc) {
    pc_pub.publish(pc);
}

void QtROS::lineCallback(const kinpro_interaction::lineConstPtr &line) {
    emit lineReceived(*line);
}
