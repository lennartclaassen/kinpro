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
//    pos_sub = nh->subscribe< nav_msgs::Odometry >("/kinect_odometer/odometry", 1, &QtROS::positionCallback, this);
    pos_sub = nh->subscribe< nav_msgs::Odometry >("/odometry/filtered", 1, &QtROS::positionCallback, this);
    line_sub = nh->subscribe< kinpro_interaction::line >("/line", 1, &QtROS::lineCallback, this);
//    ar_sub = nh->subscribe< geometry_msgs::TransformStamped >("/ar_single_board/transform", 1, &QtROS::arCallback, this);
    ar_sub = nh->subscribe< geometry_msgs::TransformStamped >("/ar_multi_boards/transform", 2, &QtROS::arCallback, this);
    posRMS_sub = nh->subscribe< std_msgs::Float32 >("/humanoid_localization/best_particle_rms", 1, &QtROS::poseRMSCallback, this);

//    octomapClient = nh->serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server_node/clear_bbx");

    pc_pub = nh->advertise< pcl::PointCloud<pcl::PointXYZRGB> >("cloud_in", 1);

    globalLocClient = nh->serviceClient< std_srvs::Empty >("/global_localization");
    localLocClient = nh->serviceClient< std_srvs::Empty >("/local_localization");
    pauseLocClient = nh->serviceClient< std_srvs::Empty >("/humanoid_localization/pause_localization_srv");
    resumeLocClient = nh->serviceClient< std_srvs::Empty >("/humanoid_localization/resume_localization_srv");
    initPosePub = nh->advertise< geometry_msgs::PoseWithCovarianceStamped >("initialpose", 1);

    pauseVisOdomClient = nh->serviceClient < std_srvs::Empty >("/pause_fovis");
    resumeVisOdomClient = nh->serviceClient < std_srvs::Empty >("/resume_fovis");

    posSigCnt = 0;
    posSigCntVal = 2;

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
    ros::Rate rate(10);

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
    posSigCnt++;
    if(posSigCnt > posSigCntVal) {
        posSigCnt = 0;
        emit positionReceived(*msg);
    }
}

void QtROS::arCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    if(!this->arTransforms.empty()) {
//        cout << "transforms not empty: size = " << this->arTransforms.size() << endl;
        if((this->arTransforms.at(0).header.stamp.toNSec() == (msg->header.stamp.toNSec()))){
//            cout << "header the same" << endl;
            this->arTransforms.push_back(*msg);
        }else {
//            cout << "different headers" << endl;
            this->arTransforms.clear();
            this->arTransforms.push_back(*msg);
        }
    }else {
        cout << "transforms empty, adding message" << endl;
        this->arTransforms.push_back(*msg);
    }

//    this->arTransform = *msg->header.stamp;

}

void QtROS::poseRMSCallback(const std_msgs::Float32::ConstPtr &rmsMsg) {
    if(currRMSVal != rmsMsg->data){
        currRMSVal = rmsMsg->data;
        emit signalPoseRMS(currRMSVal);
    }
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

void QtROS::slotPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped pose) {
    initPosePub.publish(pose);
}

void QtROS::slotCallGlobalLoc() {
    globalLocClient.call(m_e);
}

void QtROS::slotCallLocalLoc() {
    localLocClient.call(m_e);
}

void QtROS::slotCallPauseLoc() {
    pauseLocClient.call(m_e);
}

void QtROS::slotCallResumeLoc() {
    resumeLocClient.call(m_e);
}

void QtROS::slotPauseVisOdom() {
    pauseVisOdomClient.call(m_e);
}

void QtROS::slotResumeVisOdom() {
    resumeVisOdomClient.call(m_e);
}

void QtROS::slotGetARTransform() {
    if(!arTransforms.empty()) {
//        cout << "sending transforms" << endl;
        emit signalSendARTransform(arTransforms);
    }
}

void QtROS::slotToggleARDet() {
//    std_srvs::Empty e;
//    arClient.call(e);
}
