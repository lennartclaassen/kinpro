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

/**
 * @brief QtROS::QtROS      Constructor
 * @param argc
 * @param argv
 * @param node_name
 */
QtROS::QtROS(int argc, char *argv[], const char* node_name) {

    //init ROS
    ros::init(argc, argv, node_name);

    //create ROS nodehandle and image transport
    this->nh        = new ros::NodeHandle;
    this->it        = new image_transport::ImageTransport(*nh);

    //create the subscribers
    pc_sub = nh->subscribe< pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, &QtROS::pointcloudCallback, this);
//    pos_sub = nh->subscribe< nav_msgs::Odometry >("/kinect_odometer/odometry", 1, &QtROS::positionCallback, this);
    pos_sub = nh->subscribe< nav_msgs::Odometry >("/odometry/filtered", 1, &QtROS::poseCallback, this);
    line_sub = nh->subscribe< kinpro_interaction::line >("/line", 1, &QtROS::lineCallback, this);
//    ar_sub = nh->subscribe< geometry_msgs::TransformStamped >("/ar_single_board/transform", 1, &QtROS::arCallback, this);
    ar_sub = nh->subscribe< geometry_msgs::TransformStamped >("/ar_multi_boards/transform", 2, &QtROS::arCallback, this);
    posRMS_sub = nh->subscribe< std_msgs::Float32 >("/humanoid_localization/best_particle_rms", 1, &QtROS::poseRMSCallback, this);
    cam2proj_sub = nh->subscribe< std_msgs::Float32MultiArray >("/projector_transform/transform", 1, &QtROS::cam2projCallback, this);

    //create the pointcloud publisher
    pc_pub = nh->advertise< pcl::PointCloud<pcl::PointXYZRGB> >("cloud_in", 1);
    //create the initial pose publisher
    initPosePub = nh->advertise< geometry_msgs::PoseWithCovarianceStamped >("initialpose", 1);

    //create the service clients for global and local localization
    globalLocClient = nh->serviceClient< std_srvs::Empty >("/global_localization");
    localLocClient = nh->serviceClient< std_srvs::Empty >("/local_localization");
    pauseLocClient = nh->serviceClient< std_srvs::Empty >("/humanoid_localization/pause_localization_srv");
    resumeLocClient = nh->serviceClient< std_srvs::Empty >("/humanoid_localization/resume_localization_srv");
    pauseVisOdomClient = nh->serviceClient < std_srvs::Empty >("/pause_fovis");
    resumeVisOdomClient = nh->serviceClient < std_srvs::Empty >("/resume_fovis");

    posSigCnt = 0;
    posSigCntVal = 2;

    //create the image publisher for the visualization
    image_publisher = it->advertise("/raspberry_image", 1);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    start();
}

/**
 * @brief QtROS::~QtROS
 */
QtROS::~QtROS() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

/**
 * @brief QtROS::run    Main loop
 */
void QtROS::run() {
    ros::Rate rate(10);

    while(ros::ok()){

        ros::spinOnce();
        rate.sleep();
    }
    emit rosShutdown();
}

/**
 * @brief QtROS::pointcloudCallback     callback for the pointcloud message
 * @param msg
 */
void QtROS::pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud = msg->makeShared();
    emit pointCloudReceived(*pclCloud);
}

/**
 * @brief QtROS::poseCallback callback for the pose message
 * @param msg
 */
void QtROS::poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    posSigCnt++;
    if(posSigCnt > posSigCntVal) {
        posSigCnt = 0;
        emit poseReceived(*msg);
    }
}

/**
 * @brief QtROS::cam2projCallback callback for the transformation between camera and projector
 * @param msg
 */
void QtROS::cam2projCallback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
    emit cam2projTrafoReceived(*msg);
}

/**
 * @brief QtROS::arCallback callback for the transformation of the augmented reality marker
 * @param msg
 */
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
//        cout << "transforms empty, adding message" << endl;
        this->arTransforms.push_back(*msg);
    }
}

/**
 * @brief QtROS::poseRMSCallback    callback for the root mean square error of the global localization
 * @param rmsMsg
 */
void QtROS::poseRMSCallback(const std_msgs::Float32::ConstPtr &rmsMsg) {
    if(currRMSVal != rmsMsg->data){
        currRMSVal = rmsMsg->data;
        emit signalPoseRMS(currRMSVal);
    }
}

/**
 * @brief QtROS::publishImage       publish the visualization image
 */
void QtROS::publishImage() {
    image_publisher.publish(this->projectorImg);
}

/**
 * @brief QtROS::slotProjectImage SLOT to be called to project an image
 * @param img
 */
void QtROS::slotProjectImage(cv::Mat img) {

    //create the projection image
    this->projectorImg.step = this->projectorImg.width = img.cols;
    this->projectorImg.height = img.rows;
    uchar* ptr = img.ptr<uchar>(0);
    this->projectorImg.data.assign(ptr, ptr + img.rows*img.cols*img.channels());
    this->projectorImg.encoding = sensor_msgs::image_encodings::RGB8;
    this->projectorImg.is_bigendian = false;
    this->projectorImg.step = img.cols * img.channels();

    this->publishImage();
}

/**
 * @brief QtROS::slotPublishPointcloud      publish a pointcloud
 * @param pc
 */
void QtROS::slotPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB> pc) {
    pc_pub.publish(pc);
}

/**
 * @brief QtROS::lineCallback       publish the line
 * @param line
 */
void QtROS::lineCallback(const kinpro_interaction::lineConstPtr &line) {
    emit lineReceived(*line);
}

/**
 * @brief QtROS::slotPublishInitialPose     publish a pose to (re-)initialize global localization
 * @param pose
 */
void QtROS::slotPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped pose) {
    initPosePub.publish(pose);
}

/**
 * @brief QtROS::slotCallGlobalLoc      SLOT to be called to start global localization
 */
void QtROS::slotCallGlobalLoc() {
    globalLocClient.call(m_e);
}

/**
 * @brief QtROS::slotCallLocalLoc   SLOT to be called to start local localization
 */
void QtROS::slotCallLocalLoc() {
    localLocClient.call(m_e);
}

/**
 * @brief QtROS::slotCallPauseLoc   SLOT to be called to pause global localization
 */
void QtROS::slotCallPauseLoc() {
    pauseLocClient.call(m_e);
}

/**
 * @brief QtROS::slotCallResumeLoc   SLOT to be called to resume global localization
 */
void QtROS::slotCallResumeLoc() {
    resumeLocClient.call(m_e);
}

/**
 * @brief QtROS::slotPauseVisOdom   SLOT to be called to pause local localization
 */
void QtROS::slotPauseVisOdom() {
    pauseVisOdomClient.call(m_e);
}

/**
 * @brief QtROS::slotResumeVisOdom   SLOT to be called to resume local localization
 */
void QtROS::slotResumeVisOdom() {
    resumeVisOdomClient.call(m_e);
}

/**
 * @brief QtROS::slotGetARTransform   SLOT to be called to get the transformation of the augmented reality marker
 */
void QtROS::slotGetARTransform() {
    if(!arTransforms.empty()) {
//        cout << "sending transforms" << endl;
        emit signalSendARTransform(arTransforms);
    }
}

/**
 * @brief QtROS::slotToggleARDet    SLOT to be called to toggle the detection of the augmented reality marker
 */
void QtROS::slotToggleARDet() {
    //TODO

//    std_srvs::Empty e;
//    arClient.call(e);
}
