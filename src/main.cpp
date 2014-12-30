/**
 * @brief	graphical user interface for kinect-projector application
 *
 * @file 	main.cpp
 * @author	Lennart Claassen
 * @date	Sep 08 2014
 */
#include <kinpro/mainwindow.hpp>
#include <kinpro/transformationProcessing.hpp>
#include <kinpro/tfProcessing.hpp>
#include <QtGui>
#include <QApplication>
#include <stdlib.h>
#include <string.h>

/**
 * @brief   Main program
 * @return  run status
 */
int main(int argc, char **argv) {

    QApplication app(argc, argv);
    qRegisterMetaType< cv::Mat >("cv::Mat");
    qRegisterMetaType< pcl::PointCloud<pcl::PointXYZRGB> >("pcl::PointCloud<pcl::PointXYZRGB>");
    qRegisterMetaType< nav_msgs::Odometry >("nav_msgs::Odometry");
    qRegisterMetaType< kinpro_interaction::line >("kinpro_interaction::line");
    qRegisterMetaType< geometry_msgs::PoseWithCovarianceStamped >("geometry_msgs::PoseWithCovarianceStamped");
    qRegisterMetaType< tf::StampedTransform >("tf::StampedTransform");
    qRegisterMetaType< Ui::MainWindow >("Ui::MainWindow&");
    qRegisterMetaType< Eigen::Matrix3f >("Eigen::Matrix3f");
    qRegisterMetaType< Eigen::Matrix4f >("Eigen::Matrix4f");
    qRegisterMetaType< std::vector<geometry_msgs::TransformStamped> >("std::vector<geometry_msgs::TransformStamped>");



    // create the ROS Node
    QtROS qtRos(argc, argv, "kinpro_node");

    QThread transformThread, tfThread;
    TFProcessor *tfProc = new TFProcessor;
    tfProc->moveToThread(&tfThread);
    TransformationProcessor *transformProc = new TransformationProcessor;
    transformProc->moveToThread(&transformThread);

    // create a GUI
    MainWindow gui;
    gui.show();

    // signals and slots
    app.connect(&gui,   SIGNAL(signalProjectImage(cv::Mat)),                                        &qtRos, SLOT(slotProjectImage(cv::Mat)) );
    app.connect(&gui,   SIGNAL(signalPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB>)),         &qtRos, SLOT(slotPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB>)) );
    app.connect(&qtRos, SIGNAL(pointCloudReceived(pcl::PointCloud<pcl::PointXYZRGB>)),              &gui,   SLOT(newPointCloud(pcl::PointCloud<pcl::PointXYZRGB>)) );
//    app.connect(&qtRos, SIGNAL(positionReceived(nav_msgs::Odometry)),                               &gui,   SLOT(newPosition(nav_msgs::Odometry)) );
    app.connect(&qtRos, SIGNAL(lineReceived(kinpro_interaction::line)),                             &gui,   SLOT(newLine(kinpro_interaction::line)) );
    app.connect(&gui,   SIGNAL(signalPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped)), &qtRos, SLOT(slotPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped)) );
    app.connect(&gui,   SIGNAL(signalCallGlobalLoc()),                                              &qtRos, SLOT(slotCallGlobalLoc()) );
    app.connect(&gui,   SIGNAL(signalCallLocalLoc()),                                               &qtRos, SLOT(slotCallLocalLoc()) );
    app.connect(&gui,   SIGNAL(signalCallPauseLoc()),                                               &qtRos, SLOT(slotCallPauseLoc()) );
    app.connect(&gui,   SIGNAL(signalCallResumeLoc()),                                              &qtRos, SLOT(slotCallResumeLoc()) );
    app.connect(&gui,   SIGNAL(signalToggleVisOdom()),                                              &qtRos, SLOT(slotToggleVisOdom()) );
    app.connect(&gui,   SIGNAL(signalGetARTransform()),                                             &qtRos, SLOT(slotGetARTransform()));
    app.connect(&qtRos, SIGNAL(signalSendARTransform(std::vector<geometry_msgs::TransformStamped>)),             &gui,   SLOT(newARTransform(std::vector<geometry_msgs::TransformStamped>)));

    app.connect(&gui,   SIGNAL(setTransformations(Ui::MainWindow&)),                                transformProc, SLOT(setTransformations(Ui::MainWindow&)) );
    app.connect(gui.timer,   SIGNAL(timeout()),                                                    &gui, SLOT(timerCallback()) );


    app.connect(&qtRos,         SIGNAL(positionReceived(nav_msgs::Odometry)),           tfProc,         SLOT(newPositionReceived(nav_msgs::Odometry)) );
    app.connect(tfProc,         SIGNAL(signalNewTFTransform(tf::StampedTransform)),     transformProc,  SLOT(newTFTransform(tf::StampedTransform)) );

    app.connect(&qtRos,         SIGNAL(positionReceived(nav_msgs::Odometry)),           transformProc,  SLOT(newPositionReceived(nav_msgs::Odometry)) );
    app.connect(transformProc,  SIGNAL(transformationProcessingReady()),                &gui,           SLOT(transformationProcessingReady()));
    app.connect(transformProc,  SIGNAL(transformDone()),                                &gui,           SLOT(newTransform()) );

    app.connect(transformProc,  SIGNAL(newCam2ProjTransform(Eigen::Matrix4f)),          &gui,   SLOT(newCam2ProjTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newCam2ProjVTKTransform(Eigen::Matrix4f)),       &gui,   SLOT(newCam2ProjVTKTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newCamlink2CamVTKTransform(Eigen::Matrix4f)),    &gui,   SLOT(newCamlink2CamVTKTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newIntrProjTransform(Eigen::Matrix3f)),          &gui,   SLOT(newIntrProjTransform(Eigen::Matrix3f)) );
    app.connect(transformProc,  SIGNAL(newIntrProjVTKTransform(Eigen::Matrix3f)),       &gui,   SLOT(newIntrProjVTKTransform(Eigen::Matrix3f)) );
    app.connect(transformProc,  SIGNAL(newMap2WorldVTKTransform(Eigen::Matrix4f)),      &gui,   SLOT(newMap2WorldVTKTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newVTKCamTransform(Eigen::Matrix4f)),            &gui,   SLOT(newVTKCamTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newWorld2CamlinkVTKTransform(Eigen::Matrix4f)),  &gui,   SLOT(newWorld2CamlinkVTKTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newWorld2CamVTKTransform(Eigen::Matrix4f)),      &gui,   SLOT(newWorld2CamVTKTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newWorld2ProjTransform(Eigen::Matrix4f)),        &gui,   SLOT(newWorld2ProjTransform(Eigen::Matrix4f)) );
    app.connect(transformProc,  SIGNAL(newWorld2ProjVTKTransform(Eigen::Matrix4f)),     &gui,   SLOT(newWorld2ProjVTKTransform(Eigen::Matrix4f)) );

//    app.connect(transformProc,  SIGNAL(destroyed()),        transformProc,  SLOT(deleteLater()));
//    app.connect(tfProc,         SIGNAL(destroyed()),        tfProc,  SLOT(deleteLater()));

    transformThread.start();
    tfThread.start();
    int result = app.exec();

    return result;
}
