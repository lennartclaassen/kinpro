/**
 * @brief	graphical user interface for kinect-projector application
 *
 * @file 	main.cpp
 * @author	Lennart Claassen
 * @date	Sep 08 2014
 */
#include <kinpro/mainwindow.hpp>
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


    // create the ROS Node
    QtROS qtRos(argc, argv, "kinpro_node");

    QThread transformThread;
    PositionTransformer *transformer = new PositionTransformer;

    transformer->moveToThread(&transformThread);


    // create a GUI
    MainWindow gui;
    gui.show();

    // signals and slots
    app.connect(&gui,   SIGNAL(signalProjectImage(cv::Mat)),                                        &qtRos, SLOT(slotProjectImage(cv::Mat))                                         );
    app.connect(&gui,   SIGNAL(signalPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB>)),         &qtRos, SLOT(slotPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB>))          );
    app.connect(&qtRos, SIGNAL(pointCloudReceived(pcl::PointCloud<pcl::PointXYZRGB>)),              &gui,   SLOT(newPointCloud(pcl::PointCloud<pcl::PointXYZRGB>))                  );
//    app.connect(&qtRos, SIGNAL(positionReceived(nav_msgs::Odometry)),                               &gui,   SLOT(newPosition(nav_msgs::Odometry))                                   );
    app.connect(&qtRos, SIGNAL(lineReceived(kinpro_interaction::line)),                             &gui,   SLOT(newLine(kinpro_interaction::line))                                 );
    app.connect(&gui,   SIGNAL(signalPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped)), &qtRos, SLOT(slotPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped))  );
    app.connect(&gui,   SIGNAL(signalCallGlobalLoc()),                                              &qtRos, SLOT(slotCallGlobalLoc())                                               );
    app.connect(&gui,   SIGNAL(signalCallLocalLoc()),                                               &qtRos, SLOT(slotCallLocalLoc())                                                );
    app.connect(&gui,   SIGNAL(signalCallPauseLoc()),                                               &qtRos, SLOT(slotCallPauseLoc())                                                );
    app.connect(&gui,   SIGNAL(signalCallResumeLoc()),                                              &qtRos, SLOT(slotCallResumeLoc())                                               );
    app.connect(&gui,   SIGNAL(signalToggleVisOdom()),                                              &qtRos, SLOT(slotToggleVisOdom())                                               );

    app.connect(&qtRos, SIGNAL(positionReceived(nav_msgs::Odometry)),                               transformer,   SLOT(newPositionReceived(nav_msgs::Odometry))                           );
    app.connect(transformer,   SIGNAL(transformDone(tf::StampedTransform)),                             &gui,   SLOT(newTransform(tf::StampedTransform))                                               );

    transformThread.start();
    int result = app.exec();

    return result;
}
