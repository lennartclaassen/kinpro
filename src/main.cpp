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


    // create the ROS Node
    QtROS qtRos(argc, argv, "kinpro_node");

    // create a GUI
    MainWindow gui;
    gui.show();

    // signals and slots
    app.connect(&gui,   SIGNAL(signalProjectImage(cv::Mat)), &qtRos, SLOT(slotProjectImage(cv::Mat)));
    app.connect(&qtRos, SIGNAL(pointCloudReceived(pcl::PointCloud<pcl::PointXYZRGB>)), &gui, SLOT(newPointCloud(pcl::PointCloud<pcl::PointXYZRGB>)));

    //app.connect(&app,   SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    //app.connect(&qtRos, SIGNAL(rosShutdown()), &app, SLOT(quit()));
    //app.connect(&qtRos, SIGNAL(signalNewInterfaceFrame(cv::Mat)), &gui, SLOT(slotNewInterfaceFrame(cv::Mat)));
    //app.connect(&qtRos, SIGNAL(newParams(int)), &gui, SLOT(newParams(int)));
    //app.connect(&gui, SIGNAL(signalNewSelectedArea(int)), &qtRos, SLOT(slotNewSelectedArea(int)));
    //app.connect(&gui, SIGNAL(signalGetTransform(cv::Mat&, cv::Point3f&, std::string, std::string)), &qtRos, SLOT(slotGetTransform(cv::Mat&, cv::Point3f&, std::string, std::string)));
    //app.connect(&gui, SIGNAL(signalGetARMarker(std::vector<cv::Point3f>&, double&, cv::Mat&, std::string)), &qtRos, SLOT(slotGetARMarker(std::vector<cv::Point3f>&, double&, cv::Mat&, std::string)));

    int result = app.exec();

    return result;
}
