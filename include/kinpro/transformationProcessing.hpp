/* *****************************************************************
 *
 * Copyright (c) 2014,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/
/**
 * @file   transformationProcessing.hpp
 * @author Lennart Claassen
 * @date   Dec 08 2014
 *
 * @brief  This file contains a class for the kinect projector transformation processor
 */

#ifndef _TRANSFORMATION_PROCESSING_H
#define _TRANSFORMATION_PROCESSING_H

#include <kinpro/mainwindow.hpp>

// Qt
#include <QDebug>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMessageBox>
#include <QString>
#include <QtGui>


#include <stdlib.h>
#include <stdio.h>
#include <iostream>

// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Qt;


class TransformationProcessor: public QObject
{
    Q_OBJECT

public:
//    void getTransform(tf::StampedTransform &transform);
    TransformationProcessor();

public slots:
    void newPositionReceived(nav_msgs::Odometry msg);
    void newTFTransform(tf::StampedTransform transform);
    void setTransformations(Ui::MainWindow& ui);


signals:
    void transformationProcessingReady();
    void transformDone();

    void newCam2ProjVTKTransform(Eigen::Matrix4f T);
    void newMap2WorldVTKTransform(Eigen::Matrix4f T);
    void newWorld2CamlinkVTKTransform(Eigen::Matrix4f T);
    void newCamlink2CamVTKTransform(Eigen::Matrix4f T);
    void newWorld2CamVTKTransform(Eigen::Matrix4f T);
    void newVTKCamTransform(Eigen::Matrix4f T);
    void newWorld2ProjVTKTransform(Eigen::Matrix4f T);
    void newWorld2ProjTransform(Eigen::Matrix4f T);
    void newCam2ProjTransform(Eigen::Matrix4f T);
    void newIntrProjVTKTransform(Eigen::Matrix3f T);
    void newIntrProjTransform(Eigen::Matrix3f T);


private:

    bool transformationsSet;

    //Helper Functions
    int line2int(QLineEdit& line)       { return (&line)->text().toInt(); }
    double line2double(QLineEdit& line) { return (&line)->text().toDouble(); }
    float line2float(QLineEdit& line)   { return (&line)->text().toFloat(); }

    void int2line(QLineEdit& line, int value)       { (&line)->setText(QString::number(value)); }
    void double2line(QLineEdit& line, double value) { (&line)->setText(QString::number(value)); }
    void float2line(QLineEdit& line, float value)   { (&line)->setText(QString::number(value)); }

    void setTransformationMatrix(Eigen::Matrix3f in_R, Eigen::Vector3f in_t, Eigen::Matrix4f &out_T);
    void setRotationMatrixFromYPR(float yaw, float pitch, float roll, Eigen::Matrix3f &out_R);
    void setRotationMatrixFromYPR(Eigen::Vector3f ypr, Eigen::Matrix3f &out_R);

    void setIdentityMatrix(Eigen::Matrix4f &mat);
    void setIdentityMatrix(Eigen::Matrix3f &mat);

    //transformation from camera coordinates (rgb frame) to projector coordinates
    Eigen::Matrix3f R_cam2projVTK;
    Eigen::Vector3f t_cam2projVTK;
    Eigen::Matrix4f T_cam2projVTK;

    //transformation from map coordinates to world coordinates (odom)
    Eigen::Matrix3f R_map2worldVTK;
    Eigen::Vector3f t_map2worldVTK;
    Eigen::Matrix4f T_map2worldVTK;

    //transformation from world coordinates to camera_link
    Eigen::Matrix3f R_world2camlinkVTK;
    Eigen::Vector3f t_world2camlinkVTK;
    Eigen::Matrix4f T_world2camlinkVTK;

    //transformation from camera_link to camera_rgb_optical_frame
    Eigen::Matrix3f R_camlink2camVTK;
    Eigen::Vector3f t_camlink2camVTK;
    Eigen::Matrix4f T_camlink2camVTK;

    //transformation from world coordinates to camera coordinates (rgb frame)
    //combines the tf transformations "world -> camera_link" and "camera_link -> camera_rgb_optical_frame"
    Eigen::Matrix3f R_world2camVTK;
    Eigen::Vector3f t_world2camVTK;
    Eigen::Matrix4f T_world2camVTK;

    //transformation for VTK camera
    Eigen::Matrix3f R_VTKcam;
    Eigen::Matrix4f T_VTKcam;

    //transfromation from world coordinates to projector coordinates
    Eigen::Matrix4f T_world2projVTK;
    Eigen::Matrix4f T_world2proj;

    //intrinsic projector transformation (differs from calibration values due to VTK visualization specifics)
    Eigen::Matrix3f T_intrProjVTK;

    //transformation from camera coordinates (rgb frame) to projector coordinates
    //TODO: same as T_cam2projVTK?
    Eigen::Matrix4f T_cam2proj;

    //intrinsic projector transformation from calibration
    Eigen::Matrix3f T_intrProj;
};

#endif
