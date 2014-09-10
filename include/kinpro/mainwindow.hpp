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
 * @file   mainwindow.cpp
 * @author Lennart Claassen
 * @date   Sep 08 2014
 *
 * @brief  This file contains a class for the kinect projector gui window
 */
#ifndef _KINPRO_MAIN_WINDOW_H
#define _KINPRO_MAIN_WINDOW_H

#include <kinpro/qtros.hpp>
#include "../../build/kinpro/ui_mainwindow.h"

// Qt
#include <QMainWindow>
#include <QString>
#include <QtGui>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>

#include <math.h>
#include <string.h>
#include <stdio.h>

//VTK
#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkPolyDataMapper.h>
#include <vtkSliderWidget.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkImageMagnitude.h>
#include <vtkImageGradientMagnitude.h>
#include <vtkImageShiftScale.h>
#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <QVTKWidget.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>

// PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include <ros/ros.h>

// Boost
#include <boost/foreach.hpp>

class VTKPointCloudWidget: QVTKWidget
{

    public:
        /**
         * @brief MainWindow constructor
         * @param *parent pointer to parent widget
         */
        VTKPointCloudWidget(QWidget *parent = 0);

        /**
         * @brief MainWindow destructor
         */
        ~VTKPointCloudWidget();

        void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
        void showPointCloud();
        pcl::visualization::PCLVisualizer *vis;

    private:


};

/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow: public QMainWindow {
    Q_OBJECT

    public:
        /**
         * @brief MainWindow constructor
         * @param *parent pointer to parent widget
         */
        MainWindow(QWidget *parent = 0);

        /**
         * @brief MainWindow destructor
         */
        ~MainWindow();

        VTKPointCloudWidget* pclWidget;

    signals:


    private slots:

    public slots:


    private:

        Ui::MainWindow* ui;




};

#endif // _KINPRO_MAIN_WINDOW_H
