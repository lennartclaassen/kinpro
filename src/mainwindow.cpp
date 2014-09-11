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
 * @file   main_window.cpp
 * @author Lennart Claassen
 * @date   Sep 08 2014
 *
 * @brief  This file contains a class for the kinect gui window
 */
#include <kinpro/mainwindow.hpp>

using namespace Qt;
using namespace std;
using namespace pcl;

VTKPointCloudWidget::VTKPointCloudWidget(QWidget *parent) : QVTKWidget(parent)
{
    vis = new visualization::PCLVisualizer("vis", false);
    vis->setBackgroundColor(0.0, 0.0, 0.2);
}

VTKPointCloudWidget::~VTKPointCloudWidget() {

}

void VTKPointCloudWidget::setRenderWindow()
{
    vtkSmartPointer<vtkRenderWindow> renderWindow = vis->getRenderWindow();
    this->SetRenderWindow(renderWindow);
}

void VTKPointCloudWidget::addPointCloud(PointCloud<PointXYZ>::Ptr pc, const string &id)
{
    this->vis->addPointCloud(pc, id);
}

void VTKPointCloudWidget::showPointCloud(PointCloud<PointXYZ>::Ptr pc, const string &id)
{
    this->vis->updatePointCloud<PointXYZ>(pc, id);
    this->update();
}

/* METHODS */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

    firstcall = true;

    ui = new Ui::MainWindow();
    ui->setupUi(this);

    pclWidget = new VTKPointCloudWidget();

    PointCloud<PointXYZ>::Ptr pc (new PointCloud<PointXYZ>);

    pclWidget->addPointCloud(pc);
    pclWidget->setRenderWindow();

    Instantiate( renderWindow, vtkRenderWindow);

    renderWindow = pclWidget->vis->getRenderWindow();
    ui->qvtkWidget->SetRenderWindow (renderWindow);     //Only shows pointCloud data not previously added renderers
}


MainWindow::~MainWindow() {

}

void MainWindow::newPointCloud(PointCloud<PointXYZ> pc) {
//    ROS_INFO("PCL cloud received");

    m_pc = pc.makeShared();
//    string id = string("new");
    pclWidget->showPointCloud(m_pc);

    ui->qvtkWidget->update();

}
