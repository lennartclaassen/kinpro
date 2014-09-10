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
    this->resize(500, 500);


}

VTKPointCloudWidget::~VTKPointCloudWidget() {

}

void VTKPointCloudWidget::addPointCloud(PointCloud<PointXYZ>::Ptr pc)
{
    this->vis->addPointCloud(pc);

    vtkSmartPointer<vtkRenderWindow> renderWindow = vis->getRenderWindow();
    this->SetRenderWindow(renderWindow);
//    this->show();
}

void VTKPointCloudWidget::showPointCloud()
{
    PointCloud<PointXYZ>::Ptr pc (new PointCloud<PointXYZ>);

    for(int i = 0; i < 10 ; i++) {
        for(int j = 0; j < 10 ; j++){
            pc->push_back( PointXYZ( (float)i, (float)j, 1.0 ) );
        }
    }

    this->vis->updatePointCloud<PointXYZ>(pc);

    this->update();
}

/* METHODS */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

    ui = new Ui::MainWindow();
    ui->setupUi(this);

    pclWidget = new VTKPointCloudWidget();

    PointCloud<PointXYZ>::Ptr pc (new PointCloud<PointXYZ>);

    pclWidget->addPointCloud(pc);
    pclWidget->showPointCloud();

    Instantiate( sphereSource, vtkSphereSource);

    Instantiate( sphereMapper, vtkPolyDataMapper);
    Instantiate( sphereActor, vtkActor);

    Instantiate( renderer, vtkRenderer);
    Instantiate( renderWindow, vtkRenderWindow);
    Instantiate( interactor, vtkRenderWindowInteractor);
    sphereSource->SetRadius( 5 );
    sphereSource->SetPhiResolution( 36 );
    sphereSource->SetThetaResolution( 36 );

    sphereMapper->SetInputConnection( sphereSource->GetOutputPort() );
    sphereActor->SetMapper( sphereMapper );
    renderer->AddViewProp( sphereActor );
    ui->qvtkWidget->GetRenderWindow()->AddRenderer( renderer );

    renderWindow = pclWidget->vis->getRenderWindow();
    ui->qvtkWidget->SetRenderWindow (renderWindow);     //Only shows pointCloud data not previously added renderers

    ui->qvtkWidget->show();
}


MainWindow::~MainWindow() {

}
