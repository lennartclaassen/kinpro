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

#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballActor.h>

// Constructor
EventQtSlotConnect::EventQtSlotConnect()
{
  this->setupUi(this);

  this->Connections = vtkSmartPointer<vtkEventQtSlotConnect>::New();

  // Sphere
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> sphereActor =
    vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(sphereMapper);

  // VTK Renderer
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(sphereActor);

  this->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  this->Connections->Connect(this->qvtkWidget->GetRenderWindow()->GetInteractor(),
  vtkCommand::LeftButtonPressEvent,
  this,
  SLOT(slot_clicked(vtkObject*, unsigned long, void*, void*)));

};

void EventQtSlotConnect::slot_clicked(vtkObject*, unsigned long, void*, void*)
{
  std::cout << "Clicked." << std::endl;
}

/* METHODS */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

//    ui = new Ui::MainWindow();
//    ui->setupUi(this);

//    QObject::connect(ui->openButton, SIGNAL(clicked()), this, SLOT(onOpen()));
//    QObject::connect(ui->filterButton, SIGNAL(clicked()), this, SLOT(onFilter()));
//    QObject::connect(ui->saveButton, SIGNAL(clicked()), this, SLOT(onSave()));

//    Instantiate( sphereSource, vtkSphereSource);
//    Instantiate( glyph, vtkGlyph3D );
//    Instantiate( glyphMapper, vtkPolyDataMapper);
//    Instantiate( glyphActor, vtkActor);
//    Instantiate( sphereMapper, vtkPolyDataMapper);
//    Instantiate( sphereActor, vtkActor);
//    Instantiate( sliderWidget, vtkSliderWidget);
//    Instantiate( sliderRepresentation, vtkSliderRepresentation2D);
//    Instantiate( callback, vtkSliderCallback);
//    Instantiate( renderer, vtkRenderer);
//    Instantiate( renderWindow, vtkRenderWindow);
//    Instantiate( interactor, vtkRenderWindowInteractor);

//    sphereSource->SetRadius( 5 );
//    sphereSource->SetPhiResolution( 36 );
//    sphereSource->SetThetaResolution( 36 );

//    glyph->SetInputConnection( sphereSource->GetOutputPort() );
//    glyph->OrientOn();
//    glyph->SetVectorModeToUseNormal();

//    glyphMapper->SetInputConnection( glyph->GetOutputPort() );

//    glyphActor->SetMapper( glyphMapper );

//    sphereMapper->SetInputConnection( sphereSource->GetOutputPort() );

//    sphereActor->SetMapper( sphereMapper );

//    renderer->AddViewProp( sphereActor );
//    renderer->AddViewProp( glyphActor );

//    ui->qvtkWidget->GetRenderWindow()->AddRenderer( renderer );

//    renderWindow->AddRenderer( renderer );

//    renderWindow->SetInteractor( interactor );

//    sliderRepresentation->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay();
//    sliderRepresentation->GetPoint1Coordinate()->SetValue(.1, .1);
//    sliderRepresentation->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
//    sliderRepresentation->GetPoint2Coordinate()->SetValue(.3, .1);
//    sliderRepresentation->SetValue(1.0);

//    sliderWidget->SetInteractor( interactor );
//    sliderWidget->SetRepresentation( sliderRepresentation );
//    sliderWidget->EnabledOn();

//    callback->glyph = glyph;
//    sliderWidget->AddObserver( vtkCommand::InteractionEvent, callback );

//    vtkInteractorStyleSwitch* style;
//    style = vtkInteractorStyleSwitch::SafeDownCast(interactor->GetInteractorStyle());
//    style->SetCurrentStyleToTrackballCamera();


//    interactor->Initialize();
//    interactor->Start();

}


MainWindow::~MainWindow() {

}

void MainWindow::onOpen() {

}

void MainWindow::onFilter() {

}

void MainWindow::onSave() {

}
