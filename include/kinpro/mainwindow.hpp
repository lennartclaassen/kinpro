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
#include <QMainWindow>
#include <QString>
#include <QtGui>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSliderWidget.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkImageMagnitude.h>
#include <vtkImageGradientMagnitude.h>
#include <vtkImageShiftScale.h>
#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
//#include <QVTKWidget.h>

#include "../../build/kinpro/ui_mainwindow.h"

#define Instantiate( obj, class ) vtkSmartPointer<class> obj = vtkSmartPointer<class>::New();
#define vsp(class,var)\
    tmpvar = vtkSmartPointer<class>::New();\
    var = tmpvar;

class vtkSliderCallback : public vtkCommand {

    public:
        static vtkSliderCallback *New() {
            return new vtkSliderCallback;
        }

        virtual void Execute(vtkObject *caller, unsigned long eventId, void *callData) {
            vtkSliderWidget *sliderWidget = reinterpret_cast<vtkSliderWidget*>(caller);
            double value = static_cast<double> (static_cast<vtkSliderRepresentation*> (sliderWidget->GetRepresentation())->GetValue());
            this->glyph->SetScaleFactor(value);
        }

        vtkGlyph3D *glyph;

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


    signals:


    private slots:

        void onOpen();
        void onFilter();
        void onSave();


    public slots:


    private:

        Ui::MainWindow* ui;

};

#endif // _KINPRO_MAIN_WINDOW_H
