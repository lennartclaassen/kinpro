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
 * @file   tfProcessing.hpp
 * @author Lennart Claassen
 * @date   Dec 08 2014
 *
 * @brief  This file contains a class for the kinect projector tf processor
 */

#ifndef _TF_PROCESSING_H
#define _TF_PROCESSING_H

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


class TFProcessor: public QObject
{
    Q_OBJECT

public:
    TFProcessor();

public slots:
    void newPositionReceived(nav_msgs::Odometry msg);

signals:
    void signalNewTFTransform(tf::StampedTransform transform);


private:

    std::string targetFrame, sourceFrame;

    tf::TransformListener ls;
    tf::StampedTransform transform;
};

#endif
