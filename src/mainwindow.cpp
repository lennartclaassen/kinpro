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
using namespace cv;

VTKPointCloudWidget::VTKPointCloudWidget(QWidget *parent) : QVTKWidget(parent)
{
    vis = new visualization::PCLVisualizer("vis", false);
//    vis->setBackgroundColor(0.7, 0.7, 0.7);
    vis->setBackgroundColor(0,0,0);
}

VTKPointCloudWidget::~VTKPointCloudWidget() {
}

/* METHODS */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

    //init UI
    ui = new Ui::MainWindow();
    ui->setupUi(this);

    //init visualization widget
    pclWidget = new VTKPointCloudWidget();
    ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());

    //create empty mat as projector image
    projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);

//    timer = new QTimer(this);
    timerRunning = false;

    //set initial bool values
    this->displayRGBCloud = true;
    this->waitForLines = false;
    this->drawClickingCircle = false;

    //transformations not ready yet
    transformReady = false;

    //setup the actors for line-object intersecting
    m_lineActor = vtkSmartPointer<vtkActor>::New();
    m_lineActor->GetProperty()->SetLineWidth(2);
    m_lineActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    obbTreeActor = vtkSmartPointer<vtkActor>::New();
    obbTreeActor->GetProperty()->SetColor(0.0, 1.0, 0.0);
    bspTreeActor = vtkSmartPointer<vtkActor>::New();
    bspTreeActor->GetProperty()->SetColor(0.0, 0.0, 1.0);
    obbTree = vtkSmartPointer<vtkOBBTree>::New();
    bspTree = vtkSmartPointer<vtkModifiedBSPTree>::New();
    pclWidget->vis->addActorToRenderer(m_lineActor);

    noOfArrows = 4;

    actorEntry entry;
    entry.visible = true;
    entry.positionXYZ = Eigen::Vector3f(0,0,0);
    entry.orientationYPR = Eigen::Vector3f(0,0,0);
    for(int i = 0; i < noOfArrows; i++) {
        vtkSmartPointer<vtkActor> actor;
        actor = vtkSmartPointer<vtkActor>::New();
        entry.actor = actor;
        stringstream ss;
        ss << "arrow " << i;
        entry.id = ss.str();
        arrowVec.push_back(entry);
//        coneActors.at(i) = vtkSmartPointer<vtkActor>::New();
    }


    //initialize laser point
    this->laserPoint = Point(0.0, 0.0);

    //initialise values for clicking detection
    this->currentObjectIndex = 99;  //TODO: get rid of workaround
    this->selectionDuration = ros::Duration(0);
    this->idleDuration = ros::Duration(0);
    this->selection_thresh = ros::Duration(2);

    this->operationMode = BASIC;

    this->visualOdometryActive = true;

    //toggle checkbox to insert coordinate system
    ui->checkBoxCoordSys->toggle();

    //load pointcloud to speed up the workflow in the beginning
//    this->on_btnLoadPointcloud_clicked();
    this->lastSelectionTime = ros::Time::now();

    //TODO: just for the GUI, has to be reworked!
    this->previousValueSpinBoxOrientation[0] = 0;
    this->previousValueSpinBoxOrientation[1] = 0;
    this->previousValueSpinBoxOrientation[2] = 0;

    currRMSVal = 1.0;
}

MainWindow::~MainWindow() {
}

void MainWindow::transformationProcessingReady() {
    cout << "Processing ready" << endl;
    emit setTransformations(*ui);
}

void MainWindow::timerCallback() {
    cout << "timer timed out" << endl;
    if(operationMode == MOVEOBJECTS) {
        for(size_t k = 0; k < arrowVec.size(); k++) {
            removeArrow(k);
        }
        switchOperationMode(BASIC);
    }
    timer.stop();
    this->timerRunning = false;
}

void MainWindow::newTransform() {
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    if(ui->checkBoxUsePosSig->isChecked() && transformReady) {

        pclWidget->vis->setCameraParameters(T_intrProjVTK, T_world2projVTK);
        ui->qvtkWidget->update();

        this->createProjectionImageFromGUI();

        if(ui->checkBoxShowProjImage->isChecked())
            showProjectionImage();

        if(ui->checkBoxPubImage->isChecked())
            emit signalProjectImage(this->projectorImage);

        transformReady = false;

//        ui->checkBoxUsePosSig->setChecked(false);
    }
}

void MainWindow::newLine(kinpro_interaction::line line) {
    boost::lock_guard<boost::mutex> guard(m_lineMutex);

    if(ui->checkBoxActivateInteraction->isChecked()){

        if(!timerRunning) {
            cout << "starting timer" << endl;
            timer.start(5000);
            timerRunning = true;
        }
        bool lineEmpty = true;
        if(line.end.z > 0.0) {
            lineEmpty = false;
            timer.stop();
            timerRunning = false;
        }

        //clear actors
        pclWidget->vis->removeActorFromRenderer(m_lineActor);
        pclWidget->vis->removeActorFromRenderer(obbTreeActor);
        pclWidget->vis->removeActorFromRenderer(bspTreeActor);

        //transform the line points from camera into world coordinates
        Eigen::Vector4f start_cam, end_cam, start_world, end_world;
        start_cam << line.start.x, line.start.y, line.start.z, 1.0;
        end_cam << line.end.x, line.end.y, line.end.z, 1.0;
        this->transformLineToWorld(start_cam, end_cam, start_world, end_world);

        //visualize lines?
        if(ui->checkBoxShowLine->isChecked()) {
            //visualize line
            this->visualizeLine(start_world, end_world);
        }

        //clear spheres
        this->removeAllSpheres();

        //activate bounding boxes for intersection determination
        if(ui->checkBoxActivateBB->isChecked()) {

            if(!lineEmpty) {
                //calculate intersections of line with models
                vector<Eigen::Vector3f> intersections;
                vector<int> ids;
                this->intersectLineWithModels(start_world, end_world, intersections, ids);

                //check if intersections were found TODO: handle cases of multiple objects
                if(!intersections.empty()) {

                    this->lastSelectionTime = ros::Time::now();

                    //draw the intersection points (laserpointer) and highlight the models
                    for(size_t i = 0; i<intersections.size(); i++) {
                        //create spheres (laser pointer) for the intersections
                        stringstream sphereID;
                        sphereID << "Sphere " << i;
                        this->addSphere(intersections.at(i), sphereID.str());

                        //create laser points to visualize in projection image TODO: decide how to determine the most suitable intersection point if there are multiple
                        projectWorldPointToProjectorImage(intersections.at(i), this->laserPoint);

                        //highlight the intersected models
                        for(size_t j=0; j<ids.size(); j++) {
                            this->highlightActor(ids.at(j));
                        }
                    }

                    //check if a "click" happened on the selected object
                    if(this->checkForClick(ids.at(0))){

                        if(this->operationMode == BASIC){
                            //select the model
                            this->currentModel = &modelVec.at(ids.at(0));
                            //                    cout << "updated current model" << endl;
                            //show the arrows for the highlighted actor
                            cout << "adding arrows..." << endl;
                            this->addArrowsForActor(modelVec.at(ids.at(0)));
                            cout << "...done" << endl;
                            //activate the object movement mode
                            this->switchOperationMode(MOVEOBJECTS);
                        }else {
                            //detect which arrow was clicked and create movement in the selected direction TODO: rotation
                            Eigen::Vector3f translation;
                            float x, y, z;
                            x = y = z = 0;

                            //select direction
                            switch (ids.at(0)) {
                            case 0:
                                //                        translation = Eigen::Vector3f(-0.1,0,0);
                                x = 0.1;
                                break;
                            case 1:
                                //                        translation = Eigen::Vector3f(0.1,0,0);
                                x = -0.1;
                                break;
                            case 2:
                                //                        translation = Eigen::Vector3f(0,-0.1,0);
                                y = 0.1;
                                break;
                            case 3:
                                //                        translation = Eigen::Vector3f(0,0.1,0);
                                y = -0.1;
                                break;
                            case 4:
                                //                        translation = Eigen::Vector3f(0,0,-0.1);
                                z = 0.1;
                                break;
                            case 5:
                                //                        translation = Eigen::Vector3f(0,0,0.1);
                                z = -0.1;
                                break;
                            default:
                                break;
                            }
                            //                    Eigen::Matrix3f rotMat;
                            //                    setRotationMatrixFromYPR(currentModel->orientationYPR, rotMat);
                            //                    Eigen::Vector3f newTranslation;
                            //                    newTranslation = rotMat * translation;

                            //                    moveArrows(translation, Eigen::Vector3f(0,0,0));
                            //                    translation += currentModel->positionXYZ;
                            //                    moveModel(*currentModel, translation, currentModel->orientationYPR);

                            //for orientation
                            //                    float rpyIn[4], rpyOut[4];
                            //                    float diffX, diffY, diffZ;
                            //                    diffX = ui->spinBoxTestMoveRoll->value();
                            //                    diffY = ui->spinBoxTestMovePitch->value();
                            //                    diffZ = ui->spinBoxTestMoveYaw->value();
                            //                    rpyIn[0] = diffX;
                            //                    rpyIn[1] = diffY;
                            //                    rpyIn[2] = diffZ;
                            //                    rpyIn[3] = 1;

                            //                    double orientation[3];
                            //                    modelVec.at(id).actor->GetOrientation(orientation); //returned as x,y,z; order to achieve rotation is to be performed as z,x,y
                            //                    modelVec.at(id).orientationYPR(2) = orientation[0];
                            //                    modelVec.at(id).orientationYPR(1) = orientation[1];
                            //                    modelVec.at(id).orientationYPR(0) = orientation[2];

                            vtkSmartPointer<vtkMatrix4x4> mat = currentModel->actor->GetMatrix();
                            //                    mat->MultiplyPoint(rpyIn, rpyOut);

                            //                    Eigen::Vector3f rotation;

                            //                    modelVec.at(id).actor->RotateWXYZ(diffX, mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
                            //                    rotation = Eigen::Vector3f(diffZ, 0, 0);
                            //                    moveArrows(Eigen::Vector3f(0,0,0), rotation);
                            //                    modelVec.at(id).actor->RotateWXYZ(diffY, mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
                            //                    rotation = Eigen::Vector3f(0, diffY, 0);
                            //                    moveArrows(Eigen::Vector3f(0,0,0), rotation);
                            //                    modelVec.at(id).actor->RotateWXYZ(diffZ, mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));
                            //                    rotation = Eigen::Vector3f(0, 0, diffX);
                            //                    moveArrows(Eigen::Vector3f(0,0,0), rotation);


                            //detect which arrow was clicked and create movement in the selected direction
                            Eigen::Vector3f moveX, moveY, moveZ;
                            moveX = Eigen::Vector3f(mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
                            moveY = Eigen::Vector3f(mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
                            moveZ = Eigen::Vector3f(mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));

                            //select direction
                            translation = x*moveX + y*moveY + z*moveZ;
                            moveArrows(translation, Eigen::Vector3f(0,0,0));

                            moveModelRelative(*currentModel, translation, Eigen::Vector3f(0,0,0));
                        }
                    }
                } else {
                    //reset clicking Times
                    this->selectionBegin = ros::Time::now();
                    this->selectionDuration = ros::Duration(0);

                    //stop drawing the clicking circle
                    this->drawClickingCircle = false;

                    //set point coordinates to zero to use if no intersections were found
                    this->laserPoint = Point(0.0, 0.0);

                    if(end_world(0) > 0.0 && end_world(1) > 0.0 && end_world(2) > 0.0) {
                        stringstream sphereID;
                        sphereID << "Sphere end";
                        Eigen::Vector3f lineEnd(end_world(0), end_world(1), end_world(2));
                        this->addSphere(lineEnd, sphereID.str());
                    }

                    //            cout << "intersection empty" << endl;
                    //            if(operationMode == MOVEOBJECTS) {
                    //                idleDuration = ros::Time::now() - this->lastSelectionTime;
                    //                cout << "idleDuration: " << idleDuration.toSec() << endl;
                    //                if(idleDuration.toSec() > this->idle_thresh.toSec()) {
                    //                    for(size_t k = 0; k < arrowVec.size(); k++) {
                    //                        removeArrow(k);
                    //                    }
                    //                    switchOperationMode(BASIC);
                    //                }
                    //            }
                }
            }
        }

        //    if(ui->checkBoxShowProjImage->isChecked()) {
        //        this->createProjectionImageFromGUI();
        //        showProjectionImage();
        //    }


        ui->qvtkWidget->update();
    }
}

void MainWindow::moveArrows(Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR) {
    Eigen::Vector3f translation, rotation;
    for(size_t i = 0; i < arrowVec.size(); i++) {
        translation = arrowVec.at(i).positionXYZ + translateXYZ;
        rotation = arrowVec.at(i).orientationYPR + rotateYPR;
//        moveModel(arrowVec.at(i), translation, rotation);
        moveModelRelative(arrowVec.at(i), translateXYZ, rotateYPR);
    }
}

void MainWindow::transformLineToWorld(Eigen::Vector4f &pt_start, Eigen::Vector4f &pt_end, Eigen::Vector4f &pt_start_world, Eigen::Vector4f &pt_end_world) {
    //transform the start and end point of the line into the world coordinates
    pt_start_world = T_world2camVTK * pt_start;
    pt_end_world = T_world2camVTK * pt_end;
}

void MainWindow::visualizeLine(Eigen::Vector4f &start, Eigen::Vector4f &end) {
    double startPt[3], endPt[3];

    startPt[0] = start(0)/start(3);
    startPt[1] = start(1)/start(3);
    startPt[2] = start(2)/start(3);

    endPt[0] = end(0)/end(3);
    endPt[1] = end(1)/end(3);
    endPt[2] = end(2)/end(3);

    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(startPt);
    lineSource->SetPoint2(endPt);
    lineSource->Update();
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(lineSource->GetOutputPort());
    m_lineActor->SetMapper(mapper);
    m_lineActor->GetProperty()->SetColor(1.0,1.0,1.0);
    pclWidget->vis->addActorToRenderer(m_lineActor);
}

bool MainWindow::checkForClick(int id) {
    //check if the selected object is the previously selected object
    if(this->currentObjectIndex == id){
        //increment the selection duration
        this->selectionDuration = ros::Time::now() - this->selectionBegin;

        //draw timing circle onto projector image
//        this->drawClickingCircle = true;
        this->drawClickingCircle = true;

        //check if the selection duration threshold was reached
        if(this->selectionDuration > this->selection_thresh) {
            //click was executed
            cout << "clicked after " << this->selectionDuration.toSec() << "secs" << endl;
            this->selectionBegin = ros::Time::now();
            return true;
        }

    } else {
        //don't draw the clicking circle
        this->drawClickingCircle = false;

        //change the name of the selected object
        this->currentObjectIndex = id;

        //reset the selection time and the selection duration
        this->selectionBegin = ros::Time::now();
        this->selectionDuration = ros::Duration(0);
    }
    return false;
}

void MainWindow::addSphere(Eigen::Vector3f &center, string id) {

    if(!pclWidget->vis->addSphere(pcl::PointXYZ(center(0), center(1), center(2)), 0.01, 0.0, 1.0, 0.0, id))
        pclWidget->vis->updateSphere(pcl::PointXYZ(center(0), center(1), center(2)), 0.01, 0.0, 1.0, 0.0, id);
    sphereIDs.push_back(id);                //store sphere names for later removal
}

void MainWindow::removeSphere(string &id) {
    if(pclWidget->vis->removeShape(id)) {
        cout << "Sphere with ID " << id << " removed." << endl;
    } else {
        cout << "Error: Sphere with ID " << id << " could not be removed. Does it exist?" << endl;
    }

}

void MainWindow::removeAllSpheres() {
    for(size_t i = 0; i<sphereIDs.size(); i++) {
        pclWidget->vis->removeShape(sphereIDs.at(i));
    }
    sphereIDs.clear();
//    cout << "All spheres removed succesfully." << endl;
}

void MainWindow::addArrow(Eigen::Vector3f &center, Eigen::Vector3f &axis, float length, float radius, float resolution, int id) {

    //Create a cone
    vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
    coneSource->SetResolution(resolution);
    coneSource->SetCenter(0, 0, 0);
    coneSource->SetDirection(axis(0), axis(1), axis(2));
    coneSource->SetRadius(radius);
    coneSource->SetHeight(length);
    coneSource->Update();

    //Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(coneSource->GetOutputPort());

//    vtkSmartPointer<vtkActor> testAc = vtkSmartPointer<vtkActor>::New();
//    testAc->SetMapper(mapper);
//    pclWidget->vis->addActorToRenderer(testAc);

    arrowVec.at(id).actor->SetMapper(mapper);
    double position[3];
    position[0] = center(0);
    position[1] = center(1);
    position[2] = center(2);
    arrowVec.at(id).actor->SetPosition(position[0], position[1], position[2]);
    arrowVec.at(id).positionXYZ = center;
    arrowVec.at(id).orientationYPR = axis;

    //Add the actors to the scene
    pclWidget->vis->removeActorFromRenderer(arrowVec.at(id).actor);
    pclWidget->vis->addActorToRenderer(arrowVec.at(id).actor);

    ui->qvtkWidget->update();
}

void MainWindow::removeArrow(int id) {
        pclWidget->vis->removeActorFromRenderer(arrowVec.at(id).actor);
}

void MainWindow::removeAllArrows() {
    for(int i = 0; i<arrowVec.size(); i++){
        pclWidget->vis->removeActorFromRenderer(arrowVec.at(i).actor);
    }
}

void MainWindow::addArrowsForActor(actorEntry &actor) {
    Eigen::Vector3f center, axis;
    Eigen::Matrix3f rotMat;
    float length, radius, resolution;
    resolution = 50;


    //get actor position and orientation
//    setRotationMatrixFromYPR(DEG2RAD(actor.orientationYPR), rotMat);
    vtkSmartPointer<vtkMatrix4x4> mat = actor.actor->GetMatrix();
//    cout    << "Rotation Matrix" << endl
//            << rotMat(0,0) << "\t" << rotMat(0,1) << "\t" << rotMat(0,2) << endl
//            << rotMat(1,0) << "\t" << rotMat(1,1) << "\t" << rotMat(1,2) << endl
//            << rotMat(2,0) << "\t" << rotMat(2,1) << "\t" << rotMat(2,2) << endl;

    //get actor size regarding all 3 dimensions
    double position[3];
    actor.actor->GetPosition(position);
    cout << "Position: " << position[0] << "\t" << position[1] << "\t" << position[2] << endl;

    double bounds[6];
    actor.actor->GetBounds(bounds);
//    for(int i = 0; i < 6; i++){
//        actor.bounds.at(i) = bounds[i];
//    }
    length = 0;
    for(int i = 0; i < actor.bounds.size(); i+=2) {
        double diff = abs(actor.bounds[i]-actor.bounds[i+1]);
        cout << "diff: " << diff << endl;
        if(diff/2 > length)
            length = diff/2;
    }
//    length /= 2;
//    length += 0.1;
    radius = length/2;

    cout << "radius = " << radius << "\tlength = " << length << endl;

    for(size_t i = 0; i < arrowVec.size(); i++) {

        int col = i/2;
//        axis << rotMat(0,col), rotMat(1,col), rotMat(2,col);
        axis << mat->GetElement(0,col) , mat->GetElement(1,col), mat->GetElement(2,col);
        center << (bounds[0]+bounds[1])/2, (bounds[2]+bounds[3])/2, (bounds[4]+bounds[5])/2;


        if(i%2){
            axis *= (-abs(actor.bounds[i]-actor.bounds[i-1])/2-length);
        }else {
            axis *= (+abs(actor.bounds[i]-actor.bounds[i+1])/2+length);
        }

        center += axis;
        cout << "Center: " << center(0) << "\t" << center(1) << "\t" << center(2) <<  "\t" << "(" << i << ")" << endl;

        //add arrows for all dimensions
        this->addArrow(center, axis, length, radius, resolution, i);
    }
}

//void MainWindow::highlightActor(string &id) {
//    if(operationMode == BASIC) {
//        for(size_t k=0; k<modelVec.size(); k++) {
//            if(!strcmp(id.c_str(), modelVec.at(k).id.c_str()))
//                modelVec.at(k).actor->GetProperty()->SetColor(0.87, 0.898, 0.7);
//        }
//    }else if (operationMode == MOVEOBJECTS) {
//        for(size_t k=0; k<coneActors.size(); k++) {
//            stringstream ss;
//            ss << k;
//            if(!strcmp(id.c_str(), ss.str().c_str()))
//                coneActors.at(k)->GetProperty()->SetColor(0.87, 0.898, 0.7);
//        }
//    }
//}

void MainWindow::highlightActor(int id) {
    if(operationMode == BASIC) {
        modelVec.at(id).actor->GetProperty()->SetColor(0.87, 0.898, 0.7);
    }else if (operationMode == MOVEOBJECTS) {
        arrowVec.at(id).actor->GetProperty()->SetColor(0.87, 0.898, 0.7);
    }
}

void MainWindow::intersectLineWithModels(Eigen::Vector4f& start, Eigen::Vector4f& end, std::vector<Eigen::Vector3f>& intersections, std::vector<int> &ids) {

//    cout << "intersecting" << endl;
    if(operationMode == BASIC) {

        if(!modelVec.empty()) {
//            cout << "modelvec not empty" << endl;
            //calculate intersections with all models
            for(size_t cnt = 0; cnt < modelVec.size(); cnt++) {

                //only iterate over visible models
                if(modelVec.at(cnt).visible) {

                    //reset model color
                    modelVec.at(cnt).actor->GetProperty()->SetColor(1, 1, 1);

                    Eigen::Vector3f intersectionPoint;

                    double pt_start[3], pt_end[3];

                    pt_start[0] = start(0)/start(3);
                    pt_start[1] = start(1)/start(3);
                    pt_start[2] = start(2)/start(3);

                    pt_end[0] = end(0)/end(3);
                    pt_end[1] = end(1)/end(3);
                    pt_end[2] = end(2)/end(3);

                    //copy actor
                    vtkSmartPointer<vtkActor> intersectionActor = vtkSmartPointer<vtkActor>::New();
                    intersectionActor->ShallowCopy(modelVec.at(cnt).actor);
//                    vtkSmartPointer<vtkPolyData> intersectionDataSet = vtkSmartPointer<vtkPolyData>::New();
//                    intersectionDataSet->DeepCopy(intersectionActor->GetMapper()->GetInput());

                    //generate transform
                    vtkSmartPointer<vtkTransform> transformBB = vtkSmartPointer<vtkTransform>::New();
                    transformBB->SetMatrix(intersectionActor->GetMatrix());

                    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
                    transformFilter->SetInput(intersectionActor->GetMapper()->GetInput());
                    transformFilter->SetTransform(transformBB);
                    transformFilter->Update();

                    // Create a mapper and actor
                    vtkSmartPointer<vtkPolyDataMapper> transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                    transformedMapper->SetInputConnection(transformFilter->GetOutputPort());
                    intersectionActor->SetMapper(transformedMapper);

                    //use obbTree model TODO: use same procedure for both models
                    if(ui->radioButtonOBB->isChecked()) {

                        //create obbTree
                        obbTree->SetDataSet(intersectionActor->GetMapper()->GetInput());
                        obbTree->BuildLocator();

                        //Visualize obbTree bounding box
                        if(ui->checkBoxShowBB->isChecked()) {
                            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
                            obbTree->GenerateRepresentation(0, polydata);
                            vtkSmartPointer<vtkPolyDataMapper> obbtreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                            obbtreeMapper->SetInput(polydata);
                            obbTreeActor->SetMapper(obbtreeMapper);
                            pclWidget->vis->addActorToRenderer(obbTreeActor);
                        }

                        vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();

                        //calculate intersection
                        int obbHit = obbTree->IntersectWithLine(pt_start, pt_end, intersectPoints, NULL);

                        //print and save intersection points
                        if(obbHit) {
//                            cout << "Hit! (" << obbHit << ")" << endl << "Model " << modelVec.at(cnt).id << endl;
                            ids.push_back(cnt);
                            double intersection[3];
                            for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
                            {
                                intersectPoints->GetPoint(i, intersection);
                                //                            std::cout << "Intersection " << i << ": " << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << std::endl;
                                intersectionPoint << intersection[0], intersection[1], intersection[2];
                                intersections.push_back(intersectionPoint);
                            }
                        }
                    }

                    //use bspTree model
                    if(ui->radioButtonBSP->isChecked()) {

                        //build bspTree
                        bspTree->SetDataSet(intersectionActor->GetMapper()->GetInput());
                        bspTree->BuildLocator();

                        //visualize bspTree bounding box
                        if(ui->checkBoxShowBB->isChecked()) {
                            vtkSmartPointer<vtkPolyData> bspPoly = vtkSmartPointer<vtkPolyData>::New();
                            bspTree->GenerateRepresentation(0, bspPoly);
                            vtkSmartPointer<vtkPolyDataMapper> bsptreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                            bsptreeMapper->SetInput(bspPoly);
                            bspTreeActor->SetMapper(bsptreeMapper);
                            pclWidget->vis->addActorToRenderer(bspTreeActor);
                        }

                        //calculate intersection
                        double tolerance = .001;    //tolerance
                        double t;                   //parametric coordinate of intersection (0 (corresponding to p1) to 1 (corresponding to p2))
                        double x[3];                //the coordinate of the intersection
                        double pcoords[3];
                        int subId;
                        int bspHit = bspTree->IntersectWithLine(pt_start, pt_end, tolerance, t, x, pcoords, subId);

                        //print and save intersection points
                        if(bspHit) {
                            cout << "Hit! (" << bspHit << ")" << endl << "Model " << modelVec.at(cnt).id << endl;
                            ids.push_back(cnt);
                            //                        std::cout << "Intersection: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
                            intersectionPoint << x[0], x[1], x[2];
                            intersections.push_back(intersectionPoint);
                        }
                    }
                }
            }
        }
    } else if(operationMode == MOVEOBJECTS) {
        if(!arrowVec.empty()) {

            //calculate intersections with all arrows
            for(size_t cnt = 0; cnt < arrowVec.size(); cnt++) {

                    //reset model color
                    arrowVec.at(cnt).actor->GetProperty()->SetColor(1, 1, 1);

                    Eigen::Vector3f intersectionPoint;

                    double pt_start[3], pt_end[3];

                    pt_start[0] = start(0)/start(3);
                    pt_start[1] = start(1)/start(3);
                    pt_start[2] = start(2)/start(3);

                    pt_end[0] = end(0)/end(3);
                    pt_end[1] = end(1)/end(3);
                    pt_end[2] = end(2)/end(3);

                    //copy actor
                    vtkSmartPointer<vtkActor> intersectionActor = vtkSmartPointer<vtkActor>::New();
                    intersectionActor->ShallowCopy(arrowVec.at(cnt).actor);
                    vtkSmartPointer<vtkPolyData> intersectionDataSet = vtkSmartPointer<vtkPolyData>::New();
                    intersectionDataSet->DeepCopy(intersectionActor->GetMapper()->GetInput());

                    //generate transform
                    vtkSmartPointer<vtkTransform> transformBB = vtkSmartPointer<vtkTransform>::New();
                    transformBB->SetMatrix(intersectionActor->GetMatrix());

                    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
                    transformFilter->SetInput(intersectionActor->GetMapper()->GetInput());
                    transformFilter->SetTransform(transformBB);
                    transformFilter->Update();

                    // Create a mapper and actor
                    vtkSmartPointer<vtkPolyDataMapper> transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                    transformedMapper->SetInputConnection(transformFilter->GetOutputPort());
                    intersectionActor->SetMapper(transformedMapper);

                    //use obbTree model TODO: use same procedure for both models
                    if(ui->radioButtonOBB->isChecked()) {

                        //create obbTree
                        obbTree->SetDataSet(intersectionActor->GetMapper()->GetInput());
                        obbTree->BuildLocator();

                        //Visualize obbTree bounding box
                        if(ui->checkBoxShowBB->isChecked()) {
                            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
                            obbTree->GenerateRepresentation(0, polydata);
                            vtkSmartPointer<vtkPolyDataMapper> obbtreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                            obbtreeMapper->SetInput(polydata);
                            obbTreeActor->SetMapper(obbtreeMapper);
                            pclWidget->vis->addActorToRenderer(obbTreeActor);
                        }

                        vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();

                        //calculate intersection
                        int obbHit = obbTree->IntersectWithLine(pt_start, pt_end, intersectPoints, NULL);

                        //print and save intersection points
                        if(obbHit) {
//                            cout << "Hit! (" << obbHit << ")" << endl << "Model " << modelVec.at(cnt).id << endl;
                            stringstream id;
                            id << cnt;
                            ids.push_back(cnt);
                            double intersection[3];
                            for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
                            {
                                intersectPoints->GetPoint(i, intersection);
//                                std::cout << "Intersection " << i << ": " << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << std::endl;
                                intersectionPoint << intersection[0], intersection[1], intersection[2];
                                intersections.push_back(intersectionPoint);
                            }
                        }
                    }

                    //use bspTree model
                    if(ui->radioButtonBSP->isChecked()) {

                        //build bspTree
                        bspTree->SetDataSet(intersectionActor->GetMapper()->GetInput());
                        bspTree->BuildLocator();

                        //visualize bspTree bounding box
                        if(ui->checkBoxShowBB->isChecked()) {
                            vtkSmartPointer<vtkPolyData> bspPoly = vtkSmartPointer<vtkPolyData>::New();
                            bspTree->GenerateRepresentation(0, bspPoly);
                            vtkSmartPointer<vtkPolyDataMapper> bsptreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                            bsptreeMapper->SetInput(bspPoly);
                            bspTreeActor->SetMapper(bsptreeMapper);
                            pclWidget->vis->addActorToRenderer(bspTreeActor);
                        }

                        //calculate intersection
                        double tolerance = .001;    //tolerance
                        double t;                   //parametric coordinate of intersection (0 (corresponding to p1) to 1 (corresponding to p2))
                        double x[3];                //the coordinate of the intersection
                        double pcoords[3];
                        int subId;
                        int bspHit = bspTree->IntersectWithLine(pt_start, pt_end, tolerance, t, x, pcoords, subId);

                        //print and save intersection points
                        if(bspHit) {
                            //                        cout << "Hit! (" << bspHit << ")" << endl << "Model " << modelVec.at(cnt).id << endl;
                            stringstream id;
                            id << cnt;
                            ids.push_back(cnt);
                            //                        std::cout << "Intersection: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
                            intersectionPoint << x[0], x[1], x[2];
                            intersections.push_back(intersectionPoint);
                        }
                    }
            }
        }



    }
}

void MainWindow::projectWorldPointToProjectorImage(Eigen::Vector3f &pt_world, cv::Point &pt_projector) {
    //transform world point to projector coordinates

    //calculate the transformation between 3D points and the projector image
    Eigen::Vector4f p_proj3D_hom, worldPoint_hom;
    Eigen::Vector3f p_proj3D, p_proj2D_hom;
    Eigen::Vector2f p_proj2D;

//    cout << "3D: " << scale3D << " 2Dx: " << scale2Dx << " 2Dy: " << scale2Dy << endl;

    worldPoint_hom << pt_world(0), pt_world(1), pt_world(2), 1.0;

    //transform 3D points to 3D projector coordinates
    p_proj3D_hom = T_world2proj.inverse() * worldPoint_hom;
    p_proj3D << p_proj3D_hom(0), p_proj3D_hom(1), p_proj3D_hom(2);

    //transform 3D points to 2D projector pixel values
    p_proj2D_hom = T_intrProj * p_proj3D;
    p_proj2D << p_proj2D_hom(0)/p_proj2D_hom(2), p_proj2D_hom(1)/p_proj2D_hom(2);

    pt_projector.x  = p_proj2D(0);
    pt_projector.y  = p_proj2D(1);
}

void MainWindow::loadPointCloud(string filename) {
    boost::lock_guard<boost::mutex> guard(m_cloudMtx);

    //stop getting pointclouds from the kinect input stream
    ui->checkBoxKinect->setChecked(false);

    //load the pointcloud
    PointCloud<PointXYZRGB> pc_load;
    pcl::io::loadPCDFile(filename, pc_load);

    //transform cloud from rgb frame to world frame
    if(ui->checkBoxCalibMode->isChecked()) {
        m_pc = pc_load.makeShared();
        m_pc_bckp = pc_load.makeShared();
        this->displayCloud(m_pc, displayRGBCloud);
    }else {

        if(ui->checkBoxTransformPointcloud->isChecked())
            pcl::transformPointCloud(pc_load, pc_load, T_world2camVTK);

        PCEntry entry;
        entry.cloud = pc_load.makeShared();
        int cnt = 1;
        bool nameIsValid = false;
        stringstream newName;
        newName << filename;
        if(!PCVec.empty()){
            while(!nameIsValid) {
                for(int i = 0; i < PCVec.size(); i++) {
                    string currentID = PCVec.at(i).id;
                    if(!strcmp(currentID.c_str(), newName.str().c_str())) {
                        newName.str(std::string());;
                        newName << filename << " (" << cnt << ")";
                        break;
                    }else if(i==PCVec.size()-1) {
                        nameIsValid = true;
                    }
                }
                cnt++;
            }
        }else {
            nameIsValid = true;
        }
        entry.id = newName.str();
        entry.visible = true;
        entry.positionXYZ = Eigen::Vector3f(0,0,0);
        entry.orientationYPR = Eigen::Vector3f(0,0,0);
        PCVec.push_back(entry);

        this->displayCloud(PCVec.back().cloud, displayRGBCloud, PCVec.back().id);
        this->updatePCIndex();
    }
}

void MainWindow::savePointCloud(string filename) {
    PointCloud<PointXYZRGB> pc_save(*m_pc);
    pcl::io::savePCDFileBinary(filename, pc_save);
}

void MainWindow::displayCloud(PointCloud<PointXYZRGB>::Ptr pc, bool color, string id) { //only the pointcloud with the specified id is updated; default is "cloud"
    if(color){
        if(!pclWidget->vis->updatePointCloud(pc, id)) {
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, rgb, id);
            pclWidget->vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
//            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

            cout << "Adding new RGB cloud" << endl;
        }
    }else {
        if(!pclWidget->vis->updatePointCloud<pcl::PointXYZRGB>(pc, id)) {      //adding <pcl::PointXYZRGB> leads to ignoring color values
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

            cout<< "Adding new cloud" << endl;
        }
    }

//    ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());
    ui->qvtkWidget->update();
}

void MainWindow::displayCloudSingleColor(PointCloud<PointXYZRGB>::Ptr pc, float red, float green, float blue, string id) { //only the pointcloud with the specified id is updated; default is "cloud"
    if(!pclWidget->vis->updatePointCloud(pc, id)) {
//            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
        pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);
        pclWidget->vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue, id);
//            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

        cout << "Adding new Color cloud - rgb (" << red << ", " << green << ", " << blue << ")" << endl;
    }
    ui->qvtkWidget->update();
}

void MainWindow::newPointCloud(PointCloud<PointXYZRGB> pc) {
    boost::lock_guard<boost::mutex> guard(m_cloudMtx);
    if(ui->checkBoxKinect->isChecked()) {
            m_pc = pc.makeShared();
            this->processCloud(m_pc);
            m_pc_bckp = pc.makeShared();
            this->displayCloud(m_pc, displayRGBCloud);
    }
}

void MainWindow::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //transform cloud from rgb_frame to world frame
    if(ui->checkBoxTransformPointcloud->isChecked())
        pcl::transformPointCloud(*cloud, *cloud, T_world2camVTK);
}

void MainWindow::applyPassthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (line2float(*ui->linePass_min), line2float(*ui->linePass_max));
    pass.filter (*cloud);
}

void MainWindow::applyVoxelization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    float resolution = line2float(*ui->lineVoxRes);

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (resolution, resolution, resolution);
    vg.filter (*cloud);
}

void MainWindow::applySegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.005);
    seg.setMaxIterations (100);

    //single plane segmentation
//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);

//    // Project the model inliers
//    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
//    proj.setModelType (pcl::SACMODEL_PLANE);
//    proj.setInputCloud (cloud);
//    proj.setIndices (inliers);
//    proj.setModelCoefficients (coefficients);
//    proj.filter (*cloud);

    //multi plane segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_plane_proj (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_f (new pcl::PointCloud<pcl::PointXYZRGB> ());

    int vecSize = segmentedPlanes.size();
    for( int i = 0; i < vecSize; i++ ) {
        stringstream ss;
        ss << "Plane " << i;
        pclWidget->vis->removePointCloud(ss.str());
    }

    segmentedPlanes.clear();

    int nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.1 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);

        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.setIndices(inliers);
        proj.filter(*cloud_plane_proj);

        // Get the points associated with the planar surface
        segmentedPlanes.push_back(*cloud_plane_proj);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud = *cloud_f;
    }
}

void MainWindow::applyHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if(ui->radioConvex->isChecked()) {
        pcl::ConvexHull<pcl::PointXYZRGB> chull;
        chull.setInputCloud (cloud);
        chull.reconstruct (*cloud);
    }else if(ui->radioConcave->isChecked()) {
        pcl::ConcaveHull<pcl::PointXYZRGB> chull;
        chull.setInputCloud (cloud);
        chull.setAlpha (line2double(*ui->lineHullAlpha));
        chull.reconstruct (*cloud);
    }
}

void MainWindow::applyTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //calculate the transformation between 3D points and the projector image
    Eigen::Vector4f p_proj3D_hom, p_cam3D_hom;
    Eigen::Vector3f p_proj3D, p_proj2D_hom;
    Eigen::Vector2f p_proj2D;
    vector<Eigen::Vector2f> pixVec;

    for(int i=0; i<cloud->points.size(); i++) {
        //create homogenous vector of 3D contour points
        p_cam3D_hom << cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z, 1;

        //transform 3D points to 3D projector coordinates
        p_proj3D_hom = T_cam2proj.inverse() * p_cam3D_hom;
        p_proj3D << p_proj3D_hom(0), p_proj3D_hom(1), p_proj3D_hom(2);

        //transform 2D points to 2D projector pixel values
        p_proj2D_hom = T_intrProj * p_proj3D;
        p_proj2D << p_proj2D_hom(0)/p_proj2D_hom(2), p_proj2D_hom(1)/p_proj2D_hom(2);
        pixVec.push_back(p_proj2D);
    }

    vector<Point> pVec;
    for(int i=0; i<pixVec.size(); i++) {
        int x = (int)pixVec.at(i)(0);
        int y = (int)pixVec.at(i)(1);
        if(x<848 && x>=0 && y<480 && y>=0) {
            pVec.push_back(Point(x, y));
        }
    }
    this->projectionContour.clear();
    this->projectionContour.push_back(pVec);
}

void MainWindow::createProjectionImage()
{
    this->projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);
    if(!this->projectionContour[0].empty()) {
        vector<vector<Point> > contours_poly( this->projectionContour.size() );
        vector<Rect> boundRect( this->projectionContour.size() );
        cv::approxPolyDP( Mat(this->projectionContour[0]), contours_poly[0], line2double(*ui->lineVoxRes), ui->checkBoxRect->isChecked() );

        if(ui->checkBoxContours->isChecked())
            cv::drawContours(this->projectorImage, this->projectionContour, 0, Scalar(0,255,0), -1);

        if(ui->checkBoxPoly->isChecked())
            fillPoly(this->projectorImage, contours_poly, Scalar(0,0,255));

        if(ui->checkBoxPolyLines->isChecked()) {
            for(int i=0; i<contours_poly[0].size()-1; i++) {
                line(this->projectorImage, contours_poly[0][i], contours_poly[0][i+1], Scalar(0, 0, 255), 2);
            }
        }

        if(ui->checkBoxContoursLines->isChecked()) {
            for(int i=0; i<this->projectionContour[0].size()-1; i++) {
                line(this->projectorImage, this->projectionContour[0][i], this->projectionContour[0][i+1], Scalar(0, 255, 0), 2);
            }
        }

        if(ui->checkBoxRect->isChecked()) {
            boundRect[0] = boundingRect( Mat(contours_poly[0]) );
            cv::rectangle(this->projectorImage, boundRect[0], Scalar(255,255,255), 2);
        }
    }

//    if(ui->checkBoxShowProjImage->isChecked())
        showProjectionImage();

    if(ui->checkBoxPubImage->isChecked())
        emit signalProjectImage(this->projectorImage);
}

void MainWindow::showProjectionImage()
{
    cv::imshow("projected Image", this->projectorImage);
    cv::waitKey(1);
}

void MainWindow::createProjectionImageFromGUI()
{
    this->projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);
    vtkSmartPointer<vtkRenderWindow> renderWindow = ui->qvtkWidget->GetRenderWindow();
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput( renderWindow );

    //get image
    windowToImageFilter->SetInputBufferTypeToRGB();
    windowToImageFilter->Update();
    vtkImageData* vtkimage = windowToImageFilter->GetOutput();
    int dimsImage[3];
    vtkimage->GetDimensions(dimsImage);

    try{
        cv::Mat cvImage(dimsImage[1], dimsImage[0], CV_8UC3, vtkimage->GetScalarPointer());
        cv::cvtColor( cvImage, cvImage, CV_RGB2BGR); //convert color
        cv::flip( cvImage, cvImage, 0); //align axis with visualizer
        ros::Duration t_diff = ros::Time::now()-lastLocTime;
        int border = (t_diff.toSec() > 8.0 ? 8 : 24-2*(int)(t_diff.toSec()));
        cv::Rect roi(border,border,848-2*border,480-2*border);      //TODO projector size param
        if(currRMSVal > 0.1)
            currRMSVal = 0.1;
        this->projectorImage.setTo(Scalar(255*(currRMSVal*10),255*(1-currRMSVal*10),0));
//        this->projectorImage = cvImage(roi).clone();
        cvImage(roi).copyTo(this->projectorImage(roi));
//        if(this->laserPoint.x != 0.0 && this->laserPoint.y != 0.0 )
//            circle(this->projectorImage, this->laserPoint, 8.0, Scalar(0, 255, 0),3);

//        if(this->drawClickingCircle) {
//            double angle = (this->selectionDuration.toSec() / this->selection_thresh.toSec()) * 360.0;
//            ellipse(this->projectorImage, Point(424,240), Size(40, 40), angle, 0.0, 360.0, Scalar(0, 0, 0), 5);
//        }
    }catch (cv::Exception e) {
        cout << "could not create image! Error: " << e.what() << endl;
    }

}



void MainWindow::on_checkBoxRGBCloud_toggled(bool checked)
{
    this->displayRGBCloud = checked;
}

void MainWindow::on_checkBoxCoordSys_toggled(bool checked)
{
    if(checked){
        pclWidget->vis->addCoordinateSystem(0.5,0.0,0.0,0.0);
    }else{
        pclWidget->vis->removeCoordinateSystem();
    }
    ui->qvtkWidget->update();

}

void MainWindow::on_btnSetCamView_clicked()
{
//    this->setTransformations();
    emit setTransformations(*ui);

    pclWidget->vis->setCameraParameters(T_intrProjVTK, T_world2projVTK);
    ui->qvtkWidget->update();
}

void MainWindow::on_btnLoadPointcloud_clicked()
{
    std::stringstream cloudName;
    string lineTxt = ui->lineLoadPointcloud->text().toStdString();
    cloudName << lineTxt;

    this->loadPointCloud(cloudName.str());
}

void MainWindow::on_btnSavePointcloud_clicked()
{
    std::stringstream cloudName;
    string lineTxt = ui->lineSavePointcloud->text().toStdString();
    cloudName << lineTxt;

    this->savePointCloud(cloudName.str());
}

void MainWindow::on_btnTransformApply_clicked()
{
//    this->setTransformations();
    emit setTransformations(*ui);
}

void MainWindow::on_btnResetIntrFoc_clicked()
{
    //original transformation params from camera calibration TODO: these are the old values, can be changed if calibration is validated as correct
    ui->lineIntrinsicParamsProj_fx->setText("1515.51089");
    ui->lineIntrinsicParamsProj_fy->setText("1447.40731");
}

void MainWindow::on_btnResetIntrPrinc_clicked()
{
    //original transformation params from camera calibration
//    ui->lineIntrinsicParamsProj_cx->setText("437.37754");
//    ui->lineIntrinsicParamsProj_cy->setText("515.55742");
    //modified transformation params for vtk camera positioning (projector view)
//    ui->lineIntrinsicParamsProj_cx->setText("848");
//    ui->lineIntrinsicParamsProj_cy->setText("480");
    //modified transformation params for vtk camera positioning (projector view) - 2x principal point -> generates correct perspective but wrong image size
    ui->lineIntrinsicParamsProj_cx->setText("874.75508");
    ui->lineIntrinsicParamsProj_cy->setText("1031.11484");
}

void MainWindow::on_btnResetExtrRot_clicked()
{
    //original transformation params from camera calibration interpreted as [row; row; row]
//    ui->lineRotCamProj_00->setText("0.9999");
//    ui->lineRotCamProj_01->setText("-0.0104");
//    ui->lineRotCamProj_02->setText("-0.0106");
//    ui->lineRotCamProj_10->setText("0.0073");
//    ui->lineRotCamProj_11->setText("0.9661");
//    ui->lineRotCamProj_12->setText("-0.2582");
//    ui->lineRotCamProj_20->setText("0.0129");
//    ui->lineRotCamProj_21->setText("0.2581");
//    ui->lineRotCamProj_22->setText("0.9660");

    //original transformation params from camera calibration interpreted as [column; column; column]
    ui->lineRotCamProj_00->setText("0.9999");
    ui->lineRotCamProj_01->setText("0.0073");
    ui->lineRotCamProj_02->setText("0.0129");
    ui->lineRotCamProj_10->setText("-0.0104");
    ui->lineRotCamProj_11->setText("0.9661");
    ui->lineRotCamProj_12->setText("0.2581");
    ui->lineRotCamProj_20->setText("-0.0106");
    ui->lineRotCamProj_21->setText("-0.2582");
    ui->lineRotCamProj_22->setText("0.9660");
}

void MainWindow::on_btnResetExtrTrans_clicked()
{
    //modified transformation params from camera calibration
//    ui->lineTransCamProjX->setText("0.027");
//    ui->lineTransCamProjY->setText("0.049");
//    ui->lineTransCamProjZ->setText("0.020");
    //original transformation params for vtk camera positioning
    ui->lineTransCamProjX->setText("-0.027");
    ui->lineTransCamProjY->setText("-0.049");
    ui->lineTransCamProjZ->setText("-0.020");
}

void MainWindow::on_btnCreateImgFromGUI_clicked()
{
    this->createProjectionImageFromGUI();

    this->showProjectionImage();
}

void MainWindow::on_sliderPass_min_valueChanged(int value)
{
    double val = value / 100.0;
    double2line(*ui->linePass_min, val);
}

void MainWindow::on_sliderPass_max_valueChanged(int value)
{
    double val = value / 100.0;
    double2line(*ui->linePass_max, val);
}

void MainWindow::on_linePass_min_textEdited(const QString &arg1)
{
    int val = (int)(100.0 * line2double(*ui->linePass_min));
    ui->sliderPass_min->setValue(val);
}

void MainWindow::on_linePass_max_textEdited(const QString &arg1)
{
    int val = (int)(100.0 * line2double(*ui->linePass_max));
    ui->sliderPass_max->setValue(val);
}

void MainWindow::on_btnPassthroughPreview_clicked()
{
    ui->checkBoxKinect->setChecked(false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    //apply passthrough filter
    this->applyPassthrough(cloud);

    displayCloud(cloud, displayRGBCloud);
}

void MainWindow::on_btnPassthroughApply_clicked()
{
    ui->checkBoxKinect->setChecked(false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    //apply passthrough filter
    this->applyPassthrough(cloud);

    m_pc = cloud->makeShared();
    displayCloud(m_pc, displayRGBCloud);
}

void MainWindow::on_btnSegmentatePre_clicked()
{
    //deactivate kinect input
    ui->checkBoxKinect->setChecked(false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    //apply plane segmentation
    this->applySegmentation(cloud);


    //list selectable planes
    ui->comboBoxPlanes->clear();

    if(!segmentedPlanes.empty()){
        pclWidget->vis->removePointCloud("cloud");
        int vecSize = segmentedPlanes.size();
        for( int i = 0; i < vecSize; i++ ) {
            stringstream ss;
            ss << "Plane " << i;
            ui->comboBoxPlanes->addItem(QString::fromStdString(ss.str()));
            this->displayCloudSingleColor(segmentedPlanes.at(i).makeShared(), ((float)i/(float)vecSize), (1.0-(fabs(((float)i/(float)vecSize)-0.5)*2)), ((float)(vecSize-i)/(float)vecSize), ss.str());
        }

    }
}

void MainWindow::on_btnSegmentate_clicked()
{
    //deactivate kinect input
    ui->checkBoxKinect->setChecked(false);
    int id = ui->comboBoxPlanes->currentIndex();
    if(segmentedPlanes.size() > id)
        m_pc = segmentedPlanes.at(id).makeShared();
    int vecSize = segmentedPlanes.size();
    for( int i = 0; i < vecSize; i++ ) {
        stringstream ss;
        ss << "Plane " << i;
        pclWidget->vis->removePointCloud(ss.str());
    }
    displayCloud(m_pc, displayRGBCloud);
}

void MainWindow::on_comboBoxPlanes_activated(int index)
{
    if(!segmentedPlanes.empty() && index < segmentedPlanes.size()) {
        int vecSize = segmentedPlanes.size();
        for( int i = 0; i < vecSize; i++ ) {
            stringstream ss;
            ss << "Plane " << i;
            pclWidget->vis->removePointCloud(ss.str());
            if(i == index){
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
                cloud = segmentedPlanes.at(index).makeShared();
                displayCloud(cloud, displayRGBCloud, ss.str());
            }
        }
    }
}

void MainWindow::on_btnVoxelizePre_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    if(line2double(*ui->lineVoxRes) > 0.0)
        this->applyVoxelization(cloud);

    displayCloud(cloud, displayRGBCloud);
}

void MainWindow::on_btnVoxelize_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    if(line2double(*ui->lineVoxRes) > 0.0)
        this->applyVoxelization(cloud);

    m_pc = cloud->makeShared();
    displayCloud(m_pc, displayRGBCloud);
}

void MainWindow::on_btnFilterPlanePre_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    this->applyHull(cloud);
    displayCloud(cloud, displayRGBCloud);
}

void MainWindow::on_btnFilterPlane_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    this->applyHull(cloud);
    m_pc = cloud->makeShared();
    displayCloud(m_pc, displayRGBCloud);
}

void MainWindow::on_btnCreateProjImage_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    this->applyTransformation(cloud);
    this->createProjectionImage();
}

void MainWindow::on_btnResetCalibCloud_clicked()
{
    this->m_pc = this->m_pc_bckp;
    this->displayCloud(m_pc, displayRGBCloud);
}

void MainWindow::on_btnLoadModel_clicked()
{
    string filename = ui->lineLoadModel->text().toStdString();
    double scale = line2double(*ui->lineLoadModelScale);
    cout << "Reading: " << filename << endl;

//    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
//    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();

    reader->SetFileName(filename.c_str());
    reader->Update();

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->SetScale(scale, scale, scale);
//    actor->AddPosition(2.5, 1.0, 0.0);
//    double color[3];
//    actor->GetProperty()->GetColor(color);
//    cout << "actor color is: " << color[0] << ", " << color[1] << ", " << color[2] << endl;

//    renderer->AddActor(actor);
//    renderer->SetBackground(0.3, 0.6, 0.3); // Background color green

    actorEntry entry;
    entry.actor = actor;
    int cnt = 1;
    bool nameIsValid = false;
    stringstream newName;
    newName << filename;
    if(!modelVec.empty()){
        while(!nameIsValid) {
            for(int i = 0; i < modelVec.size(); i++) {
                string currentID = modelVec.at(i).id;
                if(!strcmp(currentID.c_str(), newName.str().c_str())) {
                    newName.str(std::string());;
                    newName << filename << " (" << cnt << ")";
                    break;
                }else if(i==modelVec.size()-1) {
                    nameIsValid = true;
                }
            }
            cnt++;
        }
    }else {
        nameIsValid = true;
    }
    entry.id = newName.str();
    entry.visible = true;
    entry.positionXYZ = Eigen::Vector3f(0,0,0);
    entry.orientationYPR = Eigen::Vector3f(0,0,0);
    double bounds[6];
    entry.actor->GetBounds(bounds);
    for(int i = 0; i < 6; i++){
        entry.bounds.push_back(bounds[i]);
    }
    modelVec.push_back(entry);
    pclWidget->vis->addActorToRenderer(actor);
//    getRenderWindow()->AddRenderer(renderer);
//    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    ui->qvtkWidget->update();
    this->updateModelIndex();

}

void MainWindow::updateModelIndex()
{
    ui->comboBoxModelSelect->clear();
    for(int i = 0; i< modelVec.size(); i++) {
        ui->comboBoxModelSelect->addItem(QString::fromStdString(modelVec.at(i).id));
    }
}

void MainWindow::on_btnModelShow_clicked()
{
    int id = ui->comboBoxModelSelect->currentIndex();
    if(!modelVec.empty()) {
        if(!modelVec.at(id).visible) {
            pclWidget->vis->addActorToRenderer(modelVec.at(id).actor);
            modelVec.at(id).visible = true;
            updateModelButtons();
            ui->qvtkWidget->update();
        }
    }
}

void MainWindow::on_btnModelHide_clicked()
{
    int id = ui->comboBoxModelSelect->currentIndex();
    if(!modelVec.empty()) {
        if(modelVec.at(id).visible) {
            pclWidget->vis->removeActorFromRenderer(modelVec.at(id).actor);
            modelVec.at(id).visible = false;
            updateModelButtons();
            ui->qvtkWidget->update();
        }
    }
}

void MainWindow::on_btnModelDel_clicked()
{
    int id = ui->comboBoxModelSelect->currentIndex();
    if(!modelVec.empty()) {
        pclWidget->vis->removeActorFromRenderer(modelVec.at(id).actor);
        modelVec.erase(modelVec.begin()+id);
        this->updateModelIndex();
        ui->qvtkWidget->update();
    }
}

void MainWindow::updateModelButtons() {
    if(!modelVec.empty()) {
        if(modelVec.at(ui->comboBoxModelSelect->currentIndex()).visible) {
            ui->btnModelShow->setDisabled(true);
            ui->btnModelHide->setEnabled(true);
        }else {
            ui->btnModelShow->setEnabled(true);
            ui->btnModelHide->setDisabled(true);
        }
    }
}

void MainWindow::on_comboBoxModelSelect_currentIndexChanged(int index)
{
    if(index >= 0){
        updateModelButtons();
        setModelTransformationLines();
    }
}

void MainWindow::updatePCIndex()
{
    ui->comboBoxPCSelect->clear();
    for(int i = 0; i < PCVec.size(); i++) {
        ui->comboBoxPCSelect->addItem(QString::fromStdString(PCVec.at(i).id));
    }
}

void MainWindow::on_btnPCShow_clicked()
{
    int id = ui->comboBoxPCSelect->currentIndex();
    if(!PCVec.empty()) {
        if(!PCVec.at(id).visible) {
            displayCloud(PCVec.at(id).cloud, displayRGBCloud, PCVec.at(id).id);
            PCVec.at(id).visible = true;
            updatePCButtons();
            ui->qvtkWidget->update();
        }
    }
}

void MainWindow::on_btnPCHide_clicked()
{
    int id = ui->comboBoxPCSelect->currentIndex();
    if(!PCVec.empty()) {
        if(PCVec.at(id).visible) {
            pclWidget->vis->removePointCloud(PCVec.at(id).id);
            PCVec.at(id).visible = false;
            updatePCButtons();
            ui->qvtkWidget->update();
        }
    }
}

void MainWindow::on_btnPCDel_clicked()
{
    int id = ui->comboBoxPCSelect->currentIndex();
    if(!PCVec.empty()) {
        pclWidget->vis->removePointCloud(PCVec.at(id).id);
        PCVec.erase(PCVec.begin()+id);
        this->updatePCIndex();
        ui->qvtkWidget->update();
    }
}

void MainWindow::updatePCButtons() {
    if(!PCVec.empty()) {
        if(PCVec.at(ui->comboBoxPCSelect->currentIndex()).visible) {
            ui->btnPCShow->setDisabled(true);
            ui->btnPCHide->setEnabled(true);
        }else {
            ui->btnPCShow->setEnabled(true);
            ui->btnPCHide->setDisabled(true);
        }
    }
}

void MainWindow::on_comboBoxPCSelect_currentIndexChanged(int index)
{
    if(index >= 0){
        updatePCButtons();
        setPCTransformationLines();
    }
}

void MainWindow::on_btnModelMove_clicked()
{
    this->moveModel();
}

void MainWindow::moveModel() {
    if(!waitForLines) {
        int id = ui->comboBoxModelSelect->currentIndex();
        if(!modelVec.empty() && modelVec.size() > id) {

            if(!modelVec.at(id).visible) {
                cout << "Warning: model not visible!" << endl;
            }

            double scale[3];
            modelVec.at(id).actor->GetScale(scale);

            double position[3];
            modelVec.at(id).actor->GetPosition(position);

            float rpyIn[4], rpyOut[4];
            float diffX, diffY, diffZ;
            diffX = ui->spinBoxMoveObjRoll->value()     -   this->previousValueSpinBoxOrientation[0];
            diffY = ui->spinBoxMoveObjPitch->value()    -   this->previousValueSpinBoxOrientation[1];
            diffZ = ui->spinBoxMoveObjYaw->value()      -   this->previousValueSpinBoxOrientation[2];
            rpyIn[0] = diffX;
            rpyIn[1] = diffY;
            rpyIn[2] = diffZ;
            rpyIn[3] = 1;

            this->previousValueSpinBoxOrientation[0] = ui->spinBoxMoveObjRoll->value();
            this->previousValueSpinBoxOrientation[1] = ui->spinBoxMoveObjPitch->value();
            this->previousValueSpinBoxOrientation[2]= ui->spinBoxMoveObjYaw->value();

            cout << "RPY_I: " << rpyIn[0] << " " << rpyIn[1] << " " << rpyIn[2] << endl;

            vtkSmartPointer<vtkMatrix4x4> mat = modelVec.at(id).actor->GetMatrix();
            mat->MultiplyPoint(rpyIn, rpyOut);
            cout << "RPY_O: " << rpyOut[0] << " " << rpyOut[1] << " " << rpyOut[2] << " " << rpyOut[3] << endl;

//            vtkSmartPointer<vtkTransform> transformBB = vtkSmartPointer<vtkTransform>::New();
//            transformBB->PostMultiply();
//            transformBB->Translate(-modelVec.at(id).positionXYZ(0)/scale[0], -modelVec.at(id).positionXYZ(1)/scale[0], -modelVec.at(id).positionXYZ(2)/scale[0]);
//            transformBB->RotateWXYZ(diffX, mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
//            transformBB->RotateWXYZ(diffY, mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
//            transformBB->RotateWXYZ(diffZ, mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));
//            transformBB->Translate(ui->spinBoxMoveObjX->value()/scale[0], ui->spinBoxMoveObjY->value()/scale[0], ui->spinBoxMoveObjZ->value()/scale[0]);


            modelVec.at(id).actor->SetPosition(0, 0, 0);
            modelVec.at(id).actor->RotateWXYZ(diffX, mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
            modelVec.at(id).actor->RotateWXYZ(diffY, mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
            modelVec.at(id).actor->RotateWXYZ(diffZ, mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));
            modelVec.at(id).actor->SetPosition(ui->spinBoxMoveObjX->value(), ui->spinBoxMoveObjY->value(), ui->spinBoxMoveObjZ->value());

            mat = modelVec.at(id).actor->GetMatrix();
            cout << "Pre Actor Matrix:" << endl;
            cout << mat->Element[0][0] << "\t\t" << mat->Element[0][1] << "\t\t" << mat->Element[0][2] << "\t\t" << mat->Element[0][3] << endl;
            cout << mat->Element[1][0] << "\t\t" << mat->Element[1][1] << "\t\t" << mat->Element[1][2] << "\t\t" << mat->Element[1][3] << endl;
            cout << mat->Element[2][0] << "\t\t" << mat->Element[2][1] << "\t\t" << mat->Element[2][2] << "\t\t" << mat->Element[2][3] << endl;
            cout << mat->Element[3][0] << "\t\t" << mat->Element[3][1] << "\t\t" << mat->Element[3][2] << "\t\t" << mat->Element[3][3] << endl;


            //move the model first back to origin and then using the new input values TODO: transform in body coordinates not world coordinates
//            vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
//            transform->PostMultiply();
//            transform->Translate(-modelVec.at(id).positionXYZ(0)/scale[0], -modelVec.at(id).positionXYZ(1)/scale[0], -modelVec.at(id).positionXYZ(2)/scale[0]);
//            transform->RotateZ(-modelVec.at(id).orientationYPR(0));
//            transform->RotateY(-modelVec.at(id).orientationYPR(1));
//            transform->RotateX(-modelVec.at(id).orientationYPR(2));
//            transform->RotateX(ui->spinBoxMoveObjRoll->value());
//            transform->RotateY(ui->spinBoxMoveObjPitch->value());
//            transform->RotateZ(ui->spinBoxMoveObjYaw->value());
//            transform->Translate(ui->spinBoxMoveObjX->value()/scale[0], ui->spinBoxMoveObjY->value()/scale[0], ui->spinBoxMoveObjZ->value()/scale[0]);

//            vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//            transformFilter->SetInput(modelVec.at(id).actor->GetMapper()->GetInput());
//            transformFilter->SetTransform(transformWXYZ);
//            transformFilter->Update();

//            // Create a mapper and actor
//            vtkSmartPointer<vtkPolyDataMapper> transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//            transformedMapper->SetInputConnection(transformFilter->GetOutputPort());
//            modelVec.at(id).actor->SetMapper(transformedMapper);

            //update values to new position
            modelVec.at(id).positionXYZ = Eigen::Vector3f(ui->spinBoxMoveObjX->value(), ui->spinBoxMoveObjY->value(), ui->spinBoxMoveObjZ->value());
            double orientation[3];
            modelVec.at(id).actor->GetOrientation(orientation);
            modelVec.at(id).orientationYPR = Eigen::Vector3f(orientation[2], orientation[1], orientation[0]);
            cout << "yaw = " << orientation[2] << "\tpitch = " << orientation[1] << "\troll = " << orientation[0] << endl;
//            modelVec.at(id).orientationYPR = Eigen::Vector3f(ui->spinBoxMoveObjYaw->value(), ui->spinBoxMoveObjPitch->value(), ui->spinBoxMoveObjRoll->value());

            mat = modelVec.at(id).actor->GetMatrix();
            cout << "Post Actor Matrix:" << endl;
            cout << mat->Element[0][0] << "\t\t" << mat->Element[0][1] << "\t\t" << mat->Element[0][2] << "\t\t" << mat->Element[0][3] << endl;
            cout << mat->Element[1][0] << "\t\t" << mat->Element[1][1] << "\t\t" << mat->Element[1][2] << "\t\t" << mat->Element[1][3] << endl;
            cout << mat->Element[2][0] << "\t\t" << mat->Element[2][1] << "\t\t" << mat->Element[2][2] << "\t\t" << mat->Element[2][3] << endl;
            cout << mat->Element[3][0] << "\t\t" << mat->Element[3][1] << "\t\t" << mat->Element[3][2] << "\t\t" << mat->Element[3][3] << endl;

            ui->qvtkWidget->update();
        }

    }
}

void MainWindow::moveModel(actorEntry &entry, Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR) {
            if(!entry.visible) {
                cout << "Warning: model not visible!" << endl;
            }

            //move the model first back to origin and then using the new input values TODO: transform in body coordinates not world coordinates
            vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
            transform->PostMultiply();
            transform->Translate(-entry.positionXYZ(0), -entry.positionXYZ(1), -entry.positionXYZ(2));
            transform->RotateZ(-entry.orientationYPR(0));
            transform->RotateY(-entry.orientationYPR(1));
            transform->RotateX(-entry.orientationYPR(2));
            transform->RotateX(rotateYPR(2));
            transform->RotateY(rotateYPR(1));
            transform->RotateZ(rotateYPR(0));
            transform->Translate(translateXYZ(0), translateXYZ(1), translateXYZ(2));

            vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
            transformFilter->SetInput(entry.actor->GetMapper()->GetInput());
            transformFilter->SetTransform(transform);
            transformFilter->Update();

            // Create a mapper and actor
            vtkSmartPointer<vtkPolyDataMapper> transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            transformedMapper->SetInputConnection(transformFilter->GetOutputPort());
            entry.actor->SetMapper(transformedMapper);

            //update values to new position
            entry.positionXYZ = translateXYZ;
            entry.orientationYPR = rotateYPR;

            ui->qvtkWidget->update();
}

void MainWindow::moveModelRelative(actorEntry &entry, Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR) {


    if(!entry.visible) {
        cout << "Warning: model not visible!" << endl;
    }
    double position[3];
    entry.actor->GetPosition(position);
    cout << "Pre: position of model " << entry.id.c_str() << " is: " << "x = " << position[0] << " y = " << position[1] << " z = " << position[2] << endl;

    float rpyIn[4], rpyOut[4];
    rpyIn[0] = rotateYPR(2);
    rpyIn[1] = rotateYPR(1);
    rpyIn[2] = rotateYPR(0);
    rpyIn[3] = 1;

//    entry.orientationYPR(2) = ui->spinBoxMoveObjRoll->value();
//    entry.orientationYPR(1) = ui->spinBoxMoveObjPitch->value();
//    entry.orientationYPR(0) = ui->spinBoxMoveObjYaw->value();

//    cout << "RPY_I: " << rpyIn[0] << " " << rpyIn[1] << " " << rpyIn[2] << endl;

    vtkSmartPointer<vtkMatrix4x4> mat = entry.actor->GetMatrix();
    mat->MultiplyPoint(rpyIn, rpyOut);
//    cout << "RPY_O: " << rpyOut[0] << " " << rpyOut[1] << " " << rpyOut[2] << endl;

    entry.actor->SetPosition(0, 0, 0);
    entry.actor->RotateWXYZ(rpyIn[0], mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
    entry.actor->RotateWXYZ(rpyIn[1], mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
    entry.actor->RotateWXYZ(rpyIn[2], mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));

    double orientation[3];
    entry.actor->GetOrientation(orientation);
    entry.orientationYPR = Eigen::Vector3f(orientation[2], orientation[1], orientation[0]);

    position[0] += translateXYZ(0);
    position[1] += translateXYZ(1);
    position[2] += translateXYZ(2);
    entry.actor->SetPosition(position[0], position[1], position[2]);
    entry.positionXYZ = Eigen::Vector3f(position[0], position[1], position[2]);

    ui->qvtkWidget->update();
}

void MainWindow::on_btnPCMove_clicked()
{
    this->movePC();
}

void MainWindow::movePC() {
    if(!waitForLines){
        int id = ui->comboBoxPCSelect->currentIndex();
        if(!PCVec.empty() && PCVec.size() > id) {
            if(!PCVec.at(id).visible) {
                cout << "Warning: pointcloud not visible!" << endl;
            }

            //copy the pointcloud to perform transformations
            PointCloud<PointXYZRGB> pc;
            pc = *PCVec.at(id).cloud;

            //initialize transformation matrices
            Eigen::Matrix3f rotMat, rotMat_inv;
            Eigen::Matrix4f transform, transform_inv;
            Eigen::Matrix4f curr_inv_transform;

            //calculate and apply the inverse transformation to retrieve the original position
            setRotationMatrixFromYPR(DEG2RAD(PCVec.at(id).orientationYPR), rotMat_inv);
            setTransformationMatrix(rotMat_inv, PCVec.at(id).positionXYZ, transform_inv);
            curr_inv_transform = transform_inv.inverse();
            pcl::transformPointCloud(pc, pc, curr_inv_transform);

            //update the PC entry with the new GUI parameters
            PCVec.at(id).positionXYZ = Eigen::Vector3f(ui->spinBoxMovePCX->value(), ui->spinBoxMovePCY->value(), ui->spinBoxMovePCZ->value());
            PCVec.at(id).orientationYPR = Eigen::Vector3f(ui->spinBoxMovePCYaw->value(), ui->spinBoxMovePCPitch->value(), ui->spinBoxMovePCRoll->value());

            //calculate and apply the new transformation as specified by the GUI parameters
            setRotationMatrixFromYPR(DEG2RAD(PCVec.at(id).orientationYPR), rotMat);
            setTransformationMatrix(rotMat, PCVec.at(id).positionXYZ, transform);
            pcl::transformPointCloud(pc, pc, transform);

            //replace the original entry with the transformed pointcloud
            *PCVec.at(id).cloud = pc;

            //remove pointcloud before displaying the transformed pointcloud
            pclWidget->vis->removePointCloud(PCVec.at(id).id);
            displayCloud(PCVec.at(id).cloud, displayRGBCloud, PCVec.at(id).id);

            ui->qvtkWidget->update();
        }
    }

}

void MainWindow::setTransformationMatrix(Eigen::Matrix3f in_R, Eigen::Vector3f in_t, Eigen::Matrix4f &out_T) {
    out_T <<    in_R(0,0),  in_R(0,1),  in_R(0,2),  in_t(0),
                in_R(1,0),  in_R(1,1),  in_R(1,2),  in_t(1),
                in_R(2,0),  in_R(2,1),  in_R(2,2),  in_t(2),
                0,          0,          0,          1;
}

void MainWindow::setRotationMatrixFromYPR(float yaw, float pitch, float roll, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();
}

void MainWindow::setRotationMatrixFromYPR(Eigen::Vector3f ypr, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(ypr(0), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(ypr(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(ypr(2), Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();

//    Eigen::Quaternion<float> qi = rollAngle * pitchAngle * yawAngle;
//    cout << "ypr" << endl;
//    cout << q.matrix() << endl;
//    cout << "rpy" << endl;
//    cout << qi.matrix() << endl;
}

void MainWindow::setIdentityMatrix(Eigen::Matrix3f &mat) {
    Eigen::Matrix3f m;
    m << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    mat = m;
}

void MainWindow::setIdentityMatrix(Eigen::Matrix4f &mat) {
    Eigen::Matrix4f m;
    m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    mat = m;
}

void MainWindow::on_btnMergeClouds_clicked()
{
    if(!PCVec.empty()) {
        PointCloud<PointXYZRGB> pc_merged;
        bool init = true;
        for(size_t i = 0; i < PCVec.size(); i++) {
            if(PCVec.at(i).visible) {
                if(init) {
                    pc_merged = *PCVec.at(i).cloud;
                    init = false;
                }else {
                    pc_merged += *PCVec.at(i).cloud;
                }
            }
        }
        if(!init) {
            PCEntry entry;
            entry.cloud = pc_merged.makeShared();
            int cnt = 1;
            bool nameIsValid = false;
            stringstream newName;
            newName << "merged_cloud";
            while(!nameIsValid) {
                for(int i = 0; i < PCVec.size(); i++) {
                    string currentID = PCVec.at(i).id;
                    if(!strcmp(currentID.c_str(), newName.str().c_str())) {
                        newName.str(std::string());;
                        newName << "merged_cloud" << " (" << cnt << ")";
                        break;
                    }else if(i==PCVec.size()-1) {
                        nameIsValid = true;
                    }
                }
                cnt++;
            }
            entry.id = newName.str();
            entry.visible = true;
            entry.positionXYZ = Eigen::Vector3f(0,0,0);
            entry.orientationYPR = Eigen::Vector3f(0,0,0);
            PCVec.push_back(entry);

            this->displayCloud(PCVec.back().cloud, displayRGBCloud, PCVec.back().id);
            this->updatePCIndex();
        } else {
            cout << "No visible pointclouds to merge!" << endl;
        }
    }
}

void MainWindow::setPCTransformationLines() {
    int id = ui->comboBoxPCSelect->currentIndex();
    if(PCVec.size() > id) {
        this->waitForLines = true;
        ui->spinBoxMovePCX->setValue((double)PCVec.at(id).positionXYZ(0));
        ui->spinBoxMovePCY->setValue((double)PCVec.at(id).positionXYZ(1));
        ui->spinBoxMovePCZ->setValue((double)PCVec.at(id).positionXYZ(2));

        ui->spinBoxMovePCYaw->setValue((double)(PCVec.at(id).orientationYPR(0)));
        ui->spinBoxMovePCPitch->setValue((double)PCVec.at(id).orientationYPR(1));
        ui->spinBoxMovePCRoll->setValue((double)PCVec.at(id).orientationYPR(2));
        this->waitForLines = false;

        this->movePC();
    }
}

void MainWindow::setModelTransformationLines() {
    int id = ui->comboBoxModelSelect->currentIndex();
    if(modelVec.size() > id) {
        this->waitForLines = true;
        ui->spinBoxMoveObjX->setValue((double)modelVec.at(id).positionXYZ(0));
        ui->spinBoxMoveObjY->setValue((double)modelVec.at(id).positionXYZ(1));
        ui->spinBoxMoveObjZ->setValue((double)modelVec.at(id).positionXYZ(2));

//        ui->spinBoxMoveObjYaw->setValue((double)(modelVec.at(id).orientationYPR(0)));
//        ui->spinBoxMoveObjPitch->setValue((double)modelVec.at(id).orientationYPR(1));
//        ui->spinBoxMoveObjRoll->setValue((double)modelVec.at(id).orientationYPR(2));

        ui->spinBoxMoveObjYaw->setValue(0.0);
        ui->spinBoxMoveObjPitch->setValue(0.0);
        ui->spinBoxMoveObjRoll->setValue(0.0);
        this->previousValueSpinBoxOrientation[0] = 0.0;
        this->previousValueSpinBoxOrientation[1] = 0.0;
        this->previousValueSpinBoxOrientation[2]= 0.0;

        this->waitForLines = false;

        this->moveModel();
    }
}

void MainWindow::on_btnPCSave_clicked()
{
    int id = ui->comboBoxPCSelect->currentIndex();
    if(PCVec.size() > id) {
        PointCloud<PointXYZRGB> pc_save(*PCVec.at(id).cloud);
        stringstream filename;
        string lineTxt = ui->lineLoadPointcloud->text().toStdString();
//        filename << PCVec.at(id).id << ".pcd";
        filename << lineTxt;
        pcl::io::savePCDFileBinary(filename.str(), pc_save);
    }
}

void MainWindow::on_btnModelReset_clicked()
{
    this->resetModelPose();
}

void MainWindow::resetModelPose() {
    if(!modelVec.empty()) {
        int id = ui->comboBoxModelSelect->currentIndex();

        this->previousValueSpinBoxOrientation[0] = 0;
        this->previousValueSpinBoxOrientation[1] = 0;
        this->previousValueSpinBoxOrientation[2]= 0;

        this->waitForLines = true;
        ui->spinBoxMoveObjX->setValue(0);
        ui->spinBoxMoveObjY->setValue(0);
        ui->spinBoxMoveObjZ->setValue(0);

        ui->spinBoxMoveObjYaw->setValue(0);
        ui->spinBoxMoveObjPitch->setValue(0);
        ui->spinBoxMoveObjRoll->setValue(0);
        this->waitForLines = false;

//        this->moveModel();

        modelVec.at(id).actor->SetPosition(0, 0, 0);
        modelVec.at(id).actor->SetOrientation(0, 0, 0);

        //update values to new position
        modelVec.at(id).positionXYZ = Eigen::Vector3f(0, 0, 0);
        modelVec.at(id).orientationYPR = Eigen::Vector3f(0, 0, 0);

        ui->qvtkWidget->update();

        double o[3];
        modelVec.at(id).actor->GetOrientation(o);
//        vtkSmartPointer<vtkActor> ac;
//        ac->GetOrientation(o);
        cout << "orientation: " << o[0] << " " << o[1] << " " << o[2] << endl;

    }
}

void MainWindow::on_btnPCReset_clicked()
{
    this->resetPCPose();
}

void MainWindow::resetPCPose() {
    if(!PCVec.empty()) {
        this->waitForLines = true;
        ui->spinBoxMovePCX->setValue(0);
        ui->spinBoxMovePCY->setValue(0);
        ui->spinBoxMovePCZ->setValue(0);

        ui->spinBoxMovePCYaw->setValue(0);
        ui->spinBoxMovePCPitch->setValue(0);
        ui->spinBoxMovePCRoll->setValue(0);
        this->waitForLines = false;

        this->movePC();
    }
}

void MainWindow::sendPCToOctomapServer() {
    int id = ui->comboBoxPCSelect->currentIndex();
    if(PCVec.size() > id) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0, 0, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "object_frame"));

        PointCloud<PointXYZRGB> pc_to_octomap;

        pc_to_octomap = *PCVec.at(id).cloud;
        pc_to_octomap.header.frame_id = std::string("object_frame");

        emit signalPublishPointcloud(pc_to_octomap);
    }
}

void MainWindow::on_btnPCSendOcto_clicked()
{
    this->sendPCToOctomapServer();
}

void MainWindow::on_btnAddArrows_clicked()
{
    this->addArrowsForActor(modelVec.at(0));
}

void MainWindow::switchOperationMode(int mode){
    operationMode = mode;

    //reset actor colors
    for(size_t i = 0; i < modelVec.size(); i++) {
        modelVec.at(i).actor->GetProperty()->SetColor(1, 1, 1);
    }
//    cout << "arrowvec size: " << arrowVec.size() << endl;
    for(size_t j = 0; j < arrowVec.size(); j++) {
        arrowVec.at(j).actor->GetProperty()->SetColor(1, 1, 1);
    }

    this->currentObjectIndex = 99; //TODO: get rid of workaround
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    this->switchOperationMode(index);
}

void MainWindow::on_btnGlobalLoc_clicked()
{
    emit signalCallGlobalLoc();
}

void MainWindow::on_btnLocalLoc_clicked()
{
    emit signalCallLocalLoc();
}

void MainWindow::on_btnPauseLoc_clicked()
{
    emit signalCallPauseLoc();
}

void MainWindow::on_btnResumeLoc_clicked()
{
    emit signalCallResumeLoc();
}

void MainWindow::on_btnSetInitPose_clicked()
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x = line2double(*ui->lineInitPoseX);
    pose.pose.pose.position.y = line2double(*ui->lineInitPoseY);
    pose.pose.pose.position.z = line2double(*ui->lineInitPoseZ);

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(DEG2RAD(line2double(*ui->lineInitPoseRoll)), DEG2RAD(line2double(*ui->lineInitPosePitch)), DEG2RAD(line2double(*ui->lineInitPoseYaw)));
    pose.pose.pose.orientation.x = q.x;
    pose.pose.pose.orientation.y = q.y;
    pose.pose.pose.orientation.z = q.z;
    pose.pose.pose.orientation.w = q.w;

    pose.header.frame_id = "map";
    emit signalPublishInitialPose(pose);
}

void MainWindow::on_btnPauseVisOdom_clicked()
{
    emit signalPauseVisOdom();
    visualOdometryActive = false;
    ui->labelVisOdomStatus->setText("Off");
}

void MainWindow::on_btnResumeVisOdom_clicked()
{
    emit signalResumeVisOdom();
    visualOdometryActive = true;
    ui->labelVisOdomStatus->setText("On");
}

void MainWindow::newCam2ProjVTKTransform(Eigen::Matrix4f T) {
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_cam2projVTK = T;
}

void MainWindow::newMap2WorldVTKTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_map2worldVTK = T;
}

void MainWindow::newWorld2CamlinkVTKTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2camlinkVTK = T;
}

void MainWindow::newCamlink2CamVTKTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_camlink2camVTK = T;
}

void MainWindow::newWorld2CamVTKTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2camVTK = T;
}

void MainWindow::newVTKCamTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_VTKcam = T;
}

void MainWindow::newWorld2ProjVTKTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2projVTK = T;
    transformReady = true;
}

void MainWindow::newWorld2ProjTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2proj = T;
}

void MainWindow::newCam2ProjTransform(Eigen::Matrix4f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_cam2proj = T;
}

void MainWindow::newIntrProjVTKTransform(Eigen::Matrix3f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_intrProjVTK = T;
}

void MainWindow::newIntrProjTransform(Eigen::Matrix3f T){
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_intrProj =T;
}

void MainWindow::on_btnAddTexture_clicked()
{
    int id = ui->comboBoxModelSelect->currentIndex();

    // Read the image which will be the texture

    std::string imagename = ui->lineAddTexture->text().toStdString();

//    std::cout << "Reading image " << imagename << "..." << std::endl;

    vtkSmartPointer<vtkJPEGReader> jPEGReader = vtkSmartPointer<vtkJPEGReader>::New();

    jPEGReader->SetFileName ( imagename.c_str() );

    jPEGReader->Update();

//    std::cout << "Done" << std::endl;

    // Creating the texture

//    std::cout << "Making a texture out of the image... " << std::endl;

    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();

    texture->SetInputConnection(jPEGReader->GetOutputPort());

//    std::cout << "Done" << std::endl;

//    // Import geometry from an OBJ file
//    std::string iname = ui->lineLoadModel->text().toStdString();

//    std::cout << "Reading OBJ file " << iname << "..." << std::endl;

//    vtkOBJReader* reader = vtkOBJReader::New();

//    reader->SetFileName(iname.c_str());

//    reader->Update();

//    vtkPolyData *polyData2 = reader->GetOutput();

//    std::cout << "Obj reader = " << polyData2->GetNumberOfPoints() << std::endl;

////    std::cout << "Obj point data = " << polyData2->GetPointData()->GetNumberOfArrays() << std::endl;

////    std::cout << "Obj point data tuples = " << polyData2->GetPointData()->GetArray(0)->GetNumberOfTuples() << std::endl;

////    std::cout << "Obj point data compon = " << polyData2->GetPointData()->GetArray(0)->GetNumberOfComponents() << std::endl;

//    // Renderer

//    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

//    mapper->SetInput(polyData2);

//    vtkSmartPointer<vtkActor> texturedQuad = vtkSmartPointer<vtkActor>::New();

//    texturedQuad->SetMapper(mapper);

//    texturedQuad->SetTexture(texture);

//    pclWidget->vis->addActorToRenderer(texturedQuad);

    modelVec.at(id).actor->SetTexture(texture);

    ui->qvtkWidget->update();
}

void MainWindow::on_btnTestChess_clicked()
{
    vector<Point2f> corners;
    Mat chessimage = imread("/home/sysadm/chessboard.png", CV_LOAD_IMAGE_COLOR);
    if(findChessboardCorners(chessimage, Size(3,3), corners)) {
        drawChessboardCorners(chessimage, Size(3,3),Mat(corners), true);
    }else {
        cout << "no chessboard found" << endl;
    }
    imshow("img", chessimage);
}

void MainWindow::on_btnGetARTransform_clicked()
{
//    cout << "calling ar slot" << endl;
    emit signalGetARTransform();
}

void MainWindow::newARTransform(std::vector<geometry_msgs::TransformStamped> transforms) {
//    cout << "transform received" << endl;
    if(!transforms.empty()) {
        for(size_t i = 0; i < transforms.size(); i++) {
            if(!strcmp(transforms.at(i).child_frame_id.c_str(), "/board1")) {
                arMarker1 = transforms.at(i);
//                cout << "arMarker1" << endl;
                tf::StampedTransform t;
                tf::transformStampedMsgToTF(arMarker1, t);
                tf::Quaternion q = t.getRotation();
                double x, y, z, roll, pitch, yaw;
                x = arMarker1.transform.translation.x;
                y = arMarker1.transform.translation.y;
                z = arMarker1.transform.translation.z;
                tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
                double2line(*ui->lineARPoseX, x);
                double2line(*ui->lineARPoseY, y);
                double2line(*ui->lineARPoseZ, z);
                double2line(*ui->lineARPoseRoll, roll);
                double2line(*ui->lineARPosePitch, pitch);
                double2line(*ui->lineARPoseYaw, yaw);

                Eigen::Vector3f trans(x, y, z);
                Eigen::Matrix3f rotMat;
                setRotationMatrixFromYPR(yaw, pitch, roll, rotMat);
                setTransformationMatrix(rotMat, trans, T_cam2AR);

                Eigen::Matrix4f T_world2AR;
                Eigen::Matrix3f rotAR;

                int id = ui->comboBoxModelSelect->currentIndex();
                //TODO also actor->GetOrientation possible as input for the rotation matrix? Or even necessary?
                setRotationMatrixFromYPR(DEG2RAD(modelVec.at(id).orientationYPR), rotAR);
//                setTransformationMatrix(rotAR, modelVec.at(id).positionXYZ, T_world2AR);

                vtkSmartPointer<vtkMatrix4x4> mat = modelVec.at(id).actor->GetMatrix();
                cout << "Actor Matrix:" << endl;
                cout << mat->Element[0][0] << "\t\t" << mat->Element[0][1] << "\t\t" << mat->Element[0][2] << "\t\t" << mat->Element[0][3] << endl;
                cout << mat->Element[1][0] << "\t\t" << mat->Element[1][1] << "\t\t" << mat->Element[1][2] << "\t\t" << mat->Element[1][3] << endl;
                cout << mat->Element[2][0] << "\t\t" << mat->Element[2][1] << "\t\t" << mat->Element[2][2] << "\t\t" << mat->Element[2][3] << endl;
                cout << mat->Element[3][0] << "\t\t" << mat->Element[3][1] << "\t\t" << mat->Element[3][2] << "\t\t" << mat->Element[3][3] << endl;
                T_world2AR <<   mat->Element[0][0], mat->Element[0][1], mat->Element[0][2], mat->Element[0][3],
                                mat->Element[1][0], mat->Element[1][1], mat->Element[1][2], mat->Element[1][3],
                                mat->Element[2][0], mat->Element[2][1], mat->Element[2][2], mat->Element[2][3],
                                mat->Element[3][0], mat->Element[3][1], mat->Element[3][2], mat->Element[3][3];


                T_projInWorldFromAR = T_world2AR * T_cam2AR.inverse() * T_cam2projVTK * T_VTKcam;
                T_camInWorldFromAR = T_world2AR * T_cam2AR.inverse();
                T_camInWorldFromLoc = T_world2camVTK;
                T_camlinkInWorldFromAR = T_camInWorldFromAR * T_camlink2camVTK.inverse();
            }else if(!strcmp(transforms.at(i).child_frame_id.c_str(), "/board2")) {
                arMarker2 = transforms.at(i);
//                cout << "arMarker2" << endl;
            }
        }
    }
}

void MainWindow::on_btnTransformByAR_clicked()
{
    pclWidget->vis->setCameraParameters(T_intrProjVTK, T_projInWorldFromAR);
    ui->qvtkWidget->update();
}

void MainWindow::addCoordinateSystem(Eigen::Vector4f origin, Eigen::Vector4f x, Eigen::Vector4f y, Eigen::Vector4f z, string name) {

    //create coordinate system from the vectors
    stringstream ss;
    pcl::PointXYZ p0, p1;
    p0 = pcl::PointXYZ(origin(0), origin(1), origin(2));

    //x-axis
    ss << name << "_x";
    p1 = pcl::PointXYZ(origin(0) + x(0), origin(1) + x(1), origin(2) + x(2));
    pclWidget->vis->removeShape(ss.str());
    pclWidget->vis->addArrow<pcl::PointXYZ>(p0, p1, 1.0, 0.0, 0.0, ss.str(), 0);
    ss.str("");

    //y-axis
    ss << name << "_y";
    p1 = pcl::PointXYZ(origin(0) + y(0), origin(1) + y(1), origin(2) + y(2));
    pclWidget->vis->removeShape(ss.str());
    pclWidget->vis->addArrow<pcl::PointXYZ>(p0, p1, 0.0, 1.0, 0.0, ss.str(), 0);
    ss.str("");

    //z-axis
    ss << name << "_z";
    p1 = pcl::PointXYZ(origin(0) + z(0), origin(1) + z(1), origin(2) + z(2));
    pclWidget->vis->removeShape(ss.str());
    pclWidget->vis->addArrow<pcl::PointXYZ>(p0, p1, 0.0, 0.0, 1.0, ss.str(), 0);
    ss.str("");
}

void MainWindow::on_btnGetPoseErrorProj_clicked()
{
    tf::StampedTransform t1,t2;
    tf::transformStampedMsgToTF(arMarker1, t1);
    tf::transformStampedMsgToTF(arMarker2, t2);
    tf::Quaternion q1, q2;
    q1 = t1.getRotation();
    q2 = t2.getRotation();
    double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    tf::Matrix3x3(q1).getEulerYPR(yaw1, pitch1, roll1);
    tf::Matrix3x3(q2).getEulerYPR(yaw2, pitch2, roll2);
    double E_x, E_y, E_z, E_roll, E_pitch, E_yaw;
    E_x = arMarker1.transform.translation.x - arMarker2.transform.translation.x;
    E_y = arMarker1.transform.translation.y - arMarker2.transform.translation.y;
    E_z = arMarker1.transform.translation.z - arMarker2.transform.translation.z;
    E_roll = roll1 - roll2;
    E_pitch = pitch1 - pitch2;
    E_yaw = yaw1 - yaw2;

    double2line(*ui->lineExProj, E_x);
    double2line(*ui->lineEyProj, E_y);
    double2line(*ui->lineEzProj, E_z);
    double2line(*ui->lineErollProj, E_roll);
    double2line(*ui->lineEpitchProj, E_pitch);
    double2line(*ui->lineEyawProj, E_yaw);
}

void MainWindow::on_btnGetPoseErrorLoc_clicked()
{
    Eigen::Affine3f affineAR(T_camInWorldFromAR), affineLoc(T_camInWorldFromLoc);
    float rollAR, pitchAR, yawAR, rollLoc, pitchLoc, yawLoc;
    pcl::getEulerAngles(affineAR, rollAR, pitchAR, yawAR);          //gives angle rotations beeing executed in order yaw, pitch', roll''
    pcl::getEulerAngles(affineLoc, rollLoc, pitchLoc, yawLoc);

//    cout << "AR: Roll = " << rollAR << "\tPitch = " << pitchAR << "\tYaw = " << yawAR << endl;
//    cout << "Loc: Roll = " << rollLoc << "\tPitch = " << pitchLoc << "\tYaw = " << yawLoc << endl;

    double E_x, E_y, E_z, E_roll, E_pitch, E_yaw;
    E_x = T_camInWorldFromAR(0,3) - T_camInWorldFromLoc(0,3);
    E_y = T_camInWorldFromAR(1,3) - T_camInWorldFromLoc(1,3);
    E_z = T_camInWorldFromAR(2,3) - T_camInWorldFromLoc(2,3);
    E_roll = rollAR - rollLoc;
    E_pitch = pitchAR - pitchLoc;
    E_yaw = yawAR - yawLoc;

    double2line(*ui->lineExLoc, E_x);
    double2line(*ui->lineEyLoc, E_y);
    double2line(*ui->lineEzLoc, E_z);
    double2line(*ui->lineErollLoc, E_roll);
    double2line(*ui->lineEpitchLoc, E_pitch);
    double2line(*ui->lineEyawLoc, E_yaw);

    cout << "LocErr:" << endl << E_x << "\t" << E_y << "\t" << E_z << "\t" << E_roll << "\t" << E_pitch << "\t" << E_yaw << endl;
}

void MainWindow::on_btnSetInitPoseByAR_clicked()
{
    Eigen::Matrix3f rot_camLink;
    rot_camLink <<  T_camlinkInWorldFromAR(0,0),  T_camlinkInWorldFromAR(0,1),  T_camlinkInWorldFromAR(0,2),
                    T_camlinkInWorldFromAR(1,0),  T_camlinkInWorldFromAR(1,1),  T_camlinkInWorldFromAR(1,2),
                    T_camlinkInWorldFromAR(2,0),  T_camlinkInWorldFromAR(2,1),  T_camlinkInWorldFromAR(2,2);
    Eigen::Quaternionf q_camLink(rot_camLink);

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x =     T_camlinkInWorldFromAR(0,3);
    pose.pose.pose.position.y =     T_camlinkInWorldFromAR(1,3);
    pose.pose.pose.position.z =     T_camlinkInWorldFromAR(2,3);
    pose.pose.pose.orientation.x =  q_camLink.x();
    pose.pose.pose.orientation.y =  q_camLink.y();
    pose.pose.pose.orientation.z =  q_camLink.z();
    pose.pose.pose.orientation.w =  q_camLink.w();

    pose.header.frame_id = "map";
    emit signalPublishInitialPose(pose);
}

void MainWindow::on_btnTestMove_clicked()
{
    float x, y, z;
    x = ui->spinBoxTestMoveX->value();
    y = ui->spinBoxTestMoveY->value();
    z = ui->spinBoxTestMoveZ->value();


    //select the model
    this->currentModel = &modelVec.at(0);

    if(this->operationMode == BASIC){
//                    cout << "updated current model" << endl;
        //show the arrows for the highlighted actor
        cout << "adding arrows..." << endl;
        this->addArrowsForActor(modelVec.at(0));
        cout << "...done" << endl;
        //activate the object movement mode
        this->switchOperationMode(MOVEOBJECTS);
    }else {

        highlightActor(ui->comboBoxArrowTest->currentIndex());

        float rpyIn[4], rpyOut[4];
        float diffX, diffY, diffZ;
        diffX = ui->spinBoxTestMoveRoll->value();
        diffY = ui->spinBoxTestMovePitch->value();
        diffZ = ui->spinBoxTestMoveYaw->value();
        rpyIn[0] = diffX;
        rpyIn[1] = diffY;
        rpyIn[2] = diffZ;
        rpyIn[3] = 1;

        int id = 0;

//        cout << "RPY_I: " << rpyIn[0] << " " << rpyIn[1] << " " << rpyIn[2] << endl;

        vtkSmartPointer<vtkMatrix4x4> mat = modelVec.at(id).actor->GetMatrix();
        mat->MultiplyPoint(rpyIn, rpyOut);
//        cout << "RPY_O: " << rpyOut[0] << " " << rpyOut[1] << " " << rpyOut[2] << endl;

        Eigen::Vector3f rotation;

//        modelVec.at(id).actor->SetPosition(0, 0, 0);

        modelVec.at(id).actor->RotateWXYZ(diffX, mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
        rotation = Eigen::Vector3f(diffZ, 0, 0);
//        moveArrows(Eigen::Vector3f(0,0,0), rotation);
        modelVec.at(id).actor->RotateWXYZ(diffY, mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
        rotation = Eigen::Vector3f(0, diffY, 0);
//        moveArrows(Eigen::Vector3f(0,0,0), rotation);
        modelVec.at(id).actor->RotateWXYZ(diffZ, mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));
        rotation = Eigen::Vector3f(0, 0, diffX);
//        moveArrows(Eigen::Vector3f(0,0,0), rotation);


        //detect which arrow was clicked and create movement in the selected direction
        Eigen::Vector3f translation, moveX, moveY, moveZ;
        moveX = Eigen::Vector3f(mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
        moveY = Eigen::Vector3f(mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
        moveZ = Eigen::Vector3f(mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));

        //select direction
        translation = x*moveX + y*moveY + z*moveZ;
//                Eigen::Vector3f(x,y,z);


//        Eigen::Matrix3f rotMat;
//        setRotationMatrixFromYPR(currentModel->orientationYPR, rotMat);
//        Eigen::Vector3f newTranslation;
//        newTranslation = rotMat * translation;

//        moveArrows(translation, Eigen::Vector3f(0,0,0));
//        moveArrows(newTranslation, Eigen::Vector3f(0,0,0));

//        vtkSmartPointer<vtkMatrix4x4> mat = currentModel->actor->GetMatrix();
//        cout << "Actor Matrix:" << endl;
//        cout << mat->Element[0][0] << "\t" << mat->Element[0][1] << "\t" << mat->Element[0][2] << "\t" << mat->Element[0][3] << endl;
//        cout << mat->Element[1][0] << "\t" << mat->Element[1][1] << "\t" << mat->Element[1][2] << "\t" << mat->Element[1][3] << endl;
//        cout << mat->Element[2][0] << "\t" << mat->Element[2][1] << "\t" << mat->Element[2][2] << "\t" << mat->Element[2][3] << endl;
//        cout << mat->Element[3][0] << "\t" << mat->Element[3][1] << "\t" << mat->Element[3][2] << "\t" << mat->Element[3][3] << endl;


//        translation += currentModel->positionXYZ;
//        modelVec.at(id).actor
        moveModelRelative(modelVec.at(id), translation, Eigen::Vector3f(0,0,0));
        this->addArrowsForActor(modelVec.at(id));

        ui->spinBoxTestMoveX->setValue(0.0);
        ui->spinBoxTestMoveY->setValue(0.0);
        ui->spinBoxTestMoveZ->setValue(0.0);
        ui->spinBoxTestMoveRoll->setValue(0.0);
        ui->spinBoxTestMovePitch->setValue(0.0);
        ui->spinBoxTestMoveYaw->setValue(0.0);

        double orientation[3];
        modelVec.at(id).actor->GetOrientation(orientation); //returned as x,y,z; order to achieve rotation is to be performed as z,x,y
        modelVec.at(id).orientationYPR(2) = orientation[0];
        modelVec.at(id).orientationYPR(1) = orientation[1];
        modelVec.at(id).orientationYPR(0) = orientation[2];
    }
    ui->qvtkWidget->update();
}

void MainWindow::slotPoseRMS(float rmsVal) {
    currRMSVal = rmsVal;
    lastLocTime = ros::Time::now();
//    ROS_INFO("NewRMS val received %f", currRMSVal);
}

void MainWindow::on_btnPublishImage_clicked()
{
    emit signalProjectImage(this->projectorImage);
}

