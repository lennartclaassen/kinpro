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
using namespace rapidxml;

/**
 * @brief VTKPointCloudWidget::VTKPointCloudWidget  Constructor of the vtkPCLWidget
 * @param parent
 */
VTKPointCloudWidget::VTKPointCloudWidget(QWidget *parent) : QVTKWidget(parent)
{
    vis = new visualization::PCLVisualizer("vis", false);
//    vis->setBackgroundColor(0.7, 0.7, 0.7);
    vis->setBackgroundColor(0,0,0);
}

/**
 * @brief VTKPointCloudWidget::~VTKPointCloudWidget     Destructor of the vtkPCLWidget
 */
VTKPointCloudWidget::~VTKPointCloudWidget() {
}

/**
 * @brief MainWindow::MainWindow    The GUI Main Window
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

    //init UI
    ui = new Ui::MainWindow();
    ui->setupUi(this);

    //init visualization widget
    pclWidget = new VTKPointCloudWidget();
    ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());

    //create empty mat as projector image
    projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);


    //set initial bool values
    this->displayRGBCloud = true;
    this->waitForLines = false;
    this->drawClickingCircle = false;
    timerRunning = false;
    this->visualOdometryActive = true;
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

    //define the number of arrows to modify the actors  TODO: rosparam
    noOfArrows = 4;

    //create an actor element for every arrow
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
    }

    //initialize laser point
    this->laserPoint = Point(0.0, 0.0);

    //initialise values for clicking detection
    this->currentObjectIndex = 99;  //TODO: get rid of workaround
    this->selectionDuration = ros::Duration(0);
    this->idleDuration = ros::Duration(0);
    this->selection_thresh = ros::Duration(2);

    //set current operation mode to BASIC
    this->operationMode = BASIC;

    //toggle checkbox to insert coordinate system
    ui->checkBoxCoordSys->toggle();

    //initilaize last selection time
    this->lastSelectionTime = ros::Time::now();

    //TODO: just for the GUI, has to be reworked!
    this->previousValueSpinBoxOrientation[0] = 0;
    this->previousValueSpinBoxOrientation[1] = 0;
    this->previousValueSpinBoxOrientation[2] = 0;

    //initialize current root mean square value
    currRMSVal = 1.0;

    //set the resource directory TODO: rosparam?
    resourceDir = (ros::package::getPath("kinpro") + "/resources/");
}

/**
 * @brief MainWindow::~MainWindow
 */
MainWindow::~MainWindow() {
}

/**
 * @brief MainWindow::transformationProcessingReady     SLOT to be called if the transformation processing thread is ready
 */
void MainWindow::transformationProcessingReady() {
    emit setTransformations(*ui);
}

/**
 * @brief MainWindow::timerCallback     Timer callback if user selection is inactive
 */
void MainWindow::timerCallback() {
//    cout << "timer timed out" << endl;
    if(operationMode == MOVEOBJECTS) {
        for(size_t k = 0; k < arrowVec.size(); k++) {
            removeArrow(k);
        }
        switchOperationMode(BASIC);
    }
    timer.stop();
    this->timerRunning = false;
}

/**
 * @brief MainWindow::newTransform      SLOT to be called if new transformation is available
 */
void MainWindow::newTransform() {
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    //check if pose receiving is activated and the transformations have been set
    if(ui->checkBoxUsePosSig->isChecked() && transformReady) {

        //set the camera parameters to generate the projector perspective
        pclWidget->vis->setCameraParameters(T_intrProjVTK, T_world2projVTK);
        ui->qvtkWidget->update();

        //create the projection image from the GUI visualization
        this->createProjectionImageFromGUI();

        //show image if desired
        if(ui->checkBoxShowProjImage->isChecked())
            showProjectionImage();

        //publish image as ROS message if desired
        if(ui->checkBoxPubImage->isChecked())
            emit signalProjectImage(this->projectorImage);

        //make sure the next pose signal was transformed correctly before processing it
        transformReady = false;
    }
}

/**
 * @brief MainWindow::newLine       SLOT to be called if new line is received
 * @param line
 */
//void MainWindow::newLine(kinpro_interaction::line line) {
//    boost::lock_guard<boost::mutex> guard(m_lineMutex);

//    //check if the interaction mode is activated
//    if(ui->checkBoxActivateInteraction->isChecked()){

//        //start the time to recognize if the user interaction has ended
//        if(!timerRunning) {
////            cout << "starting timer" << endl;
//            timer.start(5000);
//            timerRunning = true;
//        }

//        //check if the received line is empty
//        bool lineEmpty = true;
//        if(line.end.z > 0.0) {
//            lineEmpty = false;
//            timer.stop();
//            timerRunning = false;
//        }

//        //clear the line and arrow actors
//        pclWidget->vis->removeActorFromRenderer(m_lineActor);
//        pclWidget->vis->removeActorFromRenderer(obbTreeActor);
//        pclWidget->vis->removeActorFromRenderer(bspTreeActor);

//        //transform the line points from camera into world coordinates
//        Eigen::Vector4f start_cam, end_cam, start_world, end_world;
//        start_cam << line.start.x, line.start.y, line.start.z, 1.0;
//        end_cam << line.end.x, line.end.y, line.end.z, 1.0;
//        this->transformLineToWorld(start_cam, end_cam, start_world, end_world);

//        //visualize lines?
//        if(ui->checkBoxShowLine->isChecked()) {
//            //visualize line
//            this->visualizeLine(start_world, end_world);
//        }

//        //clear spheres
//        this->removeAllSpheres();

//        //activate bounding boxes for intersection determination
//        if(ui->checkBoxActivateBB->isChecked()) {

//            if(!lineEmpty) {
//                //calculate intersections of line with models
//                vector<Eigen::Vector3f> intersections;
//                vector<int> ids;
//                this->intersectLineWithModels(start_world, end_world, intersections, ids);

//                //check if intersections were found TODO: handle cases of multiple objects
//                if(!intersections.empty()) {

//                    this->lastSelectionTime = ros::Time::now();

//                    //draw the intersection points (laserpointer) and highlight the models
//                    for(size_t i = 0; i<intersections.size(); i++) {
//                        //create spheres (laser pointer) for the intersections
//                        stringstream sphereID;
//                        sphereID << "Sphere " << i;
//                        this->addSphere(intersections.at(i), sphereID.str());

//                        //create laser points to visualize in projection image TODO: decide how to determine the most suitable intersection point if there are multiple
//                        projectWorldPointToProjectorImage(intersections.at(i), this->laserPoint);

//                        //highlight the intersected models
//                        for(size_t j=0; j<ids.size(); j++) {
//                            this->highlightActor(ids.at(j));
//                        }
//                    }

//                    //check if a "click" happened on the selected object
//                    if(this->checkForClick(ids.at(0))){

//                        if(this->operationMode == BASIC){
//                            //select the model
//                            this->currentModel = &modelVec.at(ids.at(0));
////                            cout << "updated current model" << endl;
//                            //show the arrows for the highlighted actor
////                            cout << "adding arrows..." << endl;
//                            this->addArrowsForActor(modelVec.at(ids.at(0)));
////                            cout << "...done" << endl;
//                            //activate the object movement mode
//                            this->switchOperationMode(MOVEOBJECTS);
//                        }else {
//                            //detect which arrow was clicked and create movement in the selected direction TODO: rotation
//                            Eigen::Vector3f translation;
//                            float x, y, z;
//                            x = y = z = 0;

//                            //select direction
//                            switch (ids.at(0)) {
//                            case 0:
//                                x = 0.1;
//                                break;
//                            case 1:
//                                x = -0.1;
//                                break;
//                            case 2:
//                                y = 0.1;
//                                break;
//                            case 3:
//                                y = -0.1;
//                                break;
//                            case 4:
//                                z = 0.1;
//                                break;
//                            case 5:
//                                z = -0.1;
//                                break;
//                            default:
//                                break;
//                            }

//                            vtkSmartPointer<vtkMatrix4x4> mat = currentModel->actor->GetMatrix();

//                            //detect which arrow was clicked and create movement in the selected direction
//                            Eigen::Vector3f moveX, moveY, moveZ;
//                            moveX = Eigen::Vector3f(mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
//                            moveY = Eigen::Vector3f(mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
//                            moveZ = Eigen::Vector3f(mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));

//                            //select direction
//                            translation = x*moveX + y*moveY + z*moveZ;
//                            moveArrows(translation, Eigen::Vector3f(0,0,0));

//                            //move the actor model relative to its current position
//                            moveModelRelative(*currentModel, translation, Eigen::Vector3f(0,0,0));
//                        }
//                    }
//                } else {
//                    //reset clicking Times
//                    this->selectionBegin = ros::Time::now();
//                    this->selectionDuration = ros::Duration(0);

//                    //stop drawing the clicking circle
//                    this->drawClickingCircle = false;

//                    //set point coordinates to zero to use if no intersections were found
//                    this->laserPoint = Point(0.0, 0.0);

//                    if(end_world(0) > 0.0 && end_world(1) > 0.0 && end_world(2) > 0.0) {
//                        stringstream sphereID;
//                        sphereID << "Sphere end";
//                        Eigen::Vector3f lineEnd(end_world(0), end_world(1), end_world(2));
//                        this->addSphere(lineEnd, sphereID.str());
//                    }
//                }
//            }
//        }

//        //update the visualization area
//        ui->qvtkWidget->update();
//    }
//}

/**
 * @brief MainWindow::moveArrows    Move the arrow actors
 * @param translateXYZ      translation vector
 * @param rotateYPR         orientation vector
 */
void MainWindow::moveArrows(Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR) {
    Eigen::Vector3f translation, rotation;
    for(size_t i = 0; i < arrowVec.size(); i++) {
        translation = arrowVec.at(i).positionXYZ + translateXYZ;
        rotation = arrowVec.at(i).orientationYPR + rotateYPR;
        moveModelRelative(arrowVec.at(i), translateXYZ, rotateYPR);
    }
}

/**
 * @brief MainWindow::transformLineToWorld      Transform the received line from camera into world coordinates
 * @param pt_start          start point of the line in camera coordinates
 * @param pt_end            end point of the line in camera coordinates
 * @param pt_start_world    start point of the line in world coordinates
 * @param pt_end_world      end point of the line in world coordinates
 */
void MainWindow::transformLineToWorld(Eigen::Vector4f &pt_start, Eigen::Vector4f &pt_end, Eigen::Vector4f &pt_start_world, Eigen::Vector4f &pt_end_world) {
    //transform the start and end point of the line into the world coordinates
    pt_start_world = T_world2camVTK * pt_start;
    pt_end_world = T_world2camVTK * pt_end;
}

/**
 * @brief MainWindow::visualizeLine     Visualize the line inside the GUI
 * @param start         start point of the line
 * @param end           end point of the line
 */
void MainWindow::visualizeLine(Eigen::Vector4f &start, Eigen::Vector4f &end) {
    double startPt[3], endPt[3];

    //normalize the homogeneous coordinates to w=1
    startPt[0] = start(0)/start(3);
    startPt[1] = start(1)/start(3);
    startPt[2] = start(2)/start(3);

    //normalize the homogeneous coordinates to w=1
    endPt[0] = end(0)/end(3);
    endPt[1] = end(1)/end(3);
    endPt[2] = end(2)/end(3);

    //create a VTK line source
    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(startPt);
    lineSource->SetPoint2(endPt);
    lineSource->Update();

    //create a VTK mapper for the line to modify the color
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(lineSource->GetOutputPort());
    m_lineActor->SetMapper(mapper);
    m_lineActor->GetProperty()->SetColor(1.0,1.0,1.0);

    //add the line to the visualization
    pclWidget->vis->addActorToRenderer(m_lineActor);
}

/**
 * @brief MainWindow::checkForClick     Determine if a click was executed on a certain object
 * @param id    id of the evaluated object
 * @return
 */
bool MainWindow::checkForClick(int id) {
    //check if the selected object is the previously selected object
    if(this->currentObjectIndex == id){
        //increment the selection duration
        this->selectionDuration = ros::Time::now() - this->selectionBegin;

        //draw timing circle onto projector image
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

/**
 * @brief MainWindow::addSphere     Add a sphere to the visualization
 * @param center        center point of the sphere
 * @param id            id of the sphere (std::string)
 */
void MainWindow::addSphere(Eigen::Vector3f &center, string id) {
    if(!pclWidget->vis->addSphere(pcl::PointXYZ(center(0), center(1), center(2)), 0.01, 0.0, 1.0, 0.0, id))
        pclWidget->vis->updateSphere(pcl::PointXYZ(center(0), center(1), center(2)), 0.01, 0.0, 1.0, 0.0, id);
    sphereIDs.push_back(id);                //store sphere names for later removal
}

/**
 * @brief MainWindow::removeSphere  Remove a sphere from the visualization
 * @param id            id of the sphere
 */
void MainWindow::removeSphere(string &id) {
    if(pclWidget->vis->removeShape(id)) {
        cout << "Sphere with ID " << id << " removed." << endl;
    } else {
        cout << "Error: Sphere with ID " << id << " could not be removed. Does it exist?" << endl;
    }

}

/**
 * @brief MainWindow::removeAllSpheres  Remove all spheres from the visualization
 */
void MainWindow::removeAllSpheres() {
    for(size_t i = 0; i<sphereIDs.size(); i++) {
        pclWidget->vis->removeShape(sphereIDs.at(i));
    }
    sphereIDs.clear();
    cout << "All spheres removed succesfully." << endl;
}

/**
 * @brief MainWindow::addArrow      Add an arrow to the visualization
 * @param center        center of the arrow
 * @param axis          direction  of the arrow
 * @param length        length of the arrow
 * @param radius        radius of the arrow
 * @param resolution    resolution of the arrow
 * @param id            id of the arrow
 */
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

    //update the GUI
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::removeArrow   Remove an arrow from the visualization
 * @param id        id of the arrow
 */
void MainWindow::removeArrow(int id) {
        pclWidget->vis->removeActorFromRenderer(arrowVec.at(id).actor);
}

/**
 * @brief MainWindow::removeAllArrows   Remove all arrows from the visualization
 */
void MainWindow::removeAllArrows() {
    for(int i = 0; i<arrowVec.size(); i++){
        pclWidget->vis->removeActorFromRenderer(arrowVec.at(i).actor);
    }
}

/**
 * @brief MainWindow::addArrowsForActor     Add arrows in relation to a VTK actor
 * @param actor     the VTK actor for which the arrows should be added
 */
void MainWindow::addArrowsForActor(actorEntry &actor) {
    Eigen::Vector3f center, axis;
    float length, radius, resolution;
    resolution = 50;

    //get the transformation matrix of the actor
    vtkSmartPointer<vtkMatrix4x4> mat = actor.actor->GetMatrix();

    //get actor size regarding all 3 dimensions
    double position[3];
    actor.actor->GetPosition(position);

    //get the bounding box of the actor
    double bounds[6];
    actor.actor->GetBounds(bounds);

    //set the length and radius of the arrows depending on the actor size
    length = 0;
    for(int i = 0; i < actor.bounds.size(); i+=2) {
        double diff = abs(actor.bounds[i]-actor.bounds[i+1]);
        cout << "diff: " << diff << endl;
        if(diff/2 > length)
            length = diff/2;
    }
    radius = length/2;

    //define the arrow positions and orientations depending on the actor
    for(size_t i = 0; i < arrowVec.size(); i++) {

        int col = i/2;
        axis << mat->GetElement(0,col) , mat->GetElement(1,col), mat->GetElement(2,col);
        center << (bounds[0]+bounds[1])/2, (bounds[2]+bounds[3])/2, (bounds[4]+bounds[5])/2;

        if(i%2){
            axis *= (-abs(actor.bounds[i]-actor.bounds[i-1])/2-length);
        }else {
            axis *= (+abs(actor.bounds[i]-actor.bounds[i+1])/2+length);
        }

        center += axis;
//        cout << "Center: " << center(0) << "\t" << center(1) << "\t" << center(2) <<  "\t" << "(" << i << ")" << endl;

        //add arrows for all dimensions
        this->addArrow(center, axis, length, radius, resolution, i);
    }
}

/**
 * @brief MainWindow::highlightActor    Highlight an actor
 * @param id        id of the actor
 */
void MainWindow::highlightActor(int id) {
    //highlight an actor by changing the texture color
    if(operationMode == BASIC) {
        //in BASIC mode, the model objects are being selected
        modelVec.at(id).actor->GetProperty()->SetColor(0.87, 0.898, 0.7);
    }else if (operationMode == MOVEOBJECTS) {
        //in MOVEOBJECTS mode, the arrows are being selected
        arrowVec.at(id).actor->GetProperty()->SetColor(0.87, 0.898, 0.7);
    }
}

/**
 * @brief MainWindow::intersectLineWithModels       Intersect a line with the model objects
 * @param start             start point of the line
 * @param end               end point of the line
 * @param intersections     vector of the detected intersections
 * @param ids               ids of the detected intersection objects
 */
void MainWindow::intersectLineWithModels(Eigen::Vector4f& start, Eigen::Vector4f& end, std::vector<Eigen::Vector3f>& intersections, std::vector<int> &ids) {

    //intersect the line with the model objects in BASIC mode or the arrow objects in MOVEOBJECTS mode
    if(operationMode == BASIC) {
        //check if the vector is empty before proceeding
        if(!modelVec.empty()) {
            //calculate intersections with all models
            for(size_t cnt = 0; cnt < modelVec.size(); cnt++) {

                //only iterate over visible models
                if(modelVec.at(cnt).visible) {

                    //reset model color
                    modelVec.at(cnt).actor->GetProperty()->SetColor(1, 1, 1);

                    //normalize the homogeneous line coordinates to w=1
                    double pt_start[3], pt_end[3];
                    pt_start[0] = start(0)/start(3);
                    pt_start[1] = start(1)/start(3);
                    pt_start[2] = start(2)/start(3);
                    pt_end[0] = end(0)/end(3);
                    pt_end[1] = end(1)/end(3);
                    pt_end[2] = end(2)/end(3);

                    //create a copy of the actor
                    vtkSmartPointer<vtkActor> intersectionActor = vtkSmartPointer<vtkActor>::New();
                    intersectionActor->ShallowCopy(modelVec.at(cnt).actor);

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

                    //check if obbTree model is to be used
                    if(ui->radioButtonOBB->isChecked()) {
                        //create obbTree
                        obbTree->SetDataSet(intersectionActor->GetMapper()->GetInput());
                        obbTree->BuildLocator();

                        //Visualize obbTree bounding box if desired
                        if(ui->checkBoxShowBB->isChecked()) {
                            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
                            obbTree->GenerateRepresentation(0, polydata);
                            vtkSmartPointer<vtkPolyDataMapper> obbtreeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                            obbtreeMapper->SetInput(polydata);
                            obbTreeActor->SetMapper(obbtreeMapper);
                            pclWidget->vis->addActorToRenderer(obbTreeActor);
                        }

                        //calculate intersection
                        vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
                        int obbHit = obbTree->IntersectWithLine(pt_start, pt_end, intersectPoints, NULL);

                        //print and save intersection points
                        if(obbHit) {
//                            cout << "Hit! (" << obbHit << ")" << endl << "Model " << modelVec.at(cnt).id << endl;
                            ids.push_back(cnt);
                            double intersection[3];
                            Eigen::Vector3f intersectionPoint;
                            for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
                            {
                                intersectPoints->GetPoint(i, intersection);
//                                cout << "Intersection " << i << ": " << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << endl;
                                intersectionPoint << intersection[0], intersection[1], intersection[2];
                                intersections.push_back(intersectionPoint);
                            }
                        }
                    }

                    //check if bspTree model is to be used
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
                        double t;                   //parametric coordinate of intersection (0 (corresponding to pt_start) to 1 (corresponding to pt_end))
                        double x[3];                //the coordinate of the intersection
                        double pcoords[3];
                        int subId;
                        int bspHit = bspTree->IntersectWithLine(pt_start, pt_end, tolerance, t, x, pcoords, subId);

                        //print and save intersection points
                        if(bspHit) {
//                            cout << "Hit! (" << bspHit << ")" << endl << "Model " << modelVec.at(cnt).id << endl;
                            ids.push_back(cnt);
                            Eigen::Vector3f intersectionPoint;
//                            cout << "Intersection: " << x[0] << ", " << x[1] << ", " << x[2] << endl;
                            intersectionPoint << x[0], x[1], x[2];
                            intersections.push_back(intersectionPoint);
                        }
                    }
                }
            }
        }
    } else if(operationMode == MOVEOBJECTS) {
        //check if the vector is empty before proceeding
        if(!arrowVec.empty()) {
            //calculate intersections with all arrows
            for(size_t cnt = 0; cnt < arrowVec.size(); cnt++) {

                //reset model color
                arrowVec.at(cnt).actor->GetProperty()->SetColor(1, 1, 1);

                //normalize the homogeneous line coordinates to w=1
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

                    //calculate intersection
                    vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
                    int obbHit = obbTree->IntersectWithLine(pt_start, pt_end, intersectPoints, NULL);

                    //print and save intersection points
                    if(obbHit) {
//                            cout << "Hit! (" << obbHit << ")" << endl << "Model " << modelVec.at(cnt).id << endl;
                        stringstream id;
                        id << cnt;
                        ids.push_back(cnt);
                        Eigen::Vector3f intersectionPoint;
                        double intersection[3];
                        for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
                        {
                            intersectPoints->GetPoint(i, intersection);
//                            cout << "Intersection " << i << ": " << intersection[0] << ", " << intersection[1] << ", " << intersection[2] << endl;
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
                        Eigen::Vector3f intersectionPoint;
                        //                        std::cout << "Intersection: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
                        intersectionPoint << x[0], x[1], x[2];
                        intersections.push_back(intersectionPoint);
                    }
                }
            }
        }
    }
}

/**
 * @brief MainWindow::projectWorldPointToProjectorImage     Map a point in world coordinates to the projector image plane
 * @param pt_world          point in world coordinates (input)
 * @param pt_projector      point in projector image coordinates (output)
 */
void MainWindow::projectWorldPointToProjectorImage(Eigen::Vector3f &pt_world, cv::Point &pt_projector) {
    Eigen::Vector4f p_proj3D_hom, worldPoint_hom;
    Eigen::Vector3f p_proj3D, p_proj2D_hom;
    Eigen::Vector2f p_proj2D;

    //create homogeneous coordinates from the world point
    worldPoint_hom << pt_world(0), pt_world(1), pt_world(2), 1.0;

    //transform 3D world coordinates to 3D projector coordinates
    p_proj3D_hom = T_world2proj.inverse() * worldPoint_hom;
    p_proj3D << p_proj3D_hom(0), p_proj3D_hom(1), p_proj3D_hom(2);

    //transform 3D projector coordinates to 2D projector pixel values
    p_proj2D_hom = T_intrProj * p_proj3D;

    //normalize the homogenous projector coordinates to w=1
    p_proj2D << p_proj2D_hom(0)/p_proj2D_hom(2), p_proj2D_hom(1)/p_proj2D_hom(2);

    //assign the calculated values to the 2D projector image point
    pt_projector.x  = p_proj2D(0);
    pt_projector.y  = p_proj2D(1);
}

/**
 * @brief MainWindow::loadPointCloud        Load a pointcloud into the GUI
 * @param filename      name of the pointcloud file (.pcd)
 * @param name          name of the pointcloud for the GUI
 */
void MainWindow::loadPointCloud(string filename, string name) {
    std::stringstream cloudName;
    cloudName << resourceDir << filename;
    filename = cloudName.str();

    boost::lock_guard<boost::mutex> guard(m_cloudMtx);

    //stop getting pointclouds from the kinect input stream
    ui->checkBoxKinect->setChecked(false);

    //load the pointcloud
    PointCloud<PointXYZRGB> pc_load;
    pcl::io::loadPCDFile(filename, pc_load);

    //check if the calibration mode is active
    if(ui->checkBoxCalibMode->isChecked()) {
        m_pc = pc_load.makeShared();
        m_pc_bckp = pc_load.makeShared();
        this->displayCloud(m_pc, displayRGBCloud);
    }else {
        //transform cloud from rgb frame to world frame
        if(ui->checkBoxTransformPointcloud->isChecked())
            pcl::transformPointCloud(pc_load, pc_load, T_world2camVTK);

        //create a PCEntry struct
        PCEntry entry;
        entry.cloud = pc_load.makeShared();
        int cnt = 1;
        bool nameIsValid = false;

        //check if the desired name does already exist, add index if it does
        stringstream newName;
        newName << name;
        if(!PCVec.empty()){
            while(!nameIsValid) {
                for(int i = 0; i < PCVec.size(); i++) {
                    string currentID = PCVec.at(i).id;
                    if(!strcmp(currentID.c_str(), newName.str().c_str())) {
                        newName.str(std::string());;
                        newName << name << "_(" << cnt << ")";
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

        //set the name
        entry.id = newName.str();

        //set the visibility
        entry.visible = true;

        //set the position
        entry.positionXYZ = Eigen::Vector3f(0,0,0);

        //set the orientation
        entry.orientationYPR = Eigen::Vector3f(0,0,0);

        //add entry to the PC vector
        PCVec.push_back(entry);

        //display the cloud in the GUI
        this->displayCloud(PCVec.back().cloud, displayRGBCloud, PCVec.back().id);

        //update the combobox of the GUI
        this->updatePCIndex();
    }
}

/**
 * @brief MainWindow::savePointCloud        Save a pointcloud
 * @param filename          name of the pointcloud file (.pcd)
 */
void MainWindow::savePointCloud(string filename) {
    PointCloud<PointXYZRGB> pc_save(*m_pc);
    pcl::io::savePCDFileBinary(filename, pc_save);
}

/**
 * @brief MainWindow::displayCloud      Display a pointcloud in the GUI
 * @param pc        the pointcloud
 * @param color     bolean to decide if the color should be displayed
 * @param id        id of the pointcloud for identification by the visualizer
 */
void MainWindow::displayCloud(PointCloud<PointXYZRGB>::Ptr pc, bool color, string id) { //only the pointcloud with the specified id is updated; default is "cloud"
    //check if the color should be displayed
    if(color){
        if(!pclWidget->vis->updatePointCloud(pc, id)) {
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, rgb, id);
            pclWidget->vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
//            cout << "Adding new RGB cloud" << endl;
        }
    }else {
        if(!pclWidget->vis->updatePointCloud<pcl::PointXYZRGB>(pc, id)) {      //adding <pcl::PointXYZRGB> leads to ignoring color values
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);
//            cout<< "Adding new cloud" << endl;
        }
    }

    //update the visualization
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::displayCloudSingleColor   Display a pointcloud in the GUI in a specified color
 * @param pc        the pointcloud
 * @param red       red values
 * @param green     green value
 * @param blue      blue value
 * @param id        id of the pointcloud for identification by the visualizer
 */
void MainWindow::displayCloudSingleColor(PointCloud<PointXYZRGB>::Ptr pc, float red, float green, float blue, string id) { //only the pointcloud with the specified id is updated; default is "cloud"
    if(!pclWidget->vis->updatePointCloud(pc, id)) {
        pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);
        pclWidget->vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue, id);
//        cout << "Adding new Color cloud - rgb (" << red << ", " << green << ", " << blue << ")" << endl;
    }

    //update the visualization
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::newPointCloud     SLOT to be called when a new pointcloud is received
 * @param pc
 */
void MainWindow::newPointCloud(PointCloud<PointXYZRGB> pc) {
    boost::lock_guard<boost::mutex> guard(m_cloudMtx);
    //check if the direct pointcloud subscription is active
    if(ui->checkBoxKinect->isChecked()) {
            m_pc = pc.makeShared();
            this->processCloud(m_pc);
            m_pc_bckp = pc.makeShared();
            this->displayCloud(m_pc, displayRGBCloud);
    }
}

/**
 * @brief MainWindow::processCloud      Transform the pointcloud from camera into world coordinates
 * @param cloud     the pointcloud
 */
void MainWindow::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //transform cloud from rgb_frame to world frame
    if(ui->checkBoxTransformPointcloud->isChecked())
        pcl::transformPointCloud(*cloud, *cloud, T_world2camVTK);
}

/**
 * @brief MainWindow::applyPassthrough      Apply a passthrough filter to the pointcloud
 * @param cloud     the pointcloud
 */
void MainWindow::applyPassthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //apply the passthrough filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (line2float(*ui->linePass_min), line2float(*ui->linePass_max));
    pass.filter (*cloud);
}

/**
 * @brief MainWindow::applyVoxelization     Apply a voxelization filter to the pointcloud
 * @param cloud     the pointcloud
 */
void MainWindow::applyVoxelization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //read the value from the GUI
    float resolution = line2float(*ui->lineVoxRes);

    //apply the voxelizatin filter
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (resolution, resolution, resolution);
    vg.filter (*cloud);
}

/**
 * @brief MainWindow::applySegmentation     Apply a plane segmentation to the pointcloud
 * @param cloud     the pointcloud
 */
void MainWindow::applySegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    //set the plane segmentation parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.005);
    seg.setMaxIterations (100);

    //multi plane segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_plane_proj (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_f (new pcl::PointCloud<pcl::PointXYZRGB> ());

    //remove the last segmented planes
    int vecSize = segmentedPlanes.size();
    for( int i = 0; i < vecSize; i++ ) {
        stringstream ss;
        ss << "Plane " << i;
        pclWidget->vis->removePointCloud(ss.str());
    }
    segmentedPlanes.clear();

    //segment the planes
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

        //extract the planar inliers from the cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);

        //project the points onto the plane
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.setIndices(inliers);
        proj.filter(*cloud_plane_proj);

        //store the segmented plane
        segmentedPlanes.push_back(*cloud_plane_proj);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud = *cloud_f;
    }
}

/**
 * @brief MainWindow::applyHull     Construct a hull around the pointcloud
 * @param cloud     the pointcloud
 */
void MainWindow::applyHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //check if convex or concave hull should be created
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

/**
 * @brief MainWindow::applyTransformation       Transform 3D points (camera) into 2D points (projector image) for calibration evaluation
 * @param cloud
 */
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

    //create a contour of the pixel values
    vector<Point> pVec;
    for(int i=0; i<pixVec.size(); i++) {
        int x = (int)pixVec.at(i)(0);
        int y = (int)pixVec.at(i)(1);
        if(x<848 && x>=0 && y<480 && y>=0) {
            pVec.push_back(Point(x, y));
        }
    }

    //clear the last contour and store the new one
    this->projectionContour.clear();
    this->projectionContour.push_back(pVec);
}

/**
 * @brief MainWindow::createProjectionImage     Create a projection image for calibration evaluation
 */
void MainWindow::createProjectionImage()
{
    //create an empty projection image
    this->projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);

    //fill the projection image with the calculated contours
    if(!this->projectionContour[0].empty()) {

        //create poly contours from the calculated contours
        vector<vector<Point> > contours_poly( this->projectionContour.size() );

        //create a bounding rect around the calculated contours
        vector<Rect> boundRect( this->projectionContour.size() );

        //approximate a polynogal curves for the calculated contours
        cv::approxPolyDP( Mat(this->projectionContour[0]), contours_poly[0], line2double(*ui->lineVoxRes), ui->checkBoxRect->isChecked() );

        //fill the image using the draw contours method
        if(ui->checkBoxContours->isChecked())
            cv::drawContours(this->projectorImage, this->projectionContour, 0, Scalar(0,255,0), -1);

        //fill the image using the fill poly method
        if(ui->checkBoxPoly->isChecked())
            fillPoly(this->projectorImage, contours_poly, Scalar(0,0,255));

        //fill the image with lines based on the poly contours
        if(ui->checkBoxPolyLines->isChecked()) {
            for(int i=0; i<contours_poly[0].size()-1; i++) {
                line(this->projectorImage, contours_poly[0][i], contours_poly[0][i+1], Scalar(0, 0, 255), 2);
            }
        }

        //fill the image with lines based on the original contours
        if(ui->checkBoxContoursLines->isChecked()) {
            for(int i=0; i<this->projectionContour[0].size()-1; i++) {
                line(this->projectorImage, this->projectionContour[0][i], this->projectionContour[0][i+1], Scalar(0, 255, 0), 2);
            }
        }

        //draw a bounding rect around the contours
        if(ui->checkBoxRect->isChecked()) {
            boundRect[0] = boundingRect( Mat(contours_poly[0]) );
            cv::rectangle(this->projectorImage, boundRect[0], Scalar(255,255,255), 2);
        }
    }

    //visualize the projection image if option is activated
    if(ui->checkBoxShowProjImage->isChecked())
        showProjectionImage();

    //publish the image as a ROS message if the option is activated
    if(ui->checkBoxPubImage->isChecked())
        emit signalProjectImage(this->projectorImage);
}

/**
 * @brief MainWindow::showProjectionImage       Visualize the projection image
 */
void MainWindow::showProjectionImage()
{
    cv::imshow("projected Image", this->projectorImage);
    cv::waitKey(1);
}

/**
 * @brief MainWindow::createProjectionImageFromGUI      Create a projection image from the current view of the VTK visualization widget
 */
void MainWindow::createProjectionImageFromGUI()
{
    //create an empty projection image
//    this->projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);
    //picopro modification
    this->projectorImage = cv::Mat::zeros(720, 1280, CV_8UC3);


    //get the visualization window of the GUI as an image
    vtkSmartPointer<vtkRenderWindow> renderWindow = ui->qvtkWidget->GetRenderWindow();
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput( renderWindow );

    //extract the image
    windowToImageFilter->SetInputBufferTypeToRGB();
    windowToImageFilter->Update();
    vtkImageData* vtkimage = windowToImageFilter->GetOutput();
    int dimsImage[3];
    vtkimage->GetDimensions(dimsImage);

    //create the openCV projection image from the GUI image
    try{
        cv::Mat cvImage(dimsImage[1], dimsImage[0], CV_8UC3, vtkimage->GetScalarPointer());
        cv::cvtColor( cvImage, cvImage, CV_RGB2BGR); //convert color
        cv::flip( cvImage, cvImage, 0); //align axis with visualizer
        ros::Duration t_diff = ros::Time::now()-lastLocTime;
        int border = (t_diff.toSec() > 8.0 ? 8 : 24-2*(int)(t_diff.toSec()));
//        cv::Rect roi(border,border,848-2*border,480-2*border);      //TODO projector size param
        //picopro modification
        cv::Rect roi(border,border,1280-2*border,720-2*border);      //TODO projector size param

        if(currRMSVal > 0.1)
            currRMSVal = 0.1;
        this->projectorImage.setTo(Scalar(255*(currRMSVal*10),255*(1-currRMSVal*10),0));
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

/**
 * @brief MainWindow::on_checkBoxRGBCloud_toggled
 * @param checked
 */
void MainWindow::on_checkBoxRGBCloud_toggled(bool checked)
{
    this->displayRGBCloud = checked;
}

/**
 * @brief MainWindow::on_checkBoxCoordSys_toggled
 * @param checked
 */
void MainWindow::on_checkBoxCoordSys_toggled(bool checked)
{
    if(checked){
        //add a coordinate system to the visualizer
        pclWidget->vis->addCoordinateSystem(0.5,0.0,0.0,0.0);
    }else{
        //remove the coordinate system from the visualizer
        pclWidget->vis->removeCoordinateSystem();
    }

    //update the visualizer
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::on_btnSetCamView_clicked
 */
void MainWindow::on_btnSetCamView_clicked()
{
    emit setTransformations(*ui, false);

    //set the projector perspective in the GUI
    pclWidget->vis->setCameraParameters(T_intrProjVTK, T_world2projVTK);

    //update the visualizer
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::on_btnTransformUpdate_clicked
 */
void MainWindow::on_btnTransformUpdate_clicked()
{
    //update the transformation values (using subscribed cam2proj transform)
    emit setTransformations(*ui, false);
}

/**
 * @brief MainWindow::on_btnLoadPointcloud_clicked
 */
void MainWindow::on_btnLoadPointcloud_clicked()
{
    //load a pointcloud
    string lineTxt = ui->lineLoadPointcloud->text().toStdString();
    this->loadPointCloud(lineTxt, lineTxt);
}

/**
 * @brief MainWindow::on_btnSavePointcloud_clicked
 */
void MainWindow::on_btnSavePointcloud_clicked()
{
    //save a pointcloud
    std::stringstream cloudName;
    string lineTxt = ui->lineSavePointcloud->text().toStdString();
    cloudName << resourceDir << lineTxt;
    this->savePointCloud(cloudName.str());
}

/**
 * @brief MainWindow::on_btnTransformApply_clicked
 */
void MainWindow::on_btnTransformApply_clicked()
{
    //apply the transformation values specified in the gui
    emit setTransformations(*ui);
}

/**
 * @brief MainWindow::on_btnResetIntrFoc_clicked
 */
void MainWindow::on_btnResetIntrFoc_clicked()
{
    //original transformation params from camera calibration
    ui->lineIntrinsicParamsProj_fx->setText("1515.51089");
    ui->lineIntrinsicParamsProj_fy->setText("1447.40731");
}

/**
 * @brief MainWindow::on_btnResetIntrPrinc_clicked
 */
void MainWindow::on_btnResetIntrPrinc_clicked()
{
    //modified transformation params for vtk camera positioning (projector view) - 2x principal point -> generates correct perspective but wrong image size
    ui->lineIntrinsicParamsProj_cx->setText("874.75508");
    ui->lineIntrinsicParamsProj_cy->setText("1031.11484");
}

/**
 * @brief MainWindow::on_btnResetExtrRot_clicked
 */
void MainWindow::on_btnResetExtrRot_clicked()
{
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

/**
 * @brief MainWindow::on_btnResetExtrTrans_clicked
 */
void MainWindow::on_btnResetExtrTrans_clicked()
{
    //original transformation params for vtk camera positioning
    ui->lineTransCamProjX->setText("-0.027");
    ui->lineTransCamProjY->setText("-0.049");
    ui->lineTransCamProjZ->setText("-0.020");
}

/**
 * @brief MainWindow::on_btnCreateImgFromGUI_clicked
 */
void MainWindow::on_btnCreateImgFromGUI_clicked()
{
    //setup the window
    namedWindow("projected Image", WINDOW_NORMAL);
    moveWindow("projected Image", 1920, 0);
    resizeWindow("projected Image", 1280, 720);
    setWindowProperty("projected Image", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    //create and visualize the projection image based on the perspective of the GUI visualization
    this->createProjectionImageFromGUI();
    this->showProjectionImage();
}

/**
 * @brief MainWindow::on_sliderPass_min_valueChanged
 * @param value
 */
void MainWindow::on_sliderPass_min_valueChanged(int value)
{
    //adjust the min distance line value for the passthrough filter
    double val = value / 100.0;
    double2line(*ui->linePass_min, val);
}

/**
 * @brief MainWindow::on_sliderPass_max_valueChanged
 * @param value
 */
void MainWindow::on_sliderPass_max_valueChanged(int value)
{
    //adjust the max distance line value for the passthrough filter
    double val = value / 100.0;
    double2line(*ui->linePass_max, val);
}

/**
 * @brief MainWindow::on_linePass_min_textEdited
 * @param arg1
 */
void MainWindow::on_linePass_min_textEdited(const QString &arg1)
{
    //adjust the min distance slider value for the passthrough filter
    int val = (int)(100.0 * line2double(*ui->linePass_min));
    ui->sliderPass_min->setValue(val);
}

/**
 * @brief MainWindow::on_linePass_max_textEdited
 * @param arg1
 */
void MainWindow::on_linePass_max_textEdited(const QString &arg1)
{
    //adjust the max distance slider value for the passthrough filter
    int val = (int)(100.0 * line2double(*ui->linePass_max));
    ui->sliderPass_max->setValue(val);
}

/**
 * @brief MainWindow::on_btnPassthroughPreview_clicked
 */
void MainWindow::on_btnPassthroughPreview_clicked()
{
    //stop receiving the kinect image topic
    ui->checkBoxKinect->setChecked(false);

    //apply the passthrough filter to a temporal copy of the validation pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();
    this->applyPassthrough(cloud);
    displayCloud(cloud, displayRGBCloud);
}

/**
 * @brief MainWindow::on_btnPassthroughApply_clicked
 */
void MainWindow::on_btnPassthroughApply_clicked()
{

    //stop receiving the kinect image topic
    ui->checkBoxKinect->setChecked(false);

    //apply the passthrough filter to the validation pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();
    this->applyPassthrough(cloud);
    m_pc = cloud->makeShared();
    displayCloud(m_pc, displayRGBCloud);
}

/**
 * @brief MainWindow::on_btnSegmentatePre_clicked
 */
void MainWindow::on_btnSegmentatePre_clicked()
{
    //create a preview of the plane segmentation

    //stop receiving the kinect image topic
    ui->checkBoxKinect->setChecked(false);

    //create a copy of the validation pointcloud
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

/**
 * @brief MainWindow::on_btnSegmentate_clicked
 */
void MainWindow::on_btnSegmentate_clicked()
{
    //stop receiving the kinect image topic
    ui->checkBoxKinect->setChecked(false);

    //select the segmented part of the validation pointcloud
    int id = ui->comboBoxPlanes->currentIndex();
    if(segmentedPlanes.size() > id)
        m_pc = segmentedPlanes.at(id).makeShared();
    int vecSize = segmentedPlanes.size();
    for( int i = 0; i < vecSize; i++ ) {
        stringstream ss;
        ss << "Plane " << i;
        pclWidget->vis->removePointCloud(ss.str());
    }

    //display the pointcloud
    displayCloud(m_pc, displayRGBCloud);
}

/**
 * @brief MainWindow::on_comboBoxPlanes_activated
 * @param index
 */
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

/**
 * @brief MainWindow::on_btnVoxelizePre_clicked
 */
void MainWindow::on_btnVoxelizePre_clicked()
{
    //preview of the voxelization filter

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();
    if(line2double(*ui->lineVoxRes) > 0.0)
        this->applyVoxelization(cloud);

    displayCloud(cloud, displayRGBCloud);
}

/**
 * @brief MainWindow::on_btnVoxelize_clicked
 */
void MainWindow::on_btnVoxelize_clicked()
{
    //apply the voxelization filter to the validation pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();
    if(line2double(*ui->lineVoxRes) > 0.0)
        this->applyVoxelization(cloud);

    m_pc = cloud->makeShared();
    displayCloud(m_pc, displayRGBCloud);
}

/**
 * @brief MainWindow::on_btnFilterPlanePre_clicked
 */
void MainWindow::on_btnFilterPlanePre_clicked()
{
    //preview of the hull contour around the validation pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();
    this->applyHull(cloud);

    displayCloud(cloud, displayRGBCloud);
}

/**
 * @brief MainWindow::on_btnFilterPlane_clicked
 */
void MainWindow::on_btnFilterPlane_clicked()
{
    //create a hull contour around the validation pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();
    this->applyHull(cloud);
    m_pc = cloud->makeShared();

    displayCloud(m_pc, displayRGBCloud);
}

/**
 * @brief MainWindow::on_btnCreateProjImage_clicked
 */
void MainWindow::on_btnCreateProjImage_clicked()
{
    //create a projection image of the contours to validate the camera-projector-calibration
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    this->applyTransformation(cloud);
    this->createProjectionImage();
}

/**
 * @brief MainWindow::on_btnResetCalibCloud_clicked
 */
void MainWindow::on_btnResetCalibCloud_clicked()
{
    //reset the validatin pointcloud to the original complete Pointcloud
    this->m_pc = this->m_pc_bckp;
    this->displayCloud(m_pc, displayRGBCloud);
}





/**
 * @brief MainWindow::on_btnLoadModel_clicked
 */
void MainWindow::on_btnLoadModel_clicked()
{
    //get filename
    string modelname = ui->lineLoadModel->text().toStdString();

    //load model
    loadModel(modelname);
}

/**
 * @brief MainWindow::loadModel     Load a model file into the visualizer
 * @param modelname     name of the model file (relative to the resource directory
 */
void MainWindow::loadModel(string modelname)
{
    //put complete filepath together
    string filename = resourceDir;
    filename += modelname;
    cout << "Reading: " << filename << endl;

    //initialize the object reader - just uncomment which one should be used
//    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
//    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();

    //read the file
    reader->SetFileName(filename.c_str());
    reader->Update();

    //create the actor mapper
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    //create the actor
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    //create the entry for the actor vector
    actorEntry entry;
    entry.actor = actor;
    int cnt = 1;
    bool nameIsValid = false;
    stringstream newName;
    newName << modelname;
    //check if modelname is already taken and add index if it is
    if(!modelVec.empty()){
        while(!nameIsValid) {
            for(int i = 0; i < modelVec.size(); i++) {
                string currentID = modelVec.at(i).id;
                if(!strcmp(currentID.c_str(), newName.str().c_str())) {
                    newName.str(std::string());;
                    newName << modelname << "_(" << cnt << ")";
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
    entry.texture = string("");
    double bounds[6];
    entry.actor->GetBounds(bounds);
    for(int i = 0; i < 6; i++){
        entry.bounds.push_back(bounds[i]);
    }
    modelVec.push_back(entry);

    //add the actor to the renderer
    pclWidget->vis->addActorToRenderer(actor);

    //update the GUI
    ui->qvtkWidget->update();

    //update the dropdown menu with the new mode
    this->updateModelIndex();
}

/**
 * @brief MainWindow::on_btnAddTexture_clicked
 */
void MainWindow::on_btnAddTexture_clicked()
{
    //check which model is selected
    int id = ui->comboBoxModelSelect->currentIndex();

    //read the image which will be the texture
    string texturename = ui->lineAddTexture->text().toStdString();

    //add the texture for the currently selected model
    addTexture(id, texturename);
}

/**
 * @brief MainWindow::addTexture    Add a texture to an actor
 * @param actorID           id of the actor in the actor vector
 * @param texturename       name of the texture file (relative to the resource directory
 */
void MainWindow::addTexture(int actorID, string texturename)
{
    //put together the complete file path
    string imagename = resourceDir;
    imagename += texturename;

    //read the texture
    vtkSmartPointer<vtkJPEGReader> jPEGReader = vtkSmartPointer<vtkJPEGReader>::New();
    jPEGReader->SetFileName ( imagename.c_str() );
    jPEGReader->Update();

    //creating the vtk texture
    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetInputConnection(jPEGReader->GetOutputPort());

    //set the texture
    modelVec.at(actorID).actor->SetTexture(texture);
    modelVec.at(actorID).texture = texturename;

    //update the visualizer
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::updateModelIndex      Update the indexes of the model dropdown menu
 */
void MainWindow::updateModelIndex()
{
    ui->comboBoxModelSelect->clear();
    for(int i = 0; i< modelVec.size(); i++) {
        ui->comboBoxModelSelect->addItem(QString::fromStdString(modelVec.at(i).id));
    }
}

/**
 * @brief MainWindow::on_btnModelShow_clicked
 */
void MainWindow::on_btnModelShow_clicked()
{
    //get id of currently selected model
    int id = ui->comboBoxModelSelect->currentIndex();

    //show the model in the visualizer
    showModel(id);
}

/**
 * @brief MainWindow::showModel     Show a model in the visualizer
 * @param id        id of the actor in the actor vector
 */
void MainWindow::showModel(int id){
    //check validity of the id
    if(!modelVec.empty() && modelVec.size() > id) {
        //check if actor is already visible
        if(!modelVec.at(id).visible) {
            pclWidget->vis->addActorToRenderer(modelVec.at(id).actor);
            modelVec.at(id).visible = true;
            //update the model buttons
            updateModelButtons();
            //update the GUI
            ui->qvtkWidget->update();
        }
    }
}

/**
 * @brief MainWindow::on_btnModelHide_clicked
 */
void MainWindow::on_btnModelHide_clicked()
{
    //get id of currently selected model
    int id = ui->comboBoxModelSelect->currentIndex();

    //hide the active model
    hideModel(id);
}

/**
 * @brief MainWindow::hideModel     Hide a model in the visualizer
 * @param id        id of the actor in the actor vector
 */
void MainWindow::hideModel(int id){
    //check validity of the id
    if(!modelVec.empty() && modelVec.size() > id) {
        //check if actor is already visible
        if(modelVec.at(id).visible) {
            pclWidget->vis->removeActorFromRenderer(modelVec.at(id).actor);
            modelVec.at(id).visible = false;
            //update the model buttons
            updateModelButtons();
            //update the GUI
            ui->qvtkWidget->update();
        }
    }
}

/**
 * @brief MainWindow::on_btnModelDel_clicked
 */
void MainWindow::on_btnModelDel_clicked()
{
    //get id of currently selected model
    int id = ui->comboBoxModelSelect->currentIndex();
    //make sure the actor vector isn't empty
    if(!modelVec.empty()) {
        //remove the actor from the renderer
        pclWidget->vis->removeActorFromRenderer(modelVec.at(id).actor);
        //remove the actor from the actor vector
        modelVec.erase(modelVec.begin()+id);
        //update the model dropdown menu
        this->updateModelIndex();
        //update the GUI
        ui->qvtkWidget->update();
    }
}

/**
 * @brief MainWindow::updateModelButtons    Update the model buttons depending on the models visibility
 */
void MainWindow::updateModelButtons() {
    if(!modelVec.empty()) {
        //change the model button states regarding the current visibility
        if(modelVec.at(ui->comboBoxModelSelect->currentIndex()).visible) {
            ui->btnModelShow->setDisabled(true);
            ui->btnModelHide->setEnabled(true);
        }else {
            ui->btnModelShow->setEnabled(true);
            ui->btnModelHide->setDisabled(true);
        }
    }
}

/**
 * @brief MainWindow::on_comboBoxModelSelect_currentIndexChanged
 * @param index
 */
void MainWindow::on_comboBoxModelSelect_currentIndexChanged(int index)
{
    //make sure the index is valid
    if(index >= 0){
        //update the model button states
        updateModelButtons();

        //set the lines to the current position and orientation values of the actor
        setModelTransformationLines();
    }
}

/**
 * @brief MainWindow::updatePCIndex     Update the indexes of the pointcloud dropdown menu
 */
void MainWindow::updatePCIndex()
{
    ui->comboBoxPCSelect->clear();
    for(int i = 0; i < PCVec.size(); i++) {
        ui->comboBoxPCSelect->addItem(QString::fromStdString(PCVec.at(i).id));
    }
}

/**
 * @brief MainWindow::on_btnPCShow_clicked
 */
void MainWindow::on_btnPCShow_clicked()
{
    int id = ui->comboBoxPCSelect->currentIndex();
    showPC(id);
}

/**
 * @brief MainWindow::showPC        Show a pointcloud in the visualizer
 * @param id        id of the pointcloud in the pointcloud vector
 */
void MainWindow::showPC(int id){
    //check validity of the id
    if(!PCVec.empty() && PCVec.size() > id) {
        //check if cloud is already visible
        if(!PCVec.at(id).visible) {
            displayCloud(PCVec.at(id).cloud, displayRGBCloud, PCVec.at(id).id);
            PCVec.at(id).visible = true;
            //update the pointcloud buttons
            updatePCButtons();
            //update the GUI
            ui->qvtkWidget->update();
        }
    }    
}

/**
 * @brief MainWindow::on_btnPCHide_clicked
 */
void MainWindow::on_btnPCHide_clicked()
{
    //get the current pointcloud id from the dropdown menu
    int id = ui->comboBoxPCSelect->currentIndex();

    //hide the pointcloud
    hidePC(id);
}

/**
 * @brief MainWindow::hidePC        Hide a pointcloud in the visualizer
 * @param id        id of the pointcloud in the pointcloud vector
 */
void MainWindow::hidePC(int id){
    //check validity of the id
    if(!PCVec.empty() && PCVec.size() > id) {
        //check if cloud is currently visible
        if(PCVec.at(id).visible) {
            pclWidget->vis->removePointCloud(PCVec.at(id).id);
            PCVec.at(id).visible = false;
            //update the pointcloud buttons
            updatePCButtons();
            //update the GUI
            ui->qvtkWidget->update();
        }
    }
}

/**
 * @brief MainWindow::on_btnPCDel_clicked
 */
void MainWindow::on_btnPCDel_clicked()
{
    //get the current pointcloud id from the dropdown menu
    int id = ui->comboBoxPCSelect->currentIndex();
    //make sure the pointcloud isn't empty
    if(!PCVec.empty()) {
        //remove the pointcloud from the visualizer
        pclWidget->vis->removePointCloud(PCVec.at(id).id);
        //remove the pointcloud from the pointcloud vector
        PCVec.erase(PCVec.begin()+id);
        //update the pointcloud dropdown menu
        this->updatePCIndex();
        //update the GUI
        ui->qvtkWidget->update();
    }
}

/**
 * @brief MainWindow::updatePCButtons       Update the pointcloud buttons depending on the models visibility
 */
void MainWindow::updatePCButtons() {
    if(!PCVec.empty()) {
        //change the state of the buttons regarding the current visibility status of the pointcloud
        if(PCVec.at(ui->comboBoxPCSelect->currentIndex()).visible) {
            ui->btnPCShow->setDisabled(true);
            ui->btnPCHide->setEnabled(true);
        }else {
            ui->btnPCShow->setEnabled(true);
            ui->btnPCHide->setDisabled(true);
        }
    }
}

/**
 * @brief MainWindow::on_comboBoxPCSelect_currentIndexChanged
 * @param index
 */
void MainWindow::on_comboBoxPCSelect_currentIndexChanged(int index)
{
    //check validity of the index
    if(index >= 0){
        //update the pointcloud buttons
        updatePCButtons();
        //set the transformation lines depending on the clouds position and orientation
        setPCTransformationLines();
    }
}

/**
 * @brief MainWindow::on_btnModelMove_clicked
 */
void MainWindow::on_btnModelMove_clicked()
{
    //move the model
    this->moveModel();
}

/**
 * @brief MainWindow::moveModel     Move the currently selected model by the given transformation values
 */
void MainWindow::moveModel() {
    //only move the model if all lines have been set
    if(!waitForLines) {
        //get the current model id from the dropdown menu
        int id = ui->comboBoxModelSelect->currentIndex();

        //make sure the id is valid
        if(!modelVec.empty() && modelVec.size() > id) {

            //check models visibility and give a warning if its currently hidden
            if(!modelVec.at(id).visible) {
                cout << "Warning: model not visible!" << endl;
            }

            //get the actors current position
            double position[3];
            modelVec.at(id).actor->GetPosition(position);

            //set the transformation parameters differences
            float diffX, diffY, diffZ;
            diffX = ui->spinBoxMoveObjRoll->value()     -   this->previousValueSpinBoxOrientation[0];
            diffY = ui->spinBoxMoveObjPitch->value()    -   this->previousValueSpinBoxOrientation[1];
            diffZ = ui->spinBoxMoveObjYaw->value()      -   this->previousValueSpinBoxOrientation[2];

            //update the saved values
            this->previousValueSpinBoxOrientation[0] = ui->spinBoxMoveObjRoll->value();
            this->previousValueSpinBoxOrientation[1] = ui->spinBoxMoveObjPitch->value();
            this->previousValueSpinBoxOrientation[2]= ui->spinBoxMoveObjYaw->value();

            //get the actors transformation matrix
            vtkSmartPointer<vtkMatrix4x4> mat = modelVec.at(id).actor->GetMatrix();

            //move the actor to the origin and rotate it around its current coordinate axis
            modelVec.at(id).actor->SetPosition(0, 0, 0);
            modelVec.at(id).actor->RotateWXYZ(diffX, mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
            modelVec.at(id).actor->RotateWXYZ(diffY, mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
            modelVec.at(id).actor->RotateWXYZ(diffZ, mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));
            modelVec.at(id).actor->SetPosition(ui->spinBoxMoveObjX->value(), ui->spinBoxMoveObjY->value(), ui->spinBoxMoveObjZ->value());

            //update values to new position and orientation
            modelVec.at(id).positionXYZ = Eigen::Vector3f(ui->spinBoxMoveObjX->value(), ui->spinBoxMoveObjY->value(), ui->spinBoxMoveObjZ->value());
            double orientation[3];
            modelVec.at(id).actor->GetOrientation(orientation);
//            cout << "yaw = " << orientation[2] << "\tpitch = " << orientation[1] << "\troll = " << orientation[0] << endl;
            modelVec.at(id).orientationYPR = Eigen::Vector3f(orientation[2], orientation[1], orientation[0]);

            //update the absolute orientation values
            ui->spinBoxMoveObjAbsYaw->setValue(orientation[2]);
            ui->spinBoxMoveObjAbsPitch->setValue(orientation[1]);
            ui->spinBoxMoveObjAbsRoll->setValue(orientation[0]);
//            cout << "yaw = " << orientation[2] << "\tpitch = " << orientation[1] << "\troll = " << orientation[0] << endl;

            //update the GUI
            ui->qvtkWidget->update();
        }

    }
}

/**
 * @brief MainWindow::moveModel     Move a model by given translation and orientation valuees
 * @param entry                 entry of the actor vector
 * @param translateXYZ          translation parameters
 * @param rotateYPR             rotation parameters
 */
void MainWindow::moveModel(actorEntry &entry, Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR) {
    //check visibility status of the model
    if(!entry.visible) {
        cout << "Warning: model not visible!" << endl;
    }

    //move and rotate the model first back to origin and then using the new input values
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

    //update the GUI
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::moveModelRelative     Transform a model relative to its current position and orientation
 * @param entry                 entry of the actor vector
 * @param translateXYZ          translation parameters
 * @param rotateYPR             rotation parameters
 */
void MainWindow::moveModelRelative(actorEntry &entry, Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR) {
    //check if model is visibility
    if(!entry.visible) {
        cout << "Warning: model not visible!" << endl;
    }

    //get the actors current position
    double position[3];
    entry.actor->GetPosition(position);

    //create the homogeneous rotation vector
    float rpyIn[4], rpyOut[4];
    rpyIn[0] = rotateYPR(2);
    rpyIn[1] = rotateYPR(1);
    rpyIn[2] = rotateYPR(0);
    rpyIn[3] = 1;

    //get the actors transformation matrix
    vtkSmartPointer<vtkMatrix4x4> mat = entry.actor->GetMatrix();
    mat->MultiplyPoint(rpyIn, rpyOut);

    //rotate the actor by the given values after moving it to the origin
    entry.actor->SetPosition(0, 0, 0);
    entry.actor->RotateWXYZ(rpyIn[0], mat->GetElement(0,0), mat->GetElement(1,0), mat->GetElement(2,0));
    entry.actor->RotateWXYZ(rpyIn[1], mat->GetElement(0,1), mat->GetElement(1,1), mat->GetElement(2,1));
    entry.actor->RotateWXYZ(rpyIn[2], mat->GetElement(0,2), mat->GetElement(1,2), mat->GetElement(2,2));

    double orientation[3];
    entry.actor->GetOrientation(orientation);

    //update the actors orientation entry
    entry.orientationYPR = Eigen::Vector3f(orientation[2], orientation[1], orientation[0]);

    //update the rotation values shown in the GUI
    ui->spinBoxMoveObjAbsYaw->setValue(orientation[2]);
    ui->spinBoxMoveObjAbsPitch->setValue(orientation[1]);
    ui->spinBoxMoveObjAbsRoll->setValue(orientation[0]);

    //translate the actor by the given translation values
    position[0] += translateXYZ(0);
    position[1] += translateXYZ(1);
    position[2] += translateXYZ(2);
    entry.actor->SetPosition(position[0], position[1], position[2]);

    //update the actors translation entry
    entry.positionXYZ = Eigen::Vector3f(position[0], position[1], position[2]);

    //update the GUI
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::moveModelAbsolute         Set a models orientaion given the absolute rotation values from the GUI
 */
void MainWindow::moveModelAbsolute()
{
    //get the model id
    int id = ui->comboBoxModelSelect->currentIndex();
    //check validity of the id
    if(!modelVec.empty() && modelVec.size() > id) {

        //check if model is currently visible
        if(!modelVec.at(id).visible) {
            cout << "Warning: model not visible!" << endl;
        }

        //get the rotation values from the GUI
        double diffX, diffY, diffZ;
        diffX = ui->spinBoxMoveObjAbsRoll->value();
        diffY = ui->spinBoxMoveObjAbsPitch->value();
        diffZ = ui->spinBoxMoveObjAbsYaw->value();

        //set the actors orientation
        double orientation[3];
        modelVec.at(id).actor->SetOrientation(diffX, diffY, diffZ);
        modelVec.at(id).actor->GetOrientation(orientation);
        modelVec.at(id).orientationYPR = Eigen::Vector3f(orientation[2], orientation[1], orientation[0]);

        //update the GUI
        ui->qvtkWidget->update();
    }
}

/**
 * @brief MainWindow::on_btnModelMoveAbs_clicked
 */
void MainWindow::on_btnModelMoveAbs_clicked()
{
    //move the model by the absolute rotation values given in the GUI
    moveModelAbsolute();
}

/**
 * @brief MainWindow::on_btnPCMove_clicked
 */
void MainWindow::on_btnPCMove_clicked()
{
    //get the id of the active pointcloud
    int id = ui->comboBoxPCSelect->currentIndex();

    //move the pointcloud
    this->movePC(id, ui->spinBoxMovePCX->value(), ui->spinBoxMovePCY->value(), ui->spinBoxMovePCZ->value(), ui->spinBoxMovePCYaw->value(), ui->spinBoxMovePCPitch->value(), ui->spinBoxMovePCRoll->value());
}

/**
 * @brief MainWindow::movePC        Transform the currently active pointcloud by the GUI values
 */
void MainWindow::movePC() {
    //make sure all transformation lines have been set
    if(!waitForLines){
        //get the id of the currrently selected pointcloud
        int id = ui->comboBoxPCSelect->currentIndex();
        //check validity of the id
        if(!PCVec.empty() && PCVec.size() > id) {

            //check if the pointcloud is currently visible
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

            //update the GUI
            ui->qvtkWidget->update();
        }
    }
}

/**
 * @brief MainWindow::movePC        Transform a pointcloud using specified parameters
 * @param id            id of the pointcloud in the pointcloud vector
 * @param x             translation value in x-direction
 * @param y             translation value in y-direction
 * @param z             translation value in z-direction
 * @param yaw           rotation values for the yaw angle (z-axis)
 * @param pitch         rotation values for the pitch angle (y-axis)
 * @param roll          rotation values for the roll angle (x-axis)
 */
void MainWindow::movePC(int id, double x, double y, double z, double yaw, double pitch, double roll) {
    //make sure all transformation lines have been set
    if(!waitForLines){
        //check validity of the id
        if(!PCVec.empty() && PCVec.size() > id) {

            //check if the pointcloud is currently visible
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
            PCVec.at(id).positionXYZ = Eigen::Vector3f(x, y, z);
            PCVec.at(id).orientationYPR = Eigen::Vector3f(yaw, pitch, roll);

            //calculate and apply the new transformation as specified by the GUI parameters
            setRotationMatrixFromYPR(DEG2RAD(PCVec.at(id).orientationYPR), rotMat);
            setTransformationMatrix(rotMat, PCVec.at(id).positionXYZ, transform);
            pcl::transformPointCloud(pc, pc, transform);

            //replace the original entry with the transformed pointcloud
            *PCVec.at(id).cloud = pc;

            //remove pointcloud before displaying the transformed pointcloud
            pclWidget->vis->removePointCloud(PCVec.at(id).id);
            displayCloud(PCVec.at(id).cloud, displayRGBCloud, PCVec.at(id).id);

            //update the GUI
            ui->qvtkWidget->update();
        }
    }
}

/**
 * @brief MainWindow::setTransformationMatrix       Create a transformation matrix from a given translation and rotation
 * @param in_R      rotation matrix         input (3x3)
 * @param in_t      translation vector      input (3x1)
 * @param out_T     Transformation matrix   output (4x4)
 */
void MainWindow::setTransformationMatrix(Eigen::Matrix3f in_R, Eigen::Vector3f in_t, Eigen::Matrix4f &out_T) {
    out_T <<    in_R(0,0),  in_R(0,1),  in_R(0,2),  in_t(0),
                in_R(1,0),  in_R(1,1),  in_R(1,2),  in_t(1),
                in_R(2,0),  in_R(2,1),  in_R(2,2),  in_t(2),
                0,          0,          0,          1;
}

/**
 * @brief MainWindow::setRotationMatrixFromYPR      Create a rotation matrix from yaw, pitch, roll angles
 * @param yaw       yaw angle           input
 * @param pitch     pitch angle         input
 * @param roll      roll angle          input
 * @param out_R     rotation matrix     output
 */
void MainWindow::setRotationMatrixFromYPR(float yaw, float pitch, float roll, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();
}

/**
 * @brief MainWindow::setRotationMatrixFromYPR      Create a rotation matrix from combined yaw, pitch, roll vector
 * @param ypr       vector containing the yaw, pitch and roll value     input
 * @param out_R     rotation matrix                                     output
 */
void MainWindow::setRotationMatrixFromYPR(Eigen::Vector3f ypr, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(ypr(0), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(ypr(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(ypr(2), Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();
}

/**
 * @brief MainWindow::setIdentityMatrix     Set a 3x3 identity matrix
 * @param mat
 */
void MainWindow::setIdentityMatrix(Eigen::Matrix3f &mat) {
    Eigen::Matrix3f m;
    m << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    mat = m;
}

/**
 * @brief MainWindow::setIdentityMatrix     Set a 4x4 identity matrix
 * @param mat
 */
void MainWindow::setIdentityMatrix(Eigen::Matrix4f &mat) {
    Eigen::Matrix4f m;
    m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    mat = m;
}

/**
 * @brief MainWindow::on_btnMergeClouds_clicked     Merge the currently visible pointclouds from the GUI into a new one
 */
void MainWindow::on_btnMergeClouds_clicked()
{
    //make sure pointcloud vector isn't empty
    if(!PCVec.empty()) {
        //initialize merged cloud
        PointCloud<PointXYZRGB> pc_merged;
        bool init = true;
        //add pointclouds to the merged cloud
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
        //create entry for the merged cloud in the pointcloud vector
        if(!init) {
            PCEntry entry;
            entry.cloud = pc_merged.makeShared();
            int cnt = 1;
            bool nameIsValid = false;
            stringstream newName;
            newName << "merged_cloud";
            //make sure the new name is valid
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

            //display the new cloud in the visualizer
            this->displayCloud(PCVec.back().cloud, displayRGBCloud, PCVec.back().id);

            //update the pointcloud dropdown menu
            this->updatePCIndex();
        } else {
            cout << "No visible pointclouds to merge!" << endl;
        }
    }
}

/**
 * @brief MainWindow::setPCTransformationLines      Set the GUIs transformation lines for the pointcloud
 */
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

/**
 * @brief MainWindow::setModelTransformationLines       Set the GUIs transformation lines for the model
 */
void MainWindow::setModelTransformationLines() {
    int id = ui->comboBoxModelSelect->currentIndex();
    if(modelVec.size() > id) {
        this->waitForLines = true;
        ui->spinBoxMoveObjX->setValue((double)modelVec.at(id).positionXYZ(0));
        ui->spinBoxMoveObjY->setValue((double)modelVec.at(id).positionXYZ(1));
        ui->spinBoxMoveObjZ->setValue((double)modelVec.at(id).positionXYZ(2));

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

/**
 * @brief MainWindow::on_btnPCSave_clicked      Save the Pointcloud
 */
void MainWindow::on_btnPCSave_clicked()
{
    //get the id of the currently selected pointcloud
    int id = ui->comboBoxPCSelect->currentIndex();
    if(PCVec.size() > id) {
        PointCloud<PointXYZRGB> pc_save(*PCVec.at(id).cloud);

        //set the filename
        stringstream filename;
        string lineTxt = ui->lineSavePC->text().toStdString();
        filename << resourceDir << lineTxt;

        //save the pointcloud
        pcl::io::savePCDFileBinary(filename.str(), pc_save);
    }
}

/**
 * @brief MainWindow::on_btnModelReset_clicked
 */
void MainWindow::on_btnModelReset_clicked()
{
    this->resetModelPose();
}

/**
 * @brief MainWindow::resetModelPose        Reset the model pose
 */
void MainWindow::resetModelPose() {
    if(!modelVec.empty()) {
        //get the id of the currently selected model
        int id = ui->comboBoxModelSelect->currentIndex();

        //reset the transformation lines
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
        ui->spinBoxMoveObjAbsYaw->setValue(0);
        ui->spinBoxMoveObjAbsPitch->setValue(0);
        ui->spinBoxMoveObjAbsRoll->setValue(0);

        //set position and orientation to zero
        modelVec.at(id).actor->SetPosition(0, 0, 0);
        modelVec.at(id).actor->SetOrientation(0, 0, 0);

        //update values to new position nad orientation
        modelVec.at(id).positionXYZ = Eigen::Vector3f(0, 0, 0);
        modelVec.at(id).orientationYPR = Eigen::Vector3f(0, 0, 0);

        //update the GUI
        ui->qvtkWidget->update();
    }
}

/**
 * @brief MainWindow::on_btnPCReset_clicked
 */
void MainWindow::on_btnPCReset_clicked()
{
    this->resetPCPose();
}

/**
 * @brief MainWindow::resetPCPose       Reset the pointcloud pose
 */
void MainWindow::resetPCPose() {
    if(!PCVec.empty()) {

        //reset the transformation lines
        this->waitForLines = true;
        ui->spinBoxMovePCX->setValue(0);
        ui->spinBoxMovePCY->setValue(0);
        ui->spinBoxMovePCZ->setValue(0);
        ui->spinBoxMovePCYaw->setValue(0);
        ui->spinBoxMovePCPitch->setValue(0);
        ui->spinBoxMovePCRoll->setValue(0);
        this->waitForLines = false;

        //move the pointcloud
        this->movePC();
    }
}

/**
 * @brief MainWindow::sendPCToOctomapServer     Send a pointcloud to the octomap server
 */
void MainWindow::sendPCToOctomapServer() {
    //get the id of the currently selected pointcloud
    int id = ui->comboBoxPCSelect->currentIndex();
    if(PCVec.size() > id) {
        //set the transformation message
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0, 0, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "object_frame"));

        //create the pointcloud to send
        PointCloud<PointXYZRGB> pc_to_octomap;
        pc_to_octomap = *PCVec.at(id).cloud;
        pc_to_octomap.header.frame_id = std::string("object_frame");

        emit signalPublishPointcloud(pc_to_octomap);
    }
}

/**
 * @brief MainWindow::on_btnPCSendOcto_clicked
 */
void MainWindow::on_btnPCSendOcto_clicked()
{
    //send the pointcloud to the octomap server
    this->sendPCToOctomapServer();
}

/**
 * @brief MainWindow::switchOperationMode       Change the current operation mode
 * @param mode      the new mode (BASIC or MOVEOBJECTS)
 */
void MainWindow::switchOperationMode(int mode){
    operationMode = mode;

    //reset actor colors
    for(size_t i = 0; i < modelVec.size(); i++) {
        modelVec.at(i).actor->GetProperty()->SetColor(1, 1, 1);
    }
    for(size_t j = 0; j < arrowVec.size(); j++) {
        arrowVec.at(j).actor->GetProperty()->SetColor(1, 1, 1);
    }

    this->currentObjectIndex = 99; //TODO: get rid of workaround
}

/**
 * @brief MainWindow::on_btnGlobalLoc_clicked
 */
void MainWindow::on_btnGlobalLoc_clicked()
{
    //start global localization
    emit signalCallGlobalLoc();
}

/**
 * @brief MainWindow::on_btnLocalLoc_clicked
 */
void MainWindow::on_btnLocalLoc_clicked()
{
    //start local localization
    emit signalCallLocalLoc();
}

/**
 * @brief MainWindow::on_btnPauseLoc_clicked
 */
void MainWindow::on_btnPauseLoc_clicked()
{
    //pause the global localization
    emit signalCallPauseLoc();
}

/**
 * @brief MainWindow::on_btnResumeLoc_clicked
 */
void MainWindow::on_btnResumeLoc_clicked()
{
    //resume the local localization
    emit signalCallResumeLoc();
}

/**
 * @brief MainWindow::on_btnSetInitPose_clicked     Set the inital pose in the global localization
 */
void MainWindow::on_btnSetInitPose_clicked()
{
    //create the pose message
    geometry_msgs::PoseWithCovarianceStamped pose;

    //set the position
    pose.pose.pose.position.x = line2double(*ui->lineInitPoseX);
    pose.pose.pose.position.y = line2double(*ui->lineInitPoseY);
    pose.pose.pose.position.z = line2double(*ui->lineInitPoseZ);

    //set the orientation
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(DEG2RAD(line2double(*ui->lineInitPoseRoll)), DEG2RAD(line2double(*ui->lineInitPosePitch)), DEG2RAD(line2double(*ui->lineInitPoseYaw)));
    pose.pose.pose.orientation.x = q.x;
    pose.pose.pose.orientation.y = q.y;
    pose.pose.pose.orientation.z = q.z;
    pose.pose.pose.orientation.w = q.w;

    //set the message header
    pose.header.frame_id = "map";

    //emit the signal to set initial pose
    emit signalPublishInitialPose(pose);
}

/**
 * @brief MainWindow::on_btnPauseVisOdom_clicked
 */
void MainWindow::on_btnPauseVisOdom_clicked()
{
    //pause the visual odometry
    emit signalPauseVisOdom();
    visualOdometryActive = false;
    ui->labelVisOdomStatus->setText("Off");
}

/**
 * @brief MainWindow::on_btnResumeVisOdom_clicked
 */
void MainWindow::on_btnResumeVisOdom_clicked()
{
    //resume the visual odometry
    emit signalResumeVisOdom();
    visualOdometryActive = true;
    ui->labelVisOdomStatus->setText("On");
}

/**
 * @brief MainWindow::newCam2ProjVTKTransform
 * @param T
 */
void MainWindow::newCam2ProjVTKTransform(Eigen::Matrix4f T)
{
    //set the transformation between camera and projector (VTK)
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_cam2projVTK = T;
}

/**
 * @brief MainWindow::newMap2WorldVTKTransform
 * @param T
 */
void MainWindow::newMap2WorldVTKTransform(Eigen::Matrix4f T)
{
    //set the transformation between map and world (VTK)
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_map2worldVTK = T;
}

/**
 * @brief MainWindow::newWorld2CamlinkVTKTransform
 * @param T
 */
void MainWindow::newWorld2CamlinkVTKTransform(Eigen::Matrix4f T)
{
    //set the transformation between world and camera link (VTK)
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2camlinkVTK = T;
}

/**
 * @brief MainWindow::newCamlink2CamVTKTransform
 * @param T
 */
void MainWindow::newCamlink2CamVTKTransform(Eigen::Matrix4f T)
{
    //set the transformation between camera link and camera (VTK)
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_camlink2camVTK = T;
}

/**
 * @brief MainWindow::newWorld2CamVTKTransform
 * @param T
 */
void MainWindow::newWorld2CamVTKTransform(Eigen::Matrix4f T)
{
    //set the transformation between world and camera (VTK)
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2camVTK = T;
}

/**
 * @brief MainWindow::newVTKCamTransform
 * @param T
 */
void MainWindow::newVTKCamTransform(Eigen::Matrix4f T)
{
    //set the transformation for the VTK camera
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_VTKcam = T;
}

/**
 * @brief MainWindow::newWorld2ProjVTKTransform
 * @param T
 */
void MainWindow::newWorld2ProjVTKTransform(Eigen::Matrix4f T)
{
    //set the transformation between world and projector (VTK)
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2projVTK = T;
    transformReady = true;
}

/**
 * @brief MainWindow::newWorld2ProjTransform
 * @param T
 */
void MainWindow::newWorld2ProjTransform(Eigen::Matrix4f T)
{
    //set the transformation between world and projector
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_world2proj = T;
}

/**
 * @brief MainWindow::newCam2ProjTransform
 * @param T
 */
void MainWindow::newCam2ProjTransform(Eigen::Matrix4f T)
{
    //set the transformation between camera and projector
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_cam2proj = T;
}

/**
 * @brief MainWindow::newIntrProjVTKTransform
 * @param T
 */
void MainWindow::newIntrProjVTKTransform(Eigen::Matrix3f T)
{
    //set the intrinsic transformation of the projector (VTK)
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_intrProjVTK = T;
}

/**
 * @brief MainWindow::newIntrProjTransform
 * @param T
 */
void MainWindow::newIntrProjTransform(Eigen::Matrix3f T)
{
    //set the intrinsic transformation of the projector
    boost::lock_guard<boost::mutex> guard(m_positionMutex);
    T_intrProj =T;
}

/**
 * @brief MainWindow::on_btnGetARTransform_clicked
 */
void MainWindow::on_btnGetARTransform_clicked()
{
    //get the AR transform
//    cout << "calling ar slot" << endl;
    emit signalGetARTransform();
}

/**
 * @brief MainWindow::newARTransform    SLOT receiving the AR transform
 * @param transforms
 */
void MainWindow::newARTransform(std::vector<geometry_msgs::TransformStamped> transforms) {
//    cout << "transform received" << endl;

    //check if transformation is valid
    if(!transforms.empty()) {
        //iterate over the received transformations
        for(size_t i = 0; i < transforms.size(); i++) {
            //check if id of the transform matches "/board1"
            if(!strcmp(transforms.at(i).child_frame_id.c_str(), "/board1")) {
                arMarker1 = transforms.at(i);

                //extract the transformation
                tf::StampedTransform t;
                tf::transformStampedMsgToTF(arMarker1, t);
                tf::Quaternion q = t.getRotation();
                double x, y, z, roll, pitch, yaw;
                x = arMarker1.transform.translation.x;
                y = arMarker1.transform.translation.y;
                z = arMarker1.transform.translation.z;
                tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

                //set the GUI line values
                double2line(*ui->lineARPoseX, x);
                double2line(*ui->lineARPoseY, y);
                double2line(*ui->lineARPoseZ, z);
                double2line(*ui->lineARPoseRoll, roll);
                double2line(*ui->lineARPosePitch, pitch);
                double2line(*ui->lineARPoseYaw, yaw);

                //set camera to AR transform
                Eigen::Vector3f trans(x, y, z);
                Eigen::Matrix3f rotMat;
                setRotationMatrixFromYPR(yaw, pitch, roll, rotMat);
                setTransformationMatrix(rotMat, trans, T_cam2AR);

                //get the transformation of the model
                int id = ui->comboBoxModelSelect->currentIndex();
                vtkSmartPointer<vtkMatrix4x4> mat = modelVec.at(id).actor->GetMatrix();
                Eigen::Matrix4f T_world2AR;
                T_world2AR <<   mat->Element[0][0], mat->Element[0][1], mat->Element[0][2], mat->Element[0][3],
                                mat->Element[1][0], mat->Element[1][1], mat->Element[1][2], mat->Element[1][3],
                                mat->Element[2][0], mat->Element[2][1], mat->Element[2][2], mat->Element[2][3],
                                mat->Element[3][0], mat->Element[3][1], mat->Element[3][2], mat->Element[3][3];

                //calculate the transformations
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

/**
 * @brief MainWindow::on_btnTransformByAR_clicked
 */
void MainWindow::on_btnTransformByAR_clicked()
{
    //transform the projector view given the transformation from the AR marker
    pclWidget->vis->setCameraParameters(T_intrProjVTK, T_projInWorldFromAR);
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::addCoordinateSystem       Add a coordinate system to the visualizer
 * @param origin        origin of the CS
 * @param x             x-axis of the CS
 * @param y             y-axis of the CS
 * @param z             z-axis of the CS
 * @param name          name of the CS
 */
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

/**
 * @brief MainWindow::on_btnGetPoseErrorProj_clicked
 */
void MainWindow::on_btnGetPoseErrorProj_clicked()
{
    //get the pose error between the real and projected marker planes
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

    //write the error values to the GUI lines
    double2line(*ui->lineExProj, E_x);
    double2line(*ui->lineEyProj, E_y);
    double2line(*ui->lineEzProj, E_z);
    double2line(*ui->lineErollProj, E_roll);
    double2line(*ui->lineEpitchProj, E_pitch);
    double2line(*ui->lineEyawProj, E_yaw);
}

/**
 * @brief MainWindow::on_btnGetPoseErrorLoc_clicked
 */
void MainWindow::on_btnGetPoseErrorLoc_clicked()
{
    //calculate the error between the localized pose and the detected marker field

    //get the orientation values
    Eigen::Affine3f affineAR(T_camInWorldFromAR), affineLoc(T_camInWorldFromLoc);
    float rollAR, pitchAR, yawAR, rollLoc, pitchLoc, yawLoc;
    pcl::getEulerAngles(affineAR, rollAR, pitchAR, yawAR);          //gives angle rotations beeing executed in order yaw, pitch', roll''
    pcl::getEulerAngles(affineLoc, rollLoc, pitchLoc, yawLoc);

    //calculate the error values
    double E_x, E_y, E_z, E_roll, E_pitch, E_yaw;
    E_x = T_camInWorldFromAR(0,3) - T_camInWorldFromLoc(0,3);
    E_y = T_camInWorldFromAR(1,3) - T_camInWorldFromLoc(1,3);
    E_z = T_camInWorldFromAR(2,3) - T_camInWorldFromLoc(2,3);
    E_roll = rollAR - rollLoc;
    E_pitch = pitchAR - pitchLoc;
    E_yaw = yawAR - yawLoc;

    //write the error values to the GUI lines
    double2line(*ui->lineExLoc, E_x);
    double2line(*ui->lineEyLoc, E_y);
    double2line(*ui->lineEzLoc, E_z);
    double2line(*ui->lineErollLoc, E_roll);
    double2line(*ui->lineEpitchLoc, E_pitch);
    double2line(*ui->lineEyawLoc, E_yaw);

//    cout << endl << E_x << endl << E_y << endl << E_z << endl << E_roll << endl << E_pitch << endl << E_yaw << endl;
}

/**
 * @brief MainWindow::on_btnSetInitPoseByAR_clicked
 */
void MainWindow::on_btnSetInitPoseByAR_clicked()
{
    //set the initial pose of the global localization by using the transforamtoin from the ar markers
    Eigen::Matrix3f rot_camLink;
    rot_camLink <<  T_camlinkInWorldFromAR(0,0),  T_camlinkInWorldFromAR(0,1),  T_camlinkInWorldFromAR(0,2),
                    T_camlinkInWorldFromAR(1,0),  T_camlinkInWorldFromAR(1,1),  T_camlinkInWorldFromAR(1,2),
                    T_camlinkInWorldFromAR(2,0),  T_camlinkInWorldFromAR(2,1),  T_camlinkInWorldFromAR(2,2);
    Eigen::Quaternionf q_camLink(rot_camLink);

    //create the pose message
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose.position.x =     T_camlinkInWorldFromAR(0,3);
    pose.pose.pose.position.y =     T_camlinkInWorldFromAR(1,3);
    pose.pose.pose.position.z =     T_camlinkInWorldFromAR(2,3);
    pose.pose.pose.orientation.x =  q_camLink.x();
    pose.pose.pose.orientation.y =  q_camLink.y();
    pose.pose.pose.orientation.z =  q_camLink.z();
    pose.pose.pose.orientation.w =  q_camLink.w();
    pose.header.frame_id = "map";

    //emit the signal
    emit signalPublishInitialPose(pose);
}

/**
 * @brief MainWindow::slotPoseRMS       SLOT to be called when new root mean square value was calculated by the global localization
 * @param rmsVal
 */
void MainWindow::slotPoseRMS(float rmsVal) {
    currRMSVal = rmsVal;
    lastLocTime = ros::Time::now();
//    ROS_INFO("NewRMS val received %f", currRMSVal);
}

/**
 * @brief MainWindow::on_btnPublishImage_clicked
 */
void MainWindow::on_btnPublishImage_clicked()
{
    //publish the current projection image
    emit signalProjectImage(this->projectorImage);
}

/**
 * @brief MainWindow::on_btnLoadWorld_clicked       Load a Scene
 */
void MainWindow::on_btnLoadWorld_clicked()
{
    //set the scene filepath
    string worldFile = resourceDir;
    worldFile += ui->lineLoadWorldFile->text().toStdString();

    //create the xml file parser
    file<> xmlFile(worldFile.c_str());
    xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    //get root node
    xml_node<> *rootNode = doc.first_node("modelscene");
//    cout << rootNode->first_node("name")->value() << endl;

    //get the scene data
    stringstream ss;
    string modelFile, textureFile, visibility;
    int id;
    double x,y,z,yaw,pitch,roll;

    //iterate over all objects
    for(xml_node<> * object = rootNode->first_node("object"); object; object = object->next_sibling()) {
//        cout << "Name: " << object->first_node("name")->value() << endl;
//        cout << "Geometry: " << object->first_node("geometry")->value() << endl;

        //check if object is a model file (.obj) or a pointcloud (.pcd)
        ss.str(string());
        ss.clear();
        ss << object->first_node("geometry")->value();
        modelFile = ss.str();
        string teststring = modelFile.substr(modelFile.length() - 3, 3);
        if(strcmp(teststring.c_str(), "obj") == 0) {
            //OBJ File
            id = modelVec.size();

            //load the model
            loadModel(modelFile);

            //apply the texture
            cout << "Texture: " << object->first_node("texture")->value() << endl;
            ss.str(string());
            ss.clear();
            ss << object->first_node("texture")->value();
            textureFile = ss.str();
            if(!textureFile.empty())
                addTexture(id, textureFile);

            //set the model position
            x = atof(object->first_node("position")->first_node("x")->value()) ;
            y = atof(object->first_node("position")->first_node("y")->value()) ;
            z = atof(object->first_node("position")->first_node("z")->value()) ;
            modelVec.at(id).actor->SetPosition(x, y, z);
            modelVec.at(id).positionXYZ = Eigen::Vector3f(x, y, z);

            //set the model orientation
            yaw = atof(object->first_node("orientation")->first_node("y")->value()) ;
            pitch = atof(object->first_node("orientation")->first_node("p")->value()) ;
            roll = atof(object->first_node("orientation")->first_node("r")->value()) ;
            modelVec.at(id).actor->SetOrientation(roll, pitch, yaw);
            modelVec.at(id).orientationYPR = Eigen::Vector3f(yaw, pitch, roll);

            //update the model dropdown menu
            if(id==0){
                on_comboBoxModelSelect_currentIndexChanged(id);
            }else{
                ui->comboBoxModelSelect->setCurrentIndex(id);
            }

            //process the visibility mode
            ss.str(string());
            ss.clear();
            ss << object->first_node("visible")->value();
            visibility = ss.str();
            if(strcmp(visibility.c_str(), "false") == 0){
                hideModel(id);
            }

        }else if(strcmp(teststring.c_str(), "pcd") == 0) {
            //PCD File
            id = PCVec.size();

            //load the pointcloud
            loadPointCloud(modelFile, modelFile);

            //set the position and orientation
            x = atof(object->first_node("position")->first_node("x")->value()) ;
            y = atof(object->first_node("position")->first_node("y")->value()) ;
            z = atof(object->first_node("position")->first_node("z")->value()) ;

            yaw = atof(object->first_node("orientation")->first_node("y")->value()) ;
            pitch = atof(object->first_node("orientation")->first_node("p")->value()) ;
            roll = atof(object->first_node("orientation")->first_node("r")->value()) ;

            movePC(id, x, y, z, yaw, pitch, roll);
            PCVec.at(id).positionXYZ = Eigen::Vector3f(x, y, z);
            PCVec.at(id).orientationYPR = Eigen::Vector3f(yaw, pitch, roll);

            //update the pointcloud dropdown menu
            if(id==0){
                on_comboBoxPCSelect_currentIndexChanged(id);
            }else{
                ui->comboBoxPCSelect->setCurrentIndex(id);
            }

            //process the visibility mode
            ss.str(string());
            ss.clear();
            ss << object->first_node("visible")->value();
            visibility = ss.str();
            if(strcmp(visibility.c_str(), "false") == 0){
                hidePC(id);
            }
        }
    }
}

/**
 * @brief MainWindow::on_btnSaveWorld_clicked       Save a scene
 */
void MainWindow::on_btnSaveWorld_clicked()
{
    //initialize stringstream to write the values with a decimal precision of 3
    stringstream ss;
    ss.precision(3);

    //initialize the xml parser and set the file header
    xml_document<> doc;
    xml_node<>* decl = doc.allocate_node(node_declaration);
    decl->append_attribute(doc.allocate_attribute("version", "1.0"));
    decl->append_attribute(doc.allocate_attribute("encoding", "UTF-8"));
    doc.prepend_node(decl);

    //set the root node
    xml_node<> *root = doc.allocate_node(node_element, "modelscene");
    doc.append_node(root);

    //set the world name
    xml_node<> *name = doc.allocate_node(node_element, "name", "Model World 2");
    root->append_node(name);

    //create a temporary data structure to store all values because the xml appending stores only pointers and doesn't copy the data
    vector<string> objectNames;
    vector<string> positionVals;
    vector<string> orientationVals;
    vector<string> textures;
    vector<string> geometries;
    vector<string> visibilities;

    //iterate over all models and set the temporary data structure
    for(size_t model_cnt = 0; model_cnt < modelVec.size(); model_cnt++){
        //check if it is a duplicate of another object and reduce name to original model name
        string newObjName = modelVec.at(model_cnt).id;
        string teststring = newObjName.substr(newObjName.length() - 1, 1);
        if(strcmp(teststring.c_str(), ")") == 0) {
            newObjName = newObjName.substr(0, newObjName.length() - 3);
            teststring = newObjName.substr(newObjName.length() - 1, 1);
            while(strcmp(teststring.c_str(), "_") != 0){
                newObjName = newObjName.substr(0, newObjName.length() - 1);
                teststring = newObjName.substr(newObjName.length() - 1, 1);
            }
            newObjName = newObjName.substr(0, newObjName.length() - 1);
        }
        objectNames.push_back(newObjName);
        geometries.push_back(newObjName);

        ss.str(string());
        ss.clear();
        ss << modelVec.at(model_cnt).positionXYZ(0);
        positionVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << modelVec.at(model_cnt).positionXYZ(1);
        positionVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << modelVec.at(model_cnt).positionXYZ(2);
        positionVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << modelVec.at(model_cnt).orientationYPR(0);
        orientationVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << modelVec.at(model_cnt).orientationYPR(1);
        orientationVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << modelVec.at(model_cnt).orientationYPR(2);
        orientationVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << modelVec.at(model_cnt).texture;
        textures.push_back(ss.str());

        ss.str(string());
        ss.clear();
        if(modelVec.at(model_cnt).visible){
            ss << "true";
        }else {
            ss << "false";
        }
        visibilities.push_back(ss.str());

    }

    //iterate over all pointclouds and set the temporary data structure
    for(size_t pc_cnt = 0; pc_cnt < PCVec.size(); pc_cnt++){
        //check if it is a duplicate of another object and reduce name to original model name
        string newObjName = PCVec.at(pc_cnt).id;
        string teststring = newObjName.substr(newObjName.length() - 1, 1);
        if(strcmp(teststring.c_str(), ")") == 0) {
            newObjName = newObjName.substr(0, newObjName.length() - 3);
            teststring = newObjName.substr(newObjName.length() - 1, 1);
            while(strcmp(teststring.c_str(), "_") != 0){
                newObjName = newObjName.substr(0, newObjName.length() - 1);
                teststring = newObjName.substr(newObjName.length() - 1, 1);
            }
            newObjName = newObjName.substr(0, newObjName.length() - 1);
        }
        objectNames.push_back(newObjName);
        geometries.push_back(newObjName);

        ss.str(string());
        ss.clear();
        ss << PCVec.at(pc_cnt).positionXYZ(0);
        positionVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << PCVec.at(pc_cnt).positionXYZ(1);
        positionVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << PCVec.at(pc_cnt).positionXYZ(2);
        positionVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << PCVec.at(pc_cnt).orientationYPR(0);
        orientationVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << PCVec.at(pc_cnt).orientationYPR(1);
        orientationVals.push_back(ss.str());

        ss.str(string());
        ss.clear();
        ss << PCVec.at(pc_cnt).orientationYPR(2);
        orientationVals.push_back(ss.str());

        textures.push_back("");

        ss.str(string());
        ss.clear();
        if(PCVec.at(pc_cnt).visible){
            ss << "true";
        }else {
            ss << "false";
        }
        visibilities.push_back(ss.str());
    }

    //set the temporary data to the xml data structure
    for(size_t cnt = 0; cnt < objectNames.size(); cnt++){
        xml_node<> *object = doc.allocate_node(node_element, "object");
        root->append_node(object);

        xml_node<> *objName = doc.allocate_node(node_element, "name", objectNames.at(cnt).c_str());
        object->append_node(objName);

        xml_node<> *objPosition = doc.allocate_node(node_element, "position");
        object->append_node(objPosition);

        xml_node<> *posX = doc.allocate_node(node_element, "x", positionVals.at(3*cnt).c_str());
        objPosition->append_node(posX);

        xml_node<> *posY = doc.allocate_node(node_element, "y", positionVals.at(3*cnt+1).c_str());
        objPosition->append_node(posY);

        xml_node<> *posZ = doc.allocate_node(node_element, "z", positionVals.at(3*cnt+2).c_str());
        objPosition->append_node(posZ);

        xml_node<> *objOrientation = doc.allocate_node(node_element, "orientation");
        object->append_node(objOrientation);

        xml_node<> *orY = doc.allocate_node(node_element, "y", orientationVals.at(3*cnt).c_str());
        objOrientation->append_node(orY);

        xml_node<> *orP = doc.allocate_node(node_element, "p", orientationVals.at(3*cnt+1).c_str());
        objOrientation->append_node(orP);

        xml_node<> *orR = doc.allocate_node(node_element, "r", orientationVals.at(3*cnt+2).c_str());
        objOrientation->append_node(orR);

        xml_node<> *objTex = doc.allocate_node(node_element, "texture", textures.at(cnt).c_str());
        object->append_node(objTex);

        xml_node<> *objGeo = doc.allocate_node(node_element, "geometry", geometries.at(cnt).c_str());
        object->append_node(objGeo);

        xml_node<> *objVis = doc.allocate_node(node_element, "visible", visibilities.at(cnt).c_str());
        object->append_node(objVis);
    }

    //write the xml file
    string saveFile = resourceDir;
    saveFile += (ui->lineSaveWorldFile->text().toStdString());
    ofstream outputFile;
    outputFile.open(saveFile.c_str());
    outputFile << doc;
    outputFile.close();

}

/**
 * @brief MainWindow::clearScene    Clear the complete scene from the visualizer
 */
void MainWindow::clearScene()
{
    //remove all models
    while(!modelVec.empty()) {
        pclWidget->vis->removeActorFromRenderer(modelVec.at(0).actor);
        ui->comboBoxModelSelect->clear();
        ui->btnModelShow->setEnabled(true);
        ui->btnModelHide->setEnabled(true);
        modelVec.erase(modelVec.begin());
    }

    //remove all pointclouds
    while(!PCVec.empty()) {
        pclWidget->vis->removePointCloud(PCVec.at(0).id);
        ui->comboBoxPCSelect->clear();
        ui->btnPCShow->setEnabled(true);
        ui->btnPCHide->setEnabled(true);
        PCVec.erase(PCVec.begin());
    }

    //update the GUI
    ui->qvtkWidget->update();
}

/**
 * @brief MainWindow::on_btnClearScene_clicked
 */
void MainWindow::on_btnClearScene_clicked()
{
    //clear the current visualization scene
    clearScene();
}
