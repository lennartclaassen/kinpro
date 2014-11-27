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
    vis->setBackgroundColor(0.7, 0.7, 0.7);
}

VTKPointCloudWidget::~VTKPointCloudWidget() {
}

/* METHODS */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

//    importer = new VTKImporter();

    ui = new Ui::MainWindow();
    ui->setupUi(this);

    pclWidget = new VTKPointCloudWidget();
    ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());


    displayRGBCloud = true;
    projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);

    this->setTransformations();

    this->waitForLines = false;

    m_lineActor = vtkSmartPointer<vtkActor>::New();
    m_lineActor->GetProperty()->SetLineWidth(2);
    m_lineActor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    obbTreeActor = vtkSmartPointer<vtkActor>::New();
    obbTreeActor->GetProperty()->SetColor(0.0, 1.0, 0.0);
    bspTreeActor = vtkSmartPointer<vtkActor>::New();
    bspTreeActor->GetProperty()->SetColor(0.0, 0.0, 1.0);

    pclWidget->vis->addActorToRenderer(m_lineActor);
//    pclWidget->vis->addActorToRenderer(obbTreeActor);
//    pclWidget->vis->addActorToRenderer(bspTreeActor);

    obbTree = vtkSmartPointer<vtkOBBTree>::New();
    bspTree = vtkSmartPointer<vtkModifiedBSPTree>::New();

    ui->checkBoxCoordSys->toggle();

//    this->on_btnLoadPointcloud_clicked();
}

MainWindow::~MainWindow() {
}

void MainWindow::newPosition(nav_msgs::Odometry msg)
{
    if(ui->checkBoxUsePosSig->isChecked()) {

        cout << "new position received" << endl;

        bool transformOK = false;

        tf::TransformListener ls;
        tf::StampedTransform transform;
        try{
            string targetFrame, sourceFrame;
            targetFrame = string("/map");
            sourceFrame = string("/odom");

            ls.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(0.2));
            ls.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);
            transformOK = true;
            ui->checkBoxUsePosSig->setChecked(false);

            t_map2worldVTK(0) = transform.getOrigin().x();
            t_map2worldVTK(1) = transform.getOrigin().y();
            t_map2worldVTK(2) = transform.getOrigin().z();

            Eigen::Quaternion<float> qmap2world;
            qmap2world.x() = transform.getRotation().x();
            qmap2world.y() = transform.getRotation().y();
            qmap2world.z() = transform.getRotation().z();
            qmap2world.w() = transform.getRotation().w();

            R_map2worldVTK = qmap2world.matrix();

            this->setTransformationMatrix(R_map2worldVTK, t_map2worldVTK, T_map2worldVTK);

        }catch(tf::TransformException& ex){
            ROS_ERROR_STREAM( "Transform error for map to odom transform: " << ex.what());
        }

        if(transformOK) {

            t_world2camlinkVTK(0) = msg.pose.pose.position.x;
            t_world2camlinkVTK(1) = msg.pose.pose.position.y;
            t_world2camlinkVTK(2) = msg.pose.pose.position.z;

            //    float rollworld2camlink, pitchworld2camlink, yawworld2camlink;
            //    yawworld2camlink = DEG2RAD(line2float(*ui->lineCamTrafoYaw));
            //    pitchworld2camlink = DEG2RAD(line2float(*ui->lineCamTrafoPitch));
            //    rollworld2camlink = DEG2RAD(line2float(*ui->lineCamTrafoRoll));
            //    Eigen::AngleAxisf rollAngleworld2camlink(rollworld2camlink, Eigen::Vector3f::UnitX());
            //    Eigen::AngleAxisf yawAngleworld2camlink(yawworld2camlink, Eigen::Vector3f::UnitZ());
            //    Eigen::AngleAxisf pitchAngleworld2camlink(pitchworld2camlink, Eigen::Vector3f::UnitY());

            Eigen::Quaternion<float> qworld2camlink;
            qworld2camlink.x() = msg.pose.pose.orientation.x;
            qworld2camlink.y() = msg.pose.pose.orientation.y;
            qworld2camlink.z() = msg.pose.pose.orientation.z;
            qworld2camlink.w() = msg.pose.pose.orientation.w;

            R_world2camlinkVTK = qworld2camlink.matrix();

            T_world2camlinkVTK <<   R_world2camlinkVTK(0,0),    R_world2camlinkVTK(0,1),    R_world2camlinkVTK(0,2),    t_world2camlinkVTK(0),
                    R_world2camlinkVTK(1,0),    R_world2camlinkVTK(1,1),    R_world2camlinkVTK(1,2),    t_world2camlinkVTK(1),
                    R_world2camlinkVTK(2,0),    R_world2camlinkVTK(2,1),    R_world2camlinkVTK(2,2),    t_world2camlinkVTK(2),
                    0,                          0,                          0,                          1;

            T_world2projVTK = T_map2worldVTK * T_world2camlinkVTK * T_camlink2camVTK * T_cam2projVTK * T_VTKcam;
            T_world2camVTK = T_map2worldVTK * T_world2camlinkVTK * T_camlink2camVTK;

            pclWidget->vis->setCameraParameters(T_intrProjVTK, T_world2projVTK);
            ui->qvtkWidget->update();

            this->createProjectionImageFromGUI();

            if(ui->checkBoxShowProjImage->isChecked())
                showProjectionImage();

            if(ui->checkBoxPubImage->isChecked())
                emit signalProjectImage(this->projectorImage);
        }
    }

}

void MainWindow::newLine(kinpro_interaction::line line) {

    //clear actors
    pclWidget->vis->removeActorFromRenderer(m_lineActor);
    pclWidget->vis->removeActorFromRenderer(obbTreeActor);
    pclWidget->vis->removeActorFromRenderer(bspTreeActor);

    //set start and end point of line
    double start[3] = {line.start.x, line.start.y, line.start.z};
    double end[3] = {line.end.x, line.end.y, line.end.z};

    //transform the line points from camera into world coordinates
    Eigen::Vector4f start_cam, end_cam, start_world, end_world;
    start_cam(0) = start[0];
    start_cam(1) = start[1];
    start_cam(2) = start[2];
    start_cam(3) = 1.0;

    end_cam(0) = end[0];
    end_cam(1) = end[1];
    end_cam(2) = end[2];
    end_cam(3) = 1.0;

    this->transformLineToWorld(start_cam, end_cam, start_world, end_world);

    start[0] = start_world(0)/start_world(3);
    start[1] = start_world(1)/start_world(3);
    start[2] = start_world(2)/start_world(3);

    end[0] = end_world(0)/end_world(3);
    end[1] = end_world(1)/end_world(3);
    end[2] = end_world(2)/end_world(3);

    //visualize lines?
    if(ui->checkBoxShowLine->isChecked()) {

        //visualize line
        vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
        lineSource->SetPoint1(start);
        lineSource->SetPoint2(end);
        lineSource->Update();
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(lineSource->GetOutputPort());
        m_lineActor->SetMapper(mapper);
        pclWidget->vis->addActorToRenderer(m_lineActor);
    }

    //clear spheres
    for(size_t i = 0; i<sphereIDs.size(); i++) {
        pclWidget->vis->removeShape(sphereIDs.at(i));
    }
    sphereIDs.clear();

    //activate bounding boxes for intersection determination
    if(ui->checkBoxActivateBB->isChecked()) {

        //calculate intersections of line with models
        vector<Eigen::Vector3f> intersections;
        vector<string> ids;
        this->intersectLineWithModels(start, end, intersections, ids);

        //create spheres (laser pointer) for the intersections
        for(size_t i = 0; i<intersections.size(); i++) {
            stringstream sphereID;
            sphereID << "Sphere" << i;
            pcl::ModelCoefficients sphere_coeff;
            sphere_coeff.values.resize (4);                     //we need 4 values
            sphere_coeff.values[0] = intersections.at(i)(0);    //center x
            sphere_coeff.values[1] = intersections.at(i)(1);    //center y
            sphere_coeff.values[2] = intersections.at(i)(2);    //center z
            sphere_coeff.values[3] = 0.01;                      //radius
            if(!pclWidget->vis->addSphere(pcl::PointXYZ(intersections.at(i)(0), intersections.at(i)(1), intersections.at(i)(2)), 0.01, 1.0, 0.0, 0.0, sphereID.str()))
                pclWidget->vis->updateSphere(pcl::PointXYZ(intersections.at(i)(0), intersections.at(i)(1), intersections.at(i)(2)), 0.01, 1.0, 0.0, 0.0, sphereID.str());
            sphereIDs.push_back(sphereID.str());                //store sphere names for later removal
        }

        //highlight the intersected models
        for(size_t j=0; j<ids.size(); j++) {
            for(size_t k=0; k<modelVec.size(); k++) {
                if(!strcmp(ids.at(j).c_str(), modelVec.at(k).id.c_str()))
                    modelVec.at(k).actor->GetProperty()->SetColor(0.87, 0.898, 0.7);
            }
        }
    }

    ui->qvtkWidget->update();
}

void MainWindow::transformLineToWorld(Eigen::Vector4f &pt_start, Eigen::Vector4f &pt_end, Eigen::Vector4f &pt_start_world, Eigen::Vector4f &pt_end_world) {
    //transform the start and end point of the line into the world coordinates
    pt_start_world = T_world2camVTK * pt_start;
    pt_end_world = T_world2camVTK * pt_end;
}

void MainWindow::intersectLineWithModels(double pt_start[], double pt_end[], std::vector<Eigen::Vector3f>& intersections, std::vector<std::string> &ids) {

    if(!modelVec.empty()) {

        //calculate intersections with all models
        for(size_t cnt = 0; cnt < modelVec.size(); cnt++) {

            //only iterate over visible models
            if(modelVec.at(cnt).visible) {

                //reset model color
                modelVec.at(cnt).actor->GetProperty()->SetColor(1, 1, 1);

                Eigen::Vector3f intersectionPoint;

                //use obbTree model TODO: use same procedure for both models
                if(ui->radioButtonOBB->isChecked()) {

                    //create obbTree
                    obbTree->SetDataSet(modelVec.at(cnt).actor->GetMapper()->GetInput());
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
                        cout << "Hit! (" << obbHit << ")" << endl;
                        cout << "Model " << modelVec.at(cnt).id << endl;
                        ids.push_back(modelVec.at(cnt).id);
                        double intersection[3];
                        for(int i = 0; i < intersectPoints->GetNumberOfPoints(); i++ )
                        {
                            intersectPoints->GetPoint(i, intersection);
                            std::cout << "Intersection " << i << ": "
                                      << intersection[0] << ", "
                                      << intersection[1] << ", "
                                      << intersection[2] << std::endl;
                            intersectionPoint(0) = intersection[0];
                            intersectionPoint(1) = intersection[1];
                            intersectionPoint(2) = intersection[2];
                            intersections.push_back(intersectionPoint);
                        }
                    }
                }

                //use bspTree model
                if(ui->radioButtonBSP->isChecked()) {

                    //build bspTree
                    bspTree->SetDataSet(modelVec.at(cnt).actor->GetMapper()->GetInput());
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
                        cout << "Hit! (" << bspHit << ")" << endl;
                        cout << "Model " << modelVec.at(cnt).id << endl;
                        ids.push_back(modelVec.at(cnt).id);
                        std::cout << "Intersection: "
                                  << x[0] << ", "
                                  << x[1] << ", "
                                  << x[2] << std::endl;
                        intersectionPoint(0) = x[0];
                        intersectionPoint(1) = x[1];
                        intersectionPoint(2) = x[2];
                        intersections.push_back(intersectionPoint);
                    }
                }
            }

        }
    }
}


void MainWindow::loadPointCloud(string filename) {
    PointCloud<PointXYZRGB> pc_load;
    pcl::io::loadPCDFile(filename, pc_load);

    //transform cloud from rgb frame to world frame
    if(ui->checkBoxCalibMode->isChecked()) {
        m_pc = pc_load.makeShared();
        ui->checkBoxKinect->setChecked(false);
        this->displayCloud(m_pc);
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

        ui->checkBoxKinect->setChecked(false);
        this->displayCloud(PCVec.back().cloud, PCVec.back().id);
        this->updatePCIndex();
    }
}

void MainWindow::savePointCloud(string filename) {
    PointCloud<PointXYZRGB> pc_save(*m_pc);
    pcl::io::savePCDFileBinary(filename, pc_save);
}

void MainWindow::displayCloud(PointCloud<PointXYZRGB>::Ptr pc, string id) { //only the pointcloud with the specified id is updated; default is "cloud"
    if(displayRGBCloud){
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

void MainWindow::newPointCloud(PointCloud<PointXYZRGB> pc) {
    if(ui->checkBoxKinect->isChecked()) {
            m_pc = pc.makeShared();
            this->processCloud(m_pc);
            this->displayCloud(m_pc, "kinect");
    }
}

void MainWindow::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    //transform cloud from rgb_frame to world frame
    pcl::transformPointCloud(*cloud, *cloud, T_world2camVTK);
}

void MainWindow::setTransformations()
{
    //transformation from world to camera_link
    float rollworld2camlink, pitchworld2camlink, yawworld2camlink;
    t_world2camlinkVTK(0) = line2float(*ui->lineCamTrafoX);
    t_world2camlinkVTK(1) = line2float(*ui->lineCamTrafoY);
    t_world2camlinkVTK(2) = line2float(*ui->lineCamTrafoZ);

    yawworld2camlink = DEG2RAD(line2float(*ui->lineCamTrafoYaw));
    pitchworld2camlink = DEG2RAD(line2float(*ui->lineCamTrafoPitch));
    rollworld2camlink = DEG2RAD(line2float(*ui->lineCamTrafoRoll));

//    Eigen::AngleAxisf yawAngleworld2camlink(yawworld2camlink, Eigen::Vector3f::UnitZ());
//    Eigen::AngleAxisf pitchAngleworld2camlink(pitchworld2camlink, Eigen::Vector3f::UnitY());
//    Eigen::AngleAxisf rollAngleworld2camlink(rollworld2camlink, Eigen::Vector3f::UnitX());

//    Eigen::Quaternion<float> qworld2camlink = yawAngleworld2camlink * pitchAngleworld2camlink * rollAngleworld2camlink;

//    cout << "q.x = " << qworld2camlink.x() << endl << "q.y = " << qworld2camlink.y() << endl << "q.z = " << qworld2camlink.z() << endl << "q.w = " << qworld2camlink.w() << endl;

//    R_world2camlinkVTK = qworld2camlink.matrix();
    this->setRotationMatrixFromYPR(yawworld2camlink, pitchworld2camlink, rollworld2camlink, R_world2camlinkVTK);

//    T_world2camlinkVTK <<   R_world2camlinkVTK(0,0),    R_world2camlinkVTK(0,1),    R_world2camlinkVTK(0,2),    t_world2camlinkVTK(0),
//                            R_world2camlinkVTK(1,0),    R_world2camlinkVTK(1,1),    R_world2camlinkVTK(1,2),    t_world2camlinkVTK(1),
//                            R_world2camlinkVTK(2,0),    R_world2camlinkVTK(2,1),    R_world2camlinkVTK(2,2),    t_world2camlinkVTK(2),
//                            0,                          0,                          0,                          1;
    this->setTransformationMatrix(R_world2camlinkVTK, t_world2camlinkVTK, T_world2camlinkVTK);


    //transformation from camera_link to camera_rgb_optical_frame
    t_camlink2camVTK(0) = 0.0;
    t_camlink2camVTK(1) = -0.045;
    t_camlink2camVTK(2) = 0.0;

    float rollcamlink2cam, pitchcamlink2cam, yawcamlink2cam;
    yawcamlink2cam = -(M_PI/2);
    pitchcamlink2cam = 0.0;
    rollcamlink2cam = -(M_PI/2);
//    Eigen::AngleAxisf yawAnglecamlink2cam(yawcamlink2cam, Eigen::Vector3f::UnitZ());
//    Eigen::AngleAxisf pitchAnglecamlink2cam(pitchcamlink2cam, Eigen::Vector3f::UnitY());
//    Eigen::AngleAxisf rollAnglecamlink2cam(rollcamlink2cam, Eigen::Vector3f::UnitX());

//    Eigen::Quaternion<float> qcamlink2cam = yawAnglecamlink2cam * pitchAnglecamlink2cam * rollAnglecamlink2cam;

////    cout << "q.x = " << qcamlink2cam.x() << endl << "q.y = " << qcamlink2cam.y() << endl << "q.z = " << qcamlink2cam.z() << endl << "q.w = " << qcamlink2cam.w() << endl;

//    R_camlink2camVTK = qcamlink2cam.matrix();
    this->setRotationMatrixFromYPR(yawcamlink2cam, pitchcamlink2cam, rollcamlink2cam, R_camlink2camVTK);


//    T_camlink2camVTK << R_camlink2camVTK(0,0),  R_camlink2camVTK(0,1),  R_camlink2camVTK(0,2),  t_camlink2camVTK(0),
//                        R_camlink2camVTK(1,0),  R_camlink2camVTK(1,1),  R_camlink2camVTK(1,2),  t_camlink2camVTK(1),
//                        R_camlink2camVTK(2,0),  R_camlink2camVTK(2,1),  R_camlink2camVTK(2,2),  t_camlink2camVTK(2),
//                        0,                      0,                      0,                      1;
    this->setTransformationMatrix(R_camlink2camVTK, t_camlink2camVTK, T_camlink2camVTK);


    //transfromation from camera_rgb_optical_frame to projector frame
    t_cam2projVTK(0) = line2float(*ui->lineTransCamProjX);            //TODO: load calibration values
    t_cam2projVTK(1) = line2float(*ui->lineTransCamProjY);
    t_cam2projVTK(2) = line2float(*ui->lineTransCamProjZ);

    R_cam2projVTK <<    line2float(*ui->lineRotCamProj_00),     line2float(*ui->lineRotCamProj_01),     line2float(*ui->lineRotCamProj_02),
                        line2float(*ui->lineRotCamProj_10),     line2float(*ui->lineRotCamProj_11),     line2float(*ui->lineRotCamProj_12),
                        line2float(*ui->lineRotCamProj_20),     line2float(*ui->lineRotCamProj_21),     line2float(*ui->lineRotCamProj_22);

//    T_cam2projVTK <<    R_cam2projVTK(0,0),     R_cam2projVTK(0,1),     R_cam2projVTK(0,2),     t_cam2projVTK(0),
//                        R_cam2projVTK(1,0),     R_cam2projVTK(1,1),     R_cam2projVTK(1,2),     t_cam2projVTK(1),
//                        R_cam2projVTK(2,0),     R_cam2projVTK(2,1),     R_cam2projVTK(2,2),     t_cam2projVTK(2),
//                        0,                      0,                      0,                      1;
    this->setTransformationMatrix(R_cam2projVTK, t_cam2projVTK, T_cam2projVTK);

    //transformation from world to camera_rgb_optical_frame
    T_world2camVTK = T_world2camlinkVTK * T_camlink2camVTK;


    //transformation for VTK camera (180° yaw)
//    Eigen::AngleAxisf yawAngleVTKcam(M_PI, Eigen::Vector3f::UnitZ());
//    Eigen::AngleAxisf pitchAngleVTKcam(0.0, Eigen::Vector3f::UnitY());
//    Eigen::AngleAxisf rollAngleVTKcam(0.0, Eigen::Vector3f::UnitX());

//    Eigen::Quaternion<float> qVTKcam= yawAngleVTKcam* pitchAngleVTKcam * rollAngleVTKcam;

////    cout << "q.x = " << qVTKcam.x() << endl << "q.y = " << qVTKcam.y() << endl << "q.z = " << qVTKcam.z() << endl << "q.w = " << qVTKcam.w() << endl;

//    R_VTKcam = qVTKcam.matrix();
    this->setRotationMatrixFromYPR(M_PI, 0.0, 0.0, R_VTKcam);

//    T_VTKcam << R_VTKcam(0,0),  R_VTKcam(0,1),  R_VTKcam(0,2),  0,
//                R_VTKcam(1,0),  R_VTKcam(1,1),  R_VTKcam(1,2),  0,
//                R_VTKcam(2,0),  R_VTKcam(2,1),  R_VTKcam(2,2),  0,
//                0,              0,              0,              1;
    this->setTransformationMatrix(R_VTKcam, Eigen::Vector3f(0,0,0), T_VTKcam);


    //transformation from world to projector
/********** FOR TESTING
//    Eigen::Matrix4f first, second, last;
//    if(ui->comboBoxMatMult1->currentIndex() == 0){
//        first = T_world2camlinkVTK;
//    }else if(ui->comboBoxMatMult1->currentIndex() == 1){
//        first = T_camlink2camVTK;
//    }else if(ui->comboBoxMatMult1->currentIndex() == 2){
//        first = T_cam2projVTK;
//    }else if(ui->comboBoxMatMult1->currentIndex() == 3){
//        first = T_world2camlinkVTK.inverse();
//    }else if(ui->comboBoxMatMult1->currentIndex() == 4){
//        first = T_camlink2camVTK.inverse();
//    }else if(ui->comboBoxMatMult1->currentIndex() == 5){
//        first = T_cam2projVTK.inverse();
//    }else if(ui->comboBoxMatMult1->currentIndex() == 6){
//        first = Eigen::Matrix4f::Identity();
//    }

//    if(ui->comboBoxMatMult2->currentIndex() == 0){
//        second = T_world2camlinkVTK;
//    }else if(ui->comboBoxMatMult2->currentIndex() == 1){
//        second = T_camlink2camVTK;
//    }else if(ui->comboBoxMatMult2->currentIndex() == 2){
//        second = T_cam2projVTK;
//    }else if(ui->comboBoxMatMult2->currentIndex() == 3){
//        second = T_world2camlinkVTK.inverse();
//    }else if(ui->comboBoxMatMult2->currentIndex() == 4){
//        second = T_camlink2camVTK.inverse();
//    }else if(ui->comboBoxMatMult2->currentIndex() == 5){
//        second = T_cam2projVTK.inverse();
//    }else if(ui->comboBoxMatMult2->currentIndex() == 6){
//        second = Eigen::Matrix4f::Identity();
//    }

//    if(ui->comboBoxMatMult3->currentIndex() == 0){
//        last = T_world2camlinkVTK;
//    }else if(ui->comboBoxMatMult3->currentIndex() == 1){
//        last = T_camlink2camVTK;
//    }else if(ui->comboBoxMatMult3->currentIndex() == 2){
//        last = T_cam2projVTK;
//    }else if(ui->comboBoxMatMult3->currentIndex() == 3){
//        last = T_world2camlinkVTK.inverse();
//    }else if(ui->comboBoxMatMult3->currentIndex() == 4){
//        last = T_camlink2camVTK.inverse();
//    }else if(ui->comboBoxMatMult3->currentIndex() == 5){
//        last = T_cam2projVTK.inverse();
//    }else if(ui->comboBoxMatMult3->currentIndex() == 6){
//        last = Eigen::Matrix4f::Identity();
//    }

//    T_world2camVTK = first * second;
//    T_world2projVTK = first * second * last;
**********/
    T_world2projVTK = T_world2camlinkVTK * T_camlink2camVTK * T_cam2projVTK * T_VTKcam;


    //intrinsic projector transformation for use with VTK
    T_intrProjVTK <<    line2float(*ui->lineIntrinsicParamsProj_fx),   0,                                             line2float(*ui->lineIntrinsicParamsProj_cx),
                        0,                                             line2float(*ui->lineIntrinsicParamsProj_fy),   line2float(*ui->lineIntrinsicParamsProj_cy),
                        0,                                             0,                                             1;


    //intrinsic projector transformation from calibration used for calibration validation
    T_intrProj <<   1515.51089,     0,              437.37754,
                    0,              1447.40731,     515.55742,
                    0,              0,              1;




    //might be redundant if same tansformation as T_cam2projVTK is used
//    T_cam2proj <<   0.9999,    -0.0104,    -0.0106,     0.027,
//                    0.0073,     0.9661,    -0.2582,     0.049,
//                    0.0129,     0.2581,     0.9660,     0.020,
//                    0,          0,          0,          1;
    T_cam2proj = T_cam2projVTK;


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

    if(ui->checkBoxShowProjImage->isChecked())
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
    cv::Mat cvImage(dimsImage[1], dimsImage[0], CV_8UC3, vtkimage->GetScalarPointer());
    cv::cvtColor( cvImage, cvImage, CV_RGB2BGR); //convert color
    cv::flip( cvImage, cvImage, 0); //align axis with visualizer
    cv::Rect roi(0,0,848,480);      //TODO projector size param
    this->projectorImage = cvImage(roi).clone();
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
    this->setTransformations();

    pclWidget->vis->setCameraParameters(T_intrProjVTK, T_world2projVTK);
    ui->qvtkWidget->update();
}

void MainWindow::on_btnSetCamTrafo_clicked()
{
    float rollW, pitchW, yawW;
    float txW, tyW, tzW;
    txW = line2float(*ui->lineCamTrafoX);
    tyW = line2float(*ui->lineCamTrafoY);
    tzW = line2float(*ui->lineCamTrafoZ);

    yawW = DEG2RAD(line2float(*ui->lineCamTrafoYaw));
    pitchW = DEG2RAD(line2float(*ui->lineCamTrafoPitch));
    rollW = DEG2RAD(line2float(*ui->lineCamTrafoRoll));
    Eigen::AngleAxisf rollAngleW(rollW, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngleW(yawW, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngleW(pitchW, Eigen::Vector3f::UnitY());

    Eigen::Quaternion<float> qW = rollAngleW * pitchAngleW * yawAngleW;

    Eigen::Matrix3f rotMatWorld2Cam;
    rotMatWorld2Cam = qW.matrix();
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
    this->setTransformations();
}

void MainWindow::on_btnResetIntrFoc_clicked()
{
    //original transformation params from camera calibration
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

void MainWindow::on_btnSetCamViewPos_clicked()
{
    double posX, posY, posZ, X_foc, Y_foc, Z_foc, X_up, Y_up, Z_up;
    posX = line2double(*ui->lineX);
    posY = line2double(*ui->lineY);
    posZ = line2double(*ui->lineZ);
    X_foc = line2double(*ui->lineX_foc);
    Y_foc = line2double(*ui->lineY_foc);
    Z_foc = line2double(*ui->lineZ_foc);
    X_up = line2double(*ui->lineX_up);
    Y_up = line2double(*ui->lineY_up);
    Z_up = line2double(*ui->lineZ_up);

    pclWidget->vis->setCameraPosition(posX, posY, posZ, X_foc, Y_foc, Z_foc, X_up, Y_up, Z_up);
    ui->qvtkWidget->update();
}

void MainWindow::on_btnGetCamParams_clicked()
{
    vector<pcl::visualization::Camera> camVec;
    pclWidget->vis->getCameras(camVec);
    if(!camVec.empty()) {

        //focal
        double2line(*ui->lineCamParamsFocX, camVec.at(0).focal[0]);
        double2line(*ui->lineCamParamsFocY, camVec.at(0).focal[1]);
        double2line(*ui->lineCamParamsFocZ, camVec.at(0).focal[2]);
        cout << "Focal\t\t" << camVec.at(0).focal[0] << "\t" << camVec.at(0).focal[1] << "\t" << camVec.at(0).focal[2] << endl;

        //position
        double2line(*ui->lineCamParamsPosX, camVec.at(0).pos[0]);
        double2line(*ui->lineCamParamsPosY, camVec.at(0).pos[1]);
        double2line(*ui->lineCamParamsPosZ, camVec.at(0).pos[2]);
        cout << "Position\t" << camVec.at(0).pos[0] << "\t" << camVec.at(0).pos[1] << "\t" << camVec.at(0).pos[2] << endl;

        //view
        double2line(*ui->lineCamParamsViewX, camVec.at(0).view[0]);
        double2line(*ui->lineCamParamsViewY, camVec.at(0).view[1]);
        double2line(*ui->lineCamParamsViewZ, camVec.at(0).view[2]);
        cout << "View\t\t" << camVec.at(0).view[0] << "\t" << camVec.at(0).view[1] << "\t" << camVec.at(0).view[2] << endl;

        //clipping
        double2line(*ui->lineCamParamsClipX, camVec.at(0).clip[0]);
        double2line(*ui->lineCamParamsClipY, camVec.at(0).clip[1]);
        cout << "Clipping\t" << camVec.at(0).view[0] << "\t" << camVec.at(0).view[1] << "\t" << camVec.at(0).view[2] << endl;

        //FOV Y
        double2line(*ui->lineCamParamsFovY, camVec.at(0).fovy);
        cout << "FOV y\t\t" << camVec.at(0).fovy << endl;

        //window size
        double2line(*ui->lineCamParamsWinSizeX, camVec.at(0).window_size[0]);
        double2line(*ui->lineCamParamsWinSizeY, camVec.at(0).window_size[1]);

        //window position
        double2line(*ui->lineCamParamsWinPosX, camVec.at(0).window_pos[0]);
        double2line(*ui->lineCamParamsWinPosY, camVec.at(0).window_pos[1]);
    }
}

void MainWindow::on_btnSetCamParams_clicked()
{
    pcl::visualization::Camera cam;

    //focal
    cam.focal[0] = line2double(*ui->lineCamParamsFocX);
    cam.focal[1] = line2double(*ui->lineCamParamsFocY);
    cam.focal[2] = line2double(*ui->lineCamParamsFocZ);

    //position
    cam.pos[0] = line2double(*ui->lineCamParamsPosX);
    cam.pos[1] = line2double(*ui->lineCamParamsPosY);
    cam.pos[2] = line2double(*ui->lineCamParamsPosZ);

    //view
    cam.view[0] = line2double(*ui->lineCamParamsViewX);
    cam.view[1] = line2double(*ui->lineCamParamsViewY);
    cam.view[2] = line2double(*ui->lineCamParamsViewZ);

    //clipping
    cam.clip[0] = line2double(*ui->lineCamParamsClipX);
    cam.clip[1] = line2double(*ui->lineCamParamsClipY);

    //FOV Y
    cam.fovy = line2double(*ui->lineCamParamsFovY);

    //window size
    cam.window_size[0] = line2double(*ui->lineCamParamsWinSizeX);
    cam.window_size[1] = line2double(*ui->lineCamParamsWinSizeY);

    //window position
    cam.window_pos[0] = line2double(*ui->lineCamParamsWinPosX);
    cam.window_pos[1] = line2double(*ui->lineCamParamsWinPosY);

    pclWidget->vis->setCameraParameters(cam);
    ui->qvtkWidget->update();
}

void MainWindow::on_btnResetCamParams_clicked()
{
    //focal
    double2line(*ui->lineCamParamsFocX, -0.0164);
    double2line(*ui->lineCamParamsFocY, 0.2092);
    double2line(*ui->lineCamParamsFocZ, 0.946);

    //position
    double2line(*ui->lineCamParamsPosX, -0.027);
    double2line(*ui->lineCamParamsPosY, -0.049);
    double2line(*ui->lineCamParamsPosZ, -0.02);

    //view
    double2line(*ui->lineCamParamsViewX, 0.0103997);
    double2line(*ui->lineCamParamsViewY, -0.966065);
    double2line(*ui->lineCamParamsViewZ, 0.258091);

    //clipping
    double2line(*ui->lineCamParamsClipX, 0.01);
    double2line(*ui->lineCamParamsClipY, 1000.01);

    //FOV Y
    double2line(*ui->lineCamParamsFovY, 0.328637);

    //window size
    double2line(*ui->lineCamParamsWinSizeX, 848);
    double2line(*ui->lineCamParamsWinSizeY, 480);

    //window position
    double2line(*ui->lineCamParamsWinPosX, 0.0);
    double2line(*ui->lineCamParamsWinPosY, 0.0);
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

void MainWindow::on_btnSegmentate_clicked()
{
    //deactivate kinect input
    ui->checkBoxKinect->setChecked(false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    //apply passthrough filter
    this->applyPassthrough(cloud);

    //apply plane segmentation
    this->applySegmentation(cloud);

    //list selectable planes
    ui->comboBoxPlanes->clear();
    for( int i = 0; i<segmentedPlanes.size(); i++ ) {
        stringstream ss;
        ss << "Plane " << i;
        ui->comboBoxPlanes->addItem(QString::fromStdString(ss.str()));
    }
}

void MainWindow::on_comboBoxPlanes_activated(int index)
{
    if(!segmentedPlanes.empty() && index < segmentedPlanes.size()) {
        m_pc = segmentedPlanes.at(index).makeShared();
        displayCloud(m_pc);
    }
}

void MainWindow::on_btnFilterPlane_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    if(line2double(*ui->lineVoxRes) > 0.0)
        this->applyVoxelization(cloud);

    this->applyHull(cloud);
    m_pc = cloud->makeShared();
    displayCloud(cloud);
}

void MainWindow::on_btnCreateProjImage_clicked()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    cloud = m_pc->makeShared();

    this->applyTransformation(cloud);
    this->createProjectionImage();
}

void MainWindow::on_btnLoadModel_clicked()
{
    string filename = ui->lineLoadModel->text().toStdString();
    string actorname = ui->lineLoadActor->text().toStdString();
    const char *cstr = actorname.c_str();
    cout << "Reading: " << filename << endl;

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
//    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
//    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();

    reader->SetFileName(filename.c_str());
    reader->Update();

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
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
            displayCloud(PCVec.at(id).cloud, PCVec.at(id).id);
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

            //move the model first back to origin and then using the new input values
            vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
            transform->PostMultiply();
            transform->Translate(-modelVec.at(id).positionXYZ(0), -modelVec.at(id).positionXYZ(1), -modelVec.at(id).positionXYZ(2));
            transform->RotateX(-modelVec.at(id).orientationYPR(2));
            transform->RotateY(-modelVec.at(id).orientationYPR(1));
            transform->RotateZ(-modelVec.at(id).orientationYPR(0));
            transform->RotateZ(ui->spinBoxMoveObjYaw->value());
            transform->RotateY(ui->spinBoxMoveObjPitch->value());
            transform->RotateX(ui->spinBoxMoveObjRoll->value());
            transform->Translate(ui->spinBoxMoveObjX->value(), ui->spinBoxMoveObjY->value(), ui->spinBoxMoveObjZ->value());

            vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
            transformFilter->SetInput(modelVec.at(id).actor->GetMapper()->GetInput());
            transformFilter->SetTransform(transform);
            transformFilter->Update();

            // Create a mapper and actor
            vtkSmartPointer<vtkPolyDataMapper> transformedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            transformedMapper->SetInputConnection(transformFilter->GetOutputPort());
            modelVec.at(id).actor->SetMapper(transformedMapper);

            //update values to new position
            modelVec.at(id).positionXYZ = Eigen::Vector3f(ui->spinBoxMoveObjX->value(), ui->spinBoxMoveObjY->value(), ui->spinBoxMoveObjZ->value());
            modelVec.at(id).orientationYPR = Eigen::Vector3f(ui->spinBoxMoveObjYaw->value(), ui->spinBoxMoveObjPitch->value(), ui->spinBoxMoveObjRoll->value());

            ui->qvtkWidget->update();
        }

    }
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
            displayCloud(PCVec.at(id).cloud, PCVec.at(id).id);

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

            ui->checkBoxKinect->setChecked(false);
            this->displayCloud(PCVec.back().cloud, PCVec.back().id);
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

        ui->spinBoxMoveObjYaw->setValue((double)(modelVec.at(id).orientationYPR(0)));
        ui->spinBoxMoveObjPitch->setValue((double)modelVec.at(id).orientationYPR(1));
        ui->spinBoxMoveObjRoll->setValue((double)modelVec.at(id).orientationYPR(2));
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
        filename << PCVec.at(id).id << ".pcd";
        pcl::io::savePCDFileBinary(filename.str(), pc_save);
    }
}

void MainWindow::on_btnModelReset_clicked()
{
    this->resetModelPose();
}

void MainWindow::resetModelPose() {
    if(!modelVec.empty()) {
        this->waitForLines = true;
        ui->spinBoxMoveObjX->setValue(0);
        ui->spinBoxMoveObjY->setValue(0);
        ui->spinBoxMoveObjZ->setValue(0);

        ui->spinBoxMoveObjYaw->setValue(0);
        ui->spinBoxMoveObjPitch->setValue(0);
        ui->spinBoxMoveObjRoll->setValue(0);
        this->waitForLines = false;

        this->moveModel();
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
