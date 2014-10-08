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

    toggleCloudName = string("pointcloud.pcd");
//    this->on_btnLoadPointcloud_clicked();
}

MainWindow::~MainWindow() {
}

void MainWindow::loadPointCloud(string filename) {
    PointCloud<PointXYZRGB> pc_load;
    pcl::io::loadPCDFile(filename, pc_load);

    m_pc = pc_load.makeShared();

    ui->checkBoxKinect->setChecked(false);

    this->displayCloud(m_pc);
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

            ROS_INFO("Adding new cloud");
        }
    }else {
        if(!pclWidget->vis->updatePointCloud<pcl::PointXYZRGB>(pc, id)) {      //adding <pcl::PointXYZRGB> leads to ignoring color values
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

            ROS_INFO("Adding new cloud");
        }
    }

//    ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());
    ui->qvtkWidget->update();
}

void MainWindow::newPointCloud(PointCloud<PointXYZRGB> pc) {
    if(ui->checkBoxKinect->isChecked()) {
            m_pc = pc.makeShared();
            this->processCloud(m_pc);
            this->displayCloud(m_pc);
    }
}

void MainWindow::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

}

void MainWindow::setTransformations()
{
    float tx, ty, tz;
    tx = line2float(*ui->lineTransCamProjX);            //TODO: load calibration values
    ty = line2float(*ui->lineTransCamProjY);
    tz = line2float(*ui->lineTransCamProjZ);

    R_cam2projVTK <<    line2float(*ui->lineRotCamProj_00),     line2float(*ui->lineRotCamProj_01),     line2float(*ui->lineRotCamProj_02),
                        line2float(*ui->lineRotCamProj_10),     line2float(*ui->lineRotCamProj_11),     line2float(*ui->lineRotCamProj_12),
                        line2float(*ui->lineRotCamProj_20),     line2float(*ui->lineRotCamProj_21),     line2float(*ui->lineRotCamProj_22);

    T_cam2projVTK <<    R_cam2projVTK(0,0),     R_cam2projVTK(0,1),     R_cam2projVTK(0,2),     tx,
                        R_cam2projVTK(1,0),     R_cam2projVTK(1,1),     R_cam2projVTK(1,2),     ty,
                        R_cam2projVTK(2,0),     R_cam2projVTK(2,1),     R_cam2projVTK(2,2),     tz,
                        0,                      0,                      0,                      1;

    T_intrProjVTK <<    line2float(*ui->lineIntrinsicParamsProj_fx),   0,                                             line2float(*ui->lineIntrinsicParamsProj_cx),
                        0,                                             line2float(*ui->lineIntrinsicParamsProj_fy),   line2float(*ui->lineIntrinsicParamsProj_cy),
                        0,                                             0,                                             1;

    T_cam2proj <<   0.9999,    -0.0104,    -0.0106,     0.027,
                    0.0073,     0.9661,    -0.2582,     0.049,
                    0.0129,     0.2581,     0.9660,      0.020,
                    0,          0,          0,          1;

    T_intrProj <<   1515.51089,     0,              437.37754,
                    0,              1447.40731,     515.55742,
                    0,              0,              1;
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
        p_proj3D_hom = T_cam2proj * p_cam3D_hom;
        p_proj3D << p_proj3D_hom(0), p_proj3D_hom(1), p_proj3D_hom(2);

        //transform 2D points to 2D projector pixel values
        p_proj2D_hom = T_intrProj * p_proj3D;
        p_proj2D << p_proj2D_hom(0), p_proj2D_hom(1);
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

    float roll, pitch, yaw;
    float tx, ty, tz;
    tx = line2float(*ui->lineTransCamProjX);
    ty = line2float(*ui->lineTransCamProjY);
    tz = line2float(*ui->lineTransCamProjZ);

    yaw = line2float(*ui->lineYaw)*M_PI/180.0;
    pitch = line2float(*ui->linePitch)*M_PI/180.0;
    roll = line2float(*ui->lineRoll)*M_PI/180.0;
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());

    Eigen::Quaternion<float> q = rollAngle * pitchAngle * yawAngle;

    Eigen::Matrix3f rotMatIR2RGB;
    rotMatIR2RGB = q.matrix();

    Eigen::Matrix3f rotMatIR2Projector;
    rotMatIR2Projector = rotMatIR2RGB * R_cam2projVTK;

    Eigen::Matrix4f extrinsicParams;
    extrinsicParams <<  rotMatIR2Projector(0,0), rotMatIR2Projector(0,1), rotMatIR2Projector(0,2), tx,
                        rotMatIR2Projector(1,0), rotMatIR2Projector(1,1), rotMatIR2Projector(1,2), ty,
                        rotMatIR2Projector(2,0), rotMatIR2Projector(2,1), rotMatIR2Projector(2,2), tz,
                        0,                       0,                       0,                       1;

    pclWidget->vis->setCameraParameters(T_intrProjVTK, extrinsicParams);
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
    //original transformation params from camera calibration
    ui->lineRotCamProj_00->setText("0.9999");
    ui->lineRotCamProj_01->setText("-0.0104");
    ui->lineRotCamProj_02->setText("-0.0106");
    ui->lineRotCamProj_10->setText("0.0073");
    ui->lineRotCamProj_11->setText("0.9661");
    ui->lineRotCamProj_12->setText("-0.2582");
    ui->lineRotCamProj_20->setText("0.0129");
    ui->lineRotCamProj_21->setText("0.2581");
    ui->lineRotCamProj_22->setText("0.9660");
}

void MainWindow::on_btnResetExtrTrans_clicked()
{
    //original transformation params from camera calibration
//    ui->lineTransCamProjX->setText("0.027");
//    ui->lineTransCamProjY->setText("0.049");
//    ui->lineTransCamProjZ->setText("0.020");
    //modified transformation params for vtk camera positioning (projector view)
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

void MainWindow::on_btnTogglecloud_clicked()
{
    std::stringstream cloudName;
    cloudName << toggleCloudName;
    PointCloud<PointXYZRGB> pc_load;
    pcl::io::loadPCDFile(cloudName.str(), pc_load);
    m_pc = pc_load.makeShared();
    this->displayCloud(m_pc);

    if(!strcmp(toggleCloudName.c_str(), "pointcloud.pcd")) {
        toggleCloudName = string("pointcloud_dark.pcd");
    }else {
        toggleCloudName = string("pointcloud.pcd");
    }
}

void MainWindow::on_btnCreateImgFromGUI_clicked()
{
    vtkSmartPointer<vtkRenderWindow> renderWindow = pclWidget->vis->getRenderWindow();
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput( renderWindow );

    //get image
    windowToImageFilter->SetInputBufferTypeToRGB();
    windowToImageFilter->Update();
    vtkImageData* vtkimage = windowToImageFilter->GetOutput();
    int dimsImage[3];
    vtkimage->GetDimensions(dimsImage);
    cv::Mat cvImage(dimsImage[1], dimsImage[0], CV_8UC3, vtkimage->GetScalarPointer());
    cv::flip( cvImage, cvImage, 0); //align axis with visualizer
    cv::Rect roi(0,0,848,480);      //TODO projector size param
    Mat croppedImage = cvImage(roi).clone();

    //visualize
    const std::string windowNameRGBImage = "vtkRGBImage";
    cv::namedWindow( windowNameRGBImage, cv::WINDOW_AUTOSIZE);
    cv::imshow( windowNameRGBImage, croppedImage);
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

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

//    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
//    vtkDataSet *pDataset;
//    vtkActorCollection *actors = importer->GetRenderer()->GetActors();
//    actors->InitTraversal();
//    pDataset = actors->GetNextActor()->GetMapper()->GetInput();
//    vtkPolyData *polyData = vtkPolyData::SafeDownCast(pDataset);
//    polyData->Update();

//    // Renderer
//    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//    mapper->SetInput(polyData);
//    vtkSmartPointer<vtkActor> texturedQuad = vtkSmartPointer<vtkActor>::New();
//    texturedQuad->SetMapper(mapper);
//    texturedQuad->SetTexture(texture);

//    // Visualize the textured plane
//    renderer->AddActor(texturedQuad);
//    renderer->SetBackground(0.2,0.5,0.6); // Background color white
//    renderer->ResetCamera();


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
//    double color[3] = {0.89,0.81,0.34};
//    actor->GetProperty()->SetColor(color);

//    renderer->AddActor(actor);
//    renderer->SetBackground(0.3, 0.6, 0.3); // Background color green

    pclWidget->vis->addActorToRenderer(actor);
//    getRenderWindow()->AddRenderer(renderer);
//    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    ui->qvtkWidget->update();
}
