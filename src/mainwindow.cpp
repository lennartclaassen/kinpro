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

    ui = new Ui::MainWindow();
    ui->setupUi(this);

    pclWidget = new VTKPointCloudWidget();

    displayRGBCloud = false;
    imgReady = true;
    projectorImage = cv::Mat::zeros(480, 848, CV_8UC3);

    voxelize = false;
    passthrough = false;
    segmentate = false;
    hull = false;
    transform = false;
    createProjImage = false;


    connect(ui->checkBoxRGBCloud, SIGNAL(toggled(bool)), this, SLOT(toggleRGB(bool)));

    this->setTransformations();
}

MainWindow::~MainWindow() {
}

void MainWindow::toggleRGB(bool state) {
    this->displayRGBCloud = state;
}

void MainWindow::displayCloud(PointCloud<PointXYZRGB>::Ptr pc, string id) { //only the pointcloud with the specified id is updated; default is "cloud"
    if(displayRGBCloud){
        if(!pclWidget->vis->updatePointCloud(pc, id)) {
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pc);
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, rgb, id);
            pclWidget->vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

            ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());
            ROS_INFO("First Call");
        }
    }else {
        if(!pclWidget->vis->updatePointCloud<pcl::PointXYZRGB>(pc, id)) {      //adding <pcl::PointXYZRGB> leads to ignoring color values
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

            ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());
            ROS_INFO("First Call");
        }
    }

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
    if(passthrough)
        applyPassthrough(cloud);

    if(voxelize)
        applyVoxelization(cloud);

    if(segmentate)
        applySegmentation(cloud);

    if(hull)
        applyHull(cloud);

    if(transform)
        applyTransformation(cloud);

    if(createProjImage)
        createProjectionImage();
}

void MainWindow::setTransformations()
{
    float tx, ty, tz;
    tx = line2float(*ui->lineTransCamProjX);            //TODO: load calibration values
    ty = line2float(*ui->lineTransCamProjY);
    tz = line2float(*ui->lineTransCamProjZ);

    Eigen::Matrix3f rotMatRGB2Projector;                //TODO: load calibration values
    rotMatRGB2Projector <<  line2float(*ui->lineRotCamProj_00),    line2float(*ui->lineRotCamProj_01),    line2float(*ui->lineRotCamProj_02),
                            line2float(*ui->lineRotCamProj_10),    line2float(*ui->lineRotCamProj_11),    line2float(*ui->lineRotCamProj_12),
                            line2float(*ui->lineRotCamProj_20),    line2float(*ui->lineRotCamProj_21),    line2float(*ui->lineRotCamProj_22);
//    rotMatRGB2Projector <<  0.9999, -0.0104, -0.0106,
//                            0.0073,  0.9661, -0.2582,
//                            0.0129,  0.2581,  0.9660;

    T_cam2proj <<   rotMatRGB2Projector(0,0),   rotMatRGB2Projector(0,1),   rotMatRGB2Projector(0,2),   tx,
                    rotMatRGB2Projector(1,0),   rotMatRGB2Projector(1,1),   rotMatRGB2Projector(1,2),   ty,
                    rotMatRGB2Projector(2,0),   rotMatRGB2Projector(2,1),   rotMatRGB2Projector(2,2),   tz,
                    0,                          0,                          0,                          1;

    T_intrProj <<   line2float(*ui->lineIntrinsicParamsProj_fx),   0,                                                  line2float(*ui->lineIntrinsicParamsProj_cx),
                    0,                                                  line2float(*ui->lineIntrinsicParamsProj_fy),   line2float(*ui->lineIntrinsicParamsProj_cy),
                    0,                                                  0,                                                  1;
}

void MainWindow::projectImage()
{
    imgReady = false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = m_pc_projector->makeShared();

    this->applyPassthrough(cloud);
    this->applySegmentation(cloud);
    this->applyHull(cloud);
    this->applyTransformation(cloud);
    this->createProjectionImage();
    m_pc_projector = cloud->makeShared();

    imgReady = true;
}

void MainWindow::applyVoxelization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud);
//    std::cout << "PointCloud after filtering has: " << cloud->points.size ()  << " data points." << std::endl;
}

void MainWindow::applyPassthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (line2float(*ui->linePass_min), line2float(*ui->linePass_max));
    pass.filter (*cloud);
//    cout << "PointCloud after filtering has: " << cloud->points.size () << " data points." << std::endl;
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

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setIndices (inliers);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud);
}

void MainWindow::applyHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if(ui->radioConvex->isChecked()) {
        pcl::ConvexHull<pcl::PointXYZRGB> chull;
        chull.setInputCloud (cloud);
        chull.reconstruct (*cloud);
//        cout << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
    }else if(ui->radioConcave->isChecked()) {
        pcl::ConcaveHull<pcl::PointXYZRGB> chull;
        chull.setInputCloud (cloud);
        chull.setAlpha (0.01);
        chull.reconstruct (*cloud);
//        cout << "Concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
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
        p_cam3D_hom << cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z, 1;

        //transform 3D points to 3D projector coordinates
        p_proj3D_hom = T_cam2proj * p_cam3D_hom;
//        cout << p_proj3D_hom << endl;

        p_proj3D << p_proj3D_hom(0), p_proj3D_hom(1), p_proj3D_hom(2);
//        cout << p_proj3D << endl;

        //transform 3D projector coordinates to 2D projector coordinates
        p_proj2D_hom = T_intrProj * p_proj3D;
//        cout << p_proj2D_hom << endl;

        p_proj2D << p_proj2D_hom(0), p_proj2D_hom(1);
//        cout << p_proj2D << endl << endl;

        pixVec.push_back(p_proj2D);
    }

    vector<Point> pVec;

    for(int i=0; i<pixVec.size(); i++) {
        int x = (int)pixVec.at(i)(0);
        int y = (int)pixVec.at(i)(1);
//        if(x<848 && x>=0 && y<480 && y>=0) {
            pVec.push_back(Point(x, y));
//        }
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
        cv::approxPolyDP( Mat(this->projectionContour[0]), contours_poly[0], 3, true );
        boundRect[0] = boundingRect( Mat(contours_poly[0]) );

        cv::drawContours(this->projectorImage, this->projectionContour, 0, Scalar(0,255,0), -1);
        fillPoly(this->projectorImage, contours_poly, Scalar(0,0,255));
//        cv::rectangle(this->projectorImage, boundRect[0], Scalar(255,255,255), -1);
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

void MainWindow::on_btnSetCamView_clicked()
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

void MainWindow::on_checkBoxCoordSys_toggled(bool checked)
{
    if(checked){
        pclWidget->vis->addCoordinateSystem(0.5,0.0,0.0,0.0);
    }else{
        pclWidget->vis->removeCoordinateSystem();
    }
    ui->qvtkWidget->update();

}

void MainWindow::on_btnResetCamParams_clicked()
{
    float roll, pitch, yaw;
    float tx, ty, tz;
    tx = (ui->linetX->text().toFloat());
    ty = (ui->linetY->text().toFloat());
    tz = (ui->linetZ->text().toFloat());

    yaw = (ui->lineYaw->text().toFloat())*M_PI/180.0;
    pitch = (ui->linePitch->text().toFloat())*M_PI/180.0;
    roll = (ui->lineRoll->text().toFloat())*M_PI/180.0;
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());

    Eigen::Quaternion<float> q = rollAngle * pitchAngle * yawAngle;

    Eigen::Matrix3f rotMatIR2RGB;
    rotMatIR2RGB = q.matrix();

    Eigen::Matrix3f rotMatRGB2Projector;
    rotMatRGB2Projector <<  0.9999, -0.0104, -0.0106,
                            0.0073,  0.9661, -0.2582,
                            0.0129,  0.2581,  0.9660;

    Eigen::Matrix3f rotMatIR2Projector;
    rotMatIR2Projector = rotMatIR2RGB * rotMatRGB2Projector;
//    rotMatIR2Projector = rotMatIR2RGB;


    Eigen::Matrix3f intrinsicParams;
    Eigen::Matrix4f extrinsicParams;

//    intrinsicParams << 583.257049527068, 0, 321.2388215864229, 0, 585.1600543518808, 242.7879843187553, 0, 0, 1;    //kinect ir camera params
    intrinsicParams << 1515.51089, 0, 848.0, 0, 1447.40731, 480.0, 0, 0, 1;                                 //projector params, modified for image fitting
//    extrinsicParams << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;                                              //identity for cam
//    extrinsicParams <<  0.9999, -0.0104, -0.0106,  (0.0272556 + 0.045),
//                        0.0073,  0.9661, -0.2582,  0.0495066,
//                        0.0129,  0.2581,  0.9660,  0.0205514,
//                        0,       0,       0,       1;                   //mat for projector; additional x translation due to transformation from kinect rgb cam to kinect ir projector
    extrinsicParams <<  rotMatIR2Projector(0,0), rotMatIR2Projector(0,1), rotMatIR2Projector(0,2), tx,
                        rotMatIR2Projector(1,0), rotMatIR2Projector(1,1), rotMatIR2Projector(1,2), ty,
                        rotMatIR2Projector(2,0), rotMatIR2Projector(2,1), rotMatIR2Projector(2,2), tz,
                        0,                       0,                       0,                       1;

    pclWidget->vis->setCameraParameters(intrinsicParams, extrinsicParams);
    ui->qvtkWidget->update();

}

void MainWindow::on_btnSavePointcloud_clicked()
{
    std::stringstream cloudName;
    cloudName << "pointcloud.pcd";
    PointCloud<PointXYZRGB> pc_save(*m_pc);
    pcl::io::savePCDFileBinary(cloudName.str(), pc_save);
}

void MainWindow::on_btnLoadPointcloud_clicked()
{
    std::stringstream cloudName;
    cloudName << "pointcloud.pcd";
    PointCloud<PointXYZRGB> pc_load;
    pcl::io::loadPCDFile(cloudName.str(), pc_load);
    m_pc = pc_load.makeShared();
    this->displayCloud(m_pc);
}

void MainWindow::on_btnVoxelize_clicked()
{
    voxelize = !voxelize;
    ui->btnVoxelize->setChecked(voxelize);
    this->applyVoxelization(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnSegmentate_clicked()
{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(m_pc);
//    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
//    vg.setInputCloud (cloud);
//    vg.setLeafSize (0.01f, 0.01f, 0.01f);
//    vg.filter (*cloud_filtered);
//    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

//    // Create the segmentation object for the planar model and set all the parameters
//    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (100);
//    seg.setDistanceThreshold (0.005);

//    // Segment the largest planar component from the remaining cloud
//    seg.setInputCloud (cloud_filtered);
//    seg.segment (*inliers, *coefficients);
//    if (inliers->indices.size () == 0)
//    {
//        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//    }

//    // Extract the planar inliers from the input cloud
//    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
//    extract.setInputCloud (cloud_filtered);
//    extract.setIndices (inliers);
//    extract.setNegative (false);

//    // Get the points associated with the planar surface
//    extract.filter (*cloud_plane);
//    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

//    *cloud_filtered = *cloud_plane;

//    //view cloud
//    m_pc = cloud_filtered;

    segmentate = !segmentate;
    ui->btnSegmentate->setChecked(segmentate);
    this->applySegmentation(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnPassthrough_clicked()
{
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (m_pc);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

//    // Create the filtering object
//    pcl::PassThrough<pcl::PointXYZRGB> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (ui->labelPassMin->text().toFloat(), ui->linePass_max->text().toFloat());
//    //pass.setFilterLimitsNegative (true);
//    pass.filter (*cloud_filtered);

//    //view cloud
//    m_pc = cloud_filtered;

    passthrough = !passthrough;
    ui->btnPassthrough->setChecked(passthrough);
    this->applyPassthrough(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnHull_clicked()
{

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
//    cloud = m_pc;

//    // Build a filter to remove spurious NaNs
//    pcl::PassThrough<pcl::PointXYZRGB> pass;
//    pass.setInputCloud (cloud);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0.8, 0.98);
//    pass.filter (*cloud_filtered);
//    cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);

//    seg.setInputCloud (cloud_filtered);
//    seg.segment (*inliers, *coefficients);

//    // Project the model inliers
//    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
//    proj.setModelType (pcl::SACMODEL_PLANE);
//    proj.setInputCloud (cloud_filtered);
//    proj.setIndices (inliers);
//    proj.setModelCoefficients (coefficients);
//    proj.filter (*cloud_projected);

//    // Create a Convex Hull representation of the projected inliers
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);

//    if(ui->radioConvex->isChecked()) {
//        pcl::ConvexHull<pcl::PointXYZRGB> chull;
//        chull.setInputCloud (cloud_projected);
//        chull.reconstruct (*cloud_hull);

//        cout << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
//    }else if(ui->radioConcave->isChecked()) {
//        pcl::ConcaveHull<pcl::PointXYZRGB> chull;
//        chull.setInputCloud (cloud_projected);
//        chull.setAlpha (0.01);
//        chull.reconstruct (*cloud_hull);

//        cout << "Concave hull has: " << cloud_hull->points.size () << " data points." << std::endl;
//    }
//    m_pc = cloud_hull;

    hull = !hull;
    ui->btnHull->setChecked(hull);
    this->applyHull(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnTransform_clicked()
{
//    this->setTransformations();
//    //calculate the transformation between 3D points and the projector image
//    Eigen::Vector4f p_proj3D_hom, p_cam3D_hom;
//    Eigen::Vector3f p_proj3D, p_proj2D_hom;
//    Eigen::Vector2f p_proj2D;
//    vector<Eigen::Vector2f> pixVec;

//    for(int i=0; i<m_pc->points.size(); i++) {
////        cout << m_pc->points.at(i).x << m_pc->points.at(i).y << m_pc->points.at(i).z << endl;

//        p_cam3D_hom << m_pc->points.at(i).x, m_pc->points.at(i).y, m_pc->points.at(i).z, 1;

//        //transform 3D points to 3D projector coordinates
//        p_proj3D_hom = T_cam2proj * p_cam3D_hom;

////        cout << p_proj3D_hom << endl;

//        p_proj3D << p_proj3D_hom(0), p_proj3D_hom(1), p_proj3D_hom(2);

////        cout << p_proj3D << endl;

//        //transform 3D projector coordinates to 2D projector coordinates
//        p_proj2D_hom = T_intrProj * p_proj3D;

////        cout << p_proj2D_hom << endl;

//        p_proj2D << p_proj2D_hom(0), p_proj2D_hom(1);

////        cout << p_proj2D << endl << endl;

//        pixVec.push_back(p_proj2D);
//    }

//    vector< vector<Point> > contour;
//    vector<Point> pVec;
////    cv::Mat pixImg = cv::Mat::zeros(480,848,CV_8U);
//    cv::Mat pixImg = cv::imread("/home/sysadm/Desktop/chessboard.png", CV_LOAD_IMAGE_COLOR);
//    for(int i=0; i<pixVec.size(); i++) {
//        int x = (int)pixVec.at(i)(0);
//        int y = (int)pixVec.at(i)(1);
//        if(x<848 && x>=0 && y<480 && y>=0) {
//            pVec.push_back(Point(x, y));
////            pixImg.at<cv::Vec3b>(y,x)[0] = 0;
////            pixImg.at<cv::Vec3b>(y,x)[1] = 255;
////            pixImg.at<cv::Vec3b>(y,x)[2] = 0;
//        }
////            cout << "point added" << endl;
//    }
//    contour.push_back(pVec);
//    cv::drawContours(pixImg, contour, 0, Scalar(0,255,0), -1);
//    cv::imshow("pixImg", pixImg);
//    emit signalProjectImage(pixImg);
//    cv::waitKey(1);

    transform = !transform;
    ui->btnTransform->setChecked(transform);
    this->applyTransformation(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnCreateProjImage_clicked()
{
    createProjImage = !createProjImage;
    ui->btnCreateProjImage->setChecked(createProjImage);
    createProjectionImage();
}

void MainWindow::on_btnTransformApply_clicked()
{
    this->setTransformations();
}

