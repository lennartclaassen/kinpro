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

    displayRGBCloud = true;
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

    toggleCloudName = string("pointcloud.pcd");
    this->on_btnLoadPointcloud_clicked();
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
            ROS_INFO("Adding new cloud");
        }
    }else {
        if(!pclWidget->vis->updatePointCloud<pcl::PointXYZRGB>(pc, id)) {      //adding <pcl::PointXYZRGB> leads to ignoring color values
            pclWidget->vis->addPointCloud<pcl::PointXYZRGB>(pc, id);

            ui->qvtkWidget->SetRenderWindow (pclWidget->vis->getRenderWindow());
            ROS_INFO("Adding new cloud");
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

    R_cam2projVTK <<    line2float(*ui->lineRotCamProj_00),     line2float(*ui->lineRotCamProj_01),     line2float(*ui->lineRotCamProj_02),
                        line2float(*ui->lineRotCamProj_10),     line2float(*ui->lineRotCamProj_11),     line2float(*ui->lineRotCamProj_12),
                        line2float(*ui->lineRotCamProj_20),     line2float(*ui->lineRotCamProj_21),     line2float(*ui->lineRotCamProj_22);

    T_cam2projVTK <<    line2float(*ui->lineRotCamProj_00),     line2float(*ui->lineRotCamProj_01),     line2float(*ui->lineRotCamProj_02),     tx,
                        line2float(*ui->lineRotCamProj_10),     line2float(*ui->lineRotCamProj_11),     line2float(*ui->lineRotCamProj_12),     ty,
                        line2float(*ui->lineRotCamProj_20),     line2float(*ui->lineRotCamProj_21),     line2float(*ui->lineRotCamProj_22),     tz,
                        0,                                      0,                                      0,                                      1;

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

    //testing multi plane segmentation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_plane_proj (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_f (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB> ()),
                                            cloud_all_planes (new pcl::PointCloud<pcl::PointXYZRGB> ());
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

        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.setIndices(inliers);
        proj.filter(*cloud_plane);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane_proj);
        segmentedPlanes.push_back(*cloud_plane);
//        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

//        pcl::ConvexHull<pcl::PointXYZRGB> chull;
//        chull.setInputCloud (cloud_plane);
//        chull.setComputeAreaVolume(true);
//        chull.reconstruct (*cloud_hull);
//        double area = chull.getTotalArea();
//        if(area < 0.06 && area > 0.03)
            *cloud_all_planes += *cloud_plane;
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud = *cloud_f;
    }
//    *cloud = *cloud_all_planes;
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
        chull.setAlpha (line2double(*ui->lineHullAlpha));
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

void MainWindow::on_btnSetCamParams_clicked()
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
    PointCloud<PointXYZRGB> pc_load;
    pcl::io::loadPCDFile(cloudName.str(), pc_load);
    m_pc = pc_load.makeShared();
    passthrough = false;
    segmentate = false;
    voxelize = false;
    hull = false;
    transform = false;
    createProjImage = false;
    ui->btnPassthrough_2->setChecked(passthrough);
    ui->btnSegmentate_2->setChecked(segmentate);
    ui->btnHull_2->setChecked(voxelize);
    ui->btnVoxelize_2->setChecked(hull);
    ui->btnTransform_2->setChecked(transform);
    ui->btnCreateProjImage_2->setChecked(createProjImage);
    ui->checkBoxKinect->setChecked(false);

    this->displayCloud(m_pc);
}

void MainWindow::on_btnSavePointcloud_clicked()
{
    std::stringstream cloudName;
    string lineTxt = ui->lineSavePointcloud->text().toStdString();
    cloudName << lineTxt;
    PointCloud<PointXYZRGB> pc_save(*m_pc);
    pcl::io::savePCDFileBinary(cloudName.str(), pc_save);
}

void MainWindow::on_btnVoxelize_2_clicked()
{
    voxelize = !voxelize;
    ui->btnVoxelize_2->setChecked(voxelize);
    this->applyVoxelization(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnSegmentate_2_clicked()
{
    segmentate = !segmentate;
    ui->btnSegmentate_2->setChecked(segmentate);
    this->applySegmentation(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnPassthrough_2_clicked()
{
    passthrough = !passthrough;
    ui->btnPassthrough_2->setChecked(passthrough);
    this->applyPassthrough(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnHull_2_clicked()
{
    hull = !hull;
    ui->btnHull_2->setChecked(hull);
    this->applyHull(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnTransform_2_clicked()
{
    transform = !transform;
    ui->btnTransform_2->setChecked(transform);
    this->applyTransformation(m_pc);
    this->displayCloud(m_pc);
}

void MainWindow::on_btnCreateProjImage_2_clicked()
{
    createProjImage = !createProjImage;
    ui->btnCreateProjImage_2->setChecked(createProjImage);
    createProjectionImage();
}

void MainWindow::on_btnTransformApply_clicked()
{
    this->setTransformations();
}


double MainWindow::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
    tree.setInputCloud (cloud);

    cout << "Cloud size is: " << cloud->size() << endl;

    for (size_t i = 0; i < cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

void MainWindow::on_btnCorrGroup_clicked()
{
    std::string model_filename_ = std::string("calibration_cube.pcd");

    //Algorithm params
    bool show_keypoints_ (true);
    bool show_correspondences_ (true);
    bool use_cloud_resolution_ (false);
    bool use_hough_ (true);
    float model_ss_ (line2float(*ui->lineModelSS));
    float scene_ss_ (line2float(*ui->lineSceneSS));
    float rf_rad_ (line2float(*ui->lineRFRad));
    float descr_rad_ (line2float(*ui->lineDescrRad));
    float cg_size_ (line2float(*ui->lineCGSize));
    float cg_thresh_ (line2float(*ui->lineCGThresh));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors (new pcl::PointCloud<pcl::SHOT352> ());
    pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors (new pcl::PointCloud<pcl::SHOT352> ());

//    //  Load clouds
//    pcl::PointCloud<pcl::PointXYZ>::Ptr load_model (new pcl::PointCloud<pcl::PointXYZ> ());
//    if (pcl::io::loadPCDFile (model_filename_, *load_model) < 0)
//    {
//      std::cout << "Error loading model cloud." << std::endl;
//    }
//    pcl::copyPointCloud(*load_model, *model);

    model = m_cube->makeShared();

    scene = m_pc->makeShared();

    //  Set up resolution invariance
    if (use_cloud_resolution_)
    {
        float resolution = static_cast<float> (this->computeCloudResolution (model));
        if (resolution != 0.0f)
        {
            model_ss_   *= resolution;
            scene_ss_   *= resolution;
            rf_rad_     *= resolution;
            descr_rad_  *= resolution;
            cg_size_    *= resolution;
        }

        std::cout << "Model resolution:       " << resolution << std::endl;
        std::cout << "Model sampling size:    " << model_ss_ << std::endl;
        std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
        std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
        std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
        std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
    }

    //  Compute Normals
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);
    norm_est.compute (*model_normals);

    norm_est.setInputCloud (scene);
    norm_est.compute (*scene_normals);

    //  Downsample Clouds to Extract keypoints
    pcl::PointCloud<int> sampled_indices;

    pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);
    std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    uniform_sampling.setInputCloud (scene);
    uniform_sampling.setRadiusSearch (scene_ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);
    std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

    //  Compute Descriptor for keypoints
    pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (model);
    descr_est.compute (*model_descriptors);

    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (scene);
    descr_est.compute (*scene_descriptors);

    //  Find Model-Scene Correspondences with KdTree
    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud (model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (1);
        std::vector<float> neigh_sqr_dists (1);
        if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
        if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back (corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

    //  Actual Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Using Hough3D
    if (use_hough_)
    {
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());

        pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::ReferenceFrame> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (rf_rad_);

        rf_est.setInputCloud (model_keypoints);
        rf_est.setInputNormals (model_normals);
        rf_est.setSearchSurface (model);
        rf_est.compute (*model_rf);

        rf_est.setInputCloud (scene_keypoints);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (scene);
        rf_est.compute (*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
        clusterer.setHoughBinSize (cg_size_);
        clusterer.setHoughThreshold (cg_thresh_);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        clusterer.setInputCloud (model_keypoints);
        clusterer.setInputRf (model_rf);
        clusterer.setSceneCloud (scene_keypoints);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //clusterer.cluster (clustered_corrs);
        clusterer.recognize (rototranslations, clustered_corrs);
    }
    else // Using GeometricConsistency
    {
        pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> gc_clusterer;
        gc_clusterer.setGCSize (cg_size_);
        gc_clusterer.setGCThreshold (cg_thresh_);

        gc_clusterer.setInputCloud (model_keypoints);
        gc_clusterer.setSceneCloud (scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations, clustered_corrs);
    }

    //  Output results
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    }

}

void MainWindow::on_btnGenerateCube_clicked()
{
    pcl::PointCloud<pcl::PointXYZ> cube;
    pcl::PointCloud<pcl::PointXYZRGB> cubeRGB;

    double stepSize = ui->lineCubeRes->text().toDouble();
    double sideLength = 0.21;

    int cnt_max = (int)(sideLength / (stepSize));

    for(int i=0; i<=cnt_max; i++) {
        for(int j=0; j<=cnt_max; j++) {
            if(j == 0 || j == cnt_max || i == 0 || i == cnt_max){
                for(int k=0; k<=cnt_max; k++) {
                   cube.push_back(PointXYZ((float)(i*stepSize), (float)(j*stepSize), (float)(k*stepSize)));
                }
            }else {
                cube.push_back(PointXYZ((float)(i*stepSize), (float)(j*stepSize), 0.0));
                cube.push_back(PointXYZ((float)(i*stepSize), (float)(j*stepSize), 0.21));
            }
        }
    }
    copyPointCloud(cube, cubeRGB);
    m_cube = cubeRGB.makeShared();
    displayCloud(m_cube);
}

void MainWindow::on_btnSaveCube_clicked()
{
    std::stringstream cloudName;
    cloudName << "cube.pcd";
    PointCloud<PointXYZRGB> pc_save(*m_cube);
    pcl::io::savePCDFileBinary(cloudName.str(), pc_save);
}

void MainWindow::on_btnOrgConComp_clicked()
{
    pcl::IntegralImageNormalEstimation<PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());

    ne.setInputCloud (m_pc);
    ne.compute (*normal_cloud);
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
    // Set up Organized Multi Plane Segmentation
    mps.setMinInliers (10000);
    mps.setAngularThreshold (pcl::deg2rad (3.0)); //3 degrees
    mps.setDistanceThreshold (0.02); //2cm}
    mps.setInputNormals (normal_cloud);
    mps.setInputCloud (m_pc);

    std::vector<pcl::PlanarRegion<PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<PointXYZRGB> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    mps.segment(model_coefficients, inlier_indices);
    pcl::PointIndices::Ptr indices_ptr (new pcl::PointIndices );

//    for(int i = 0 ; i< inlier_indices.size(); i++) {
//        indices_ptr->indices = inlier_indices.at(i).indices;
//        cout << "size: " << indices_ptr->indices.size() << endl;
//    }
    indices_ptr->indices = inlier_indices.at(0).indices;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (m_pc);
        extract.setIndices (indices_ptr);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        *m_pc = *cloud_plane;
//        this->displayCloud(m_pc);


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

void MainWindow::on_btnSetCamParamsTest_clicked()
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

void MainWindow::on_btnResetCamParams_2_clicked()
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

    //get rgb-image
    windowToImageFilter->SetInputBufferTypeToRGB();
    windowToImageFilter->Update();
    vtkImageData* vtkRGBimage = windowToImageFilter->GetOutput();
    int dimsRGBImage[3];
    vtkRGBimage->GetDimensions(dimsRGBImage);
    cv::Mat cvImageRGB(dimsRGBImage[1], dimsRGBImage[0], CV_8UC3, vtkRGBimage->GetScalarPointer());
    cout << "dims1 = " << dimsRGBImage[1] << " dims2 = " << dimsRGBImage[0] << endl;
//    cv::cvtColor( cvImageRGB, cvImageRGB, CV_BGR2RGB); //convert color
    cv::flip( cvImageRGB, cvImageRGB, 0); //align axis with visualizer
    cv::Rect roi(0,0,848,480);
    Mat croppedImage = cvImageRGB(roi).clone();

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

