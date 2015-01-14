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
 * @file   mainwindow.hpp
 * @author Lennart Claassen
 * @date   Sep 08 2014
 *
 * @brief  This file contains a class for the kinect projector gui window
 */
#ifndef _KINPRO_MAIN_WINDOW_H
#define _KINPRO_MAIN_WINDOW_H

#include <kinpro/qtros.hpp>
//#include <kinpro/transformationProcessing.hpp>
#include "../../build/kinpro/ui_mainwindow.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Qt
#include <QDebug>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMessageBox>
#include <QString>
#include <QtGui>

//VTK
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkGlyph3D.h>
#include <vtkImageData.h>
#include <vtkImageGradientMagnitude.h>
#include <vtkImageMagnitude.h>
#include <vtkImageShiftScale.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkLineSource.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSliderWidget.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkVRMLImporter.h>
#include <vtkWindowToImageFilter.h>
#include <QVTKWidget.h>
#include <QVTKInteractor.h>
#include <vtkJPEGReader.h>

#include <vtkVersion.h>
#include <vtkDataSet.h>
#include <vtkActorCollection.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkRendererCollection.h>
#include <vtkLight.h>
#include <vtkPolyDataWriter.h>
#include <vtkConeSource.h>

#include <vtkOBBTree.h>
#include <vtkModifiedBSPTree.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/console/parse.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>

// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Boost
#include <boost/foreach.hpp>

//Eigen
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>

#define Instantiate( obj, class ) vtkSmartPointer<class> obj = vtkSmartPointer<class>::New();

class VTKPointCloudWidget: QVTKWidget
{

    public:
        /**
         * @brief MainWindow constructor
         * @param *parent pointer to parent widget
         */
        VTKPointCloudWidget(QWidget *parent = 0);

        /**
         * @brief MainWindow destructor
         */
        ~VTKPointCloudWidget();

        pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB>::Ptr c;
        pcl::visualization::PCLVisualizer *vis;

    private:


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

        VTKPointCloudWidget* pclWidget;

    signals:

        void signalProjectImage(cv::Mat img);
        void signalPublishPointcloud(pcl::PointCloud<pcl::PointXYZRGB> pc);
        void signalPublishInitialPose(geometry_msgs::PoseWithCovarianceStamped pose);
        void signalCallGlobalLoc();
        void signalCallLocalLoc();
        void signalCallPauseLoc();
        void signalCallResumeLoc();
        void signalPauseVisOdom();
        void signalResumeVisOdom();

        void signalGetARTransform();
        void signalToggleARDet();

        void setTransformations(Ui::MainWindow& ui);


    private slots:
        void timerCallback();

        void on_checkBoxRGBCloud_toggled(bool checked);

        void on_checkBoxCoordSys_toggled(bool checked);

        void on_btnSetCamView_clicked();

        void on_btnSavePointcloud_clicked();

        void on_btnLoadPointcloud_clicked();

        void on_btnTransformApply_clicked();

        void on_btnResetIntrFoc_clicked();

        void on_btnResetIntrPrinc_clicked();

        void on_btnResetExtrRot_clicked();

        void on_btnResetExtrTrans_clicked();

        void on_btnSetCamViewPos_clicked();

        void on_btnGetCamParams_clicked();

        void on_btnSetCamParams_clicked();

        void on_btnResetCamParams_clicked();

        void on_btnCreateImgFromGUI_clicked();

        void on_sliderPass_min_valueChanged(int value);

        void on_sliderPass_max_valueChanged(int value);

        void on_linePass_max_textEdited(const QString &arg1);

        void on_linePass_min_textEdited(const QString &arg1);

        void on_btnSegmentate_clicked();

        void on_comboBoxPlanes_activated(int index);

        void on_btnCreateProjImage_clicked();

        void on_btnFilterPlane_clicked();

        void on_btnLoadModel_clicked();

        void on_btnSetCamTrafo_clicked();

        void on_btnModelShow_clicked();

        void on_btnModelHide_clicked();

        void on_btnModelDel_clicked();

        void on_btnPCShow_clicked();

        void on_btnPCHide_clicked();

        void on_btnPCDel_clicked();

        void on_comboBoxPCSelect_currentIndexChanged(int index);

        void on_comboBoxModelSelect_currentIndexChanged(int index);

        void on_btnModelMove_clicked();

        void on_btnPCMove_clicked();

        void on_btnMergeClouds_clicked();

        void on_btnPCSave_clicked();

        void on_btnModelReset_clicked();

        void on_btnPCReset_clicked();

        void on_btnPCSendOcto_clicked();

        void on_btnAddCone_clicked();

        void on_btnRemoveCone_clicked();

        void on_comboBox_currentIndexChanged(int index);

        void on_btnGlobalLoc_clicked();

        void on_btnLocalLoc_clicked();

        void on_btnPauseLoc_clicked();

        void on_btnResumeLoc_clicked();

        void on_btnSetInitPose_clicked();

        void on_btnPauseVisOdom_clicked();

        void on_btnResumeVisOdom_clicked();

        void on_btnAddTexture_clicked();

        void on_btnTestChess_clicked();

        void on_btnGetARTransform_clicked();

        void on_btnTransformByAR_clicked();

        void on_btnGetPoseErrorProj_clicked();

        void on_btnProjectBoard_clicked();

        void on_btnGetPoseErrorLoc_clicked();

        void on_btnSetInitPoseByAR_clicked();

        void on_btnTestMove_clicked();

public slots:

        void newPointCloud(pcl::PointCloud<pcl::PointXYZRGB> pc);
//        void newPosition(nav_msgs::Odometry msg);
        void newLine(kinpro_interaction::line line);
        void newTransform();
        void transformationProcessingReady();
        void newARTransform(std::vector<geometry_msgs::TransformStamped> transforms);
        void slotPoseRMS(float rmsVal);


        void newCam2ProjVTKTransform(Eigen::Matrix4f T);
        void newMap2WorldVTKTransform(Eigen::Matrix4f T);
        void newWorld2CamlinkVTKTransform(Eigen::Matrix4f T);
        void newCamlink2CamVTKTransform(Eigen::Matrix4f T);
        void newWorld2CamVTKTransform(Eigen::Matrix4f T);
        void newVTKCamTransform(Eigen::Matrix4f T);
        void newWorld2ProjVTKTransform(Eigen::Matrix4f T);
        void newWorld2ProjTransform(Eigen::Matrix4f T);
        void newCam2ProjTransform(Eigen::Matrix4f T);
        void newIntrProjVTKTransform(Eigen::Matrix3f T);
        void newIntrProjTransform(Eigen::Matrix3f T);


    private:
        //structure for VTK actor entries
        struct actorEntry { vtkSmartPointer<vtkActor> actor; std::string id; bool visible; Eigen::Vector3f positionXYZ; Eigen::Vector3f orientationYPR; std::vector<double> bounds;};

        //structure for point cloud entries, position in m, orientation in DEG
        struct PCEntry { pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; std::string id; bool visible; Eigen::Vector3f positionXYZ; Eigen::Vector3f orientationYPR; std::vector<double> bounds;};

        bool transformReady;

        Ui::MainWindow* ui;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_bg;
        vector<pcl::PointCloud<pcl::PointXYZRGB> >segmentedPlanes;

        boost::mutex m_cloudMtx;
        boost::mutex m_positionMutex;
        boost::mutex m_lineMutex;

        enum OperationMode{
            BASIC = 0,
            MOVEOBJECTS
        };

        void loadPointCloud(std::string filename = std::string("pointcloud.pcd"));
        void savePointCloud(std::string filename = std::string("pointcloud.pcd"));
        void setRenderWindowVis2Qt();
        void displayCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, std::string id = std::string("cloud"));
//        void setTransformations();
        void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void applyVoxelization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void applyPassthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void applySegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void applyHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void applyTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void createProjectionImage();
        void showProjectionImage();
        void createProjectionImageFromGUI();

        void updateModelIndex();
        void updateModelButtons();
        void updatePCIndex();
        void updatePCButtons();

        void moveModel();
        void moveModel(actorEntry &entry, Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR);
        void moveModelRelative(actorEntry &entry, Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR);
        void movePC();

        void resetPCPose();
        void resetModelPose();

        void sendPCToOctomapServer();

        //Helper Functions
        int line2int(QLineEdit& line)       { return (&line)->text().toInt(); }
        double line2double(QLineEdit& line) { return (&line)->text().toDouble(); }
        float line2float(QLineEdit& line)   { return (&line)->text().toFloat(); }

        void int2line(QLineEdit& line, int value)       { (&line)->setText(QString::number(value)); }
        void double2line(QLineEdit& line, double value) { (&line)->setText(QString::number(value)); }
        void float2line(QLineEdit& line, float value)   { (&line)->setText(QString::number(value)); }

        void setTransformationMatrix(Eigen::Matrix3f in_R, Eigen::Vector3f in_t, Eigen::Matrix4f &out_T);
        void setRotationMatrixFromYPR(float yaw, float pitch, float roll, Eigen::Matrix3f &out_R);
        void setRotationMatrixFromYPR(Eigen::Vector3f ypr, Eigen::Matrix3f &out_R);

        void setIdentityMatrix(Eigen::Matrix4f &mat);
        void setIdentityMatrix(Eigen::Matrix3f &mat);

        void setPCTransformationLines();
        void setModelTransformationLines();

        void transformLineToWorld(Eigen::Vector4f &pt_start, Eigen::Vector4f &pt_end, Eigen::Vector4f &pt_start_world, Eigen::Vector4f &pt_end_world);
        void intersectLineWithModels(Eigen::Vector4f &start, Eigen::Vector4f &end, std::vector<Eigen::Vector3f> &intersections, std::vector<int> &ids);

        void projectWorldPointToProjectorImage(Eigen::Vector3f &pt_world, cv::Point &pt_projector);

        void visualizeLine(Eigen::Vector4f &start, Eigen::Vector4f &end);
        bool checkForClick(int id);

        void addSphere(Eigen::Vector3f &center, string id);
        void removeSphere(std::string &id);
        void removeAllSpheres();

        void switchOperationMode(int mode = BASIC);

        void addArrow(Eigen::Vector3f &center, Eigen::Vector3f &axis, float length = 1.0, float radius = 1.0, float resolution = 10.0, int id = 0);
        void removeArrow(int id = 0);
//        void highlightActor(std::string &id);
        void highlightActor(int id);

        int operationMode;

        void addArrowsForActor(actorEntry &actor);
        void moveArrows(Eigen::Vector3f translateXYZ, Eigen::Vector3f rotateYPR);

        void addCoordinateSystem(Eigen::Vector4f origin, Eigen::Vector4f x, Eigen::Vector4f y, Eigen::Vector4f z, std::string name = "coordinates");

        std::vector< PCEntry > PCVec;
        std::vector< actorEntry > modelVec;
//        std::vector<vtkSmartPointer<vtkActor> > coneActors;
        std::vector< actorEntry > arrowVec;
        actorEntry* currentModel;

        vtkSmartPointer<vtkActor> m_lineActor;
        vtkSmartPointer<vtkOBBTree> obbTree;
        vtkSmartPointer<vtkModifiedBSPTree> bspTree;
        vtkSmartPointer<vtkActor> obbTreeActor;
        vtkSmartPointer<vtkActor> bspTreeActor;


        std::vector<std::string> sphereIDs;
        cv::Point laserPoint;
        bool drawClickingCircle;

        bool visualOdometryActive;


        bool displayRGBCloud;

        //transformation from camera coordinates (rgb frame) to projector coordinates
        Eigen::Matrix3f R_cam2projVTK;
        Eigen::Vector3f t_cam2projVTK;
        Eigen::Matrix4f T_cam2projVTK;

        //transformation from map coordinates to world coordinates (odom)
        Eigen::Matrix3f R_map2worldVTK;
        Eigen::Vector3f t_map2worldVTK;
        Eigen::Matrix4f T_map2worldVTK;

        //transformation from world coordinates to camera_link
        Eigen::Matrix3f R_world2camlinkVTK;
        Eigen::Vector3f t_world2camlinkVTK;
        Eigen::Matrix4f T_world2camlinkVTK;

        //transformation from camera_link to camera_rgb_optical_frame
        Eigen::Matrix3f R_camlink2camVTK;
        Eigen::Vector3f t_camlink2camVTK;
        Eigen::Matrix4f T_camlink2camVTK;

        //transformation from world coordinates to camera coordinates (rgb frame)
        //combines the tf transformations "world -> camera_link" and "camera_link -> camera_rgb_optical_frame"
        Eigen::Matrix3f R_world2camVTK;
        Eigen::Vector3f t_world2camVTK;
        Eigen::Matrix4f T_world2camVTK;

        //transformation for VTK camera
        Eigen::Matrix3f R_VTKcam;
        Eigen::Matrix4f T_VTKcam;

        //transfromation from world coordinates to projector coordinates
        Eigen::Matrix4f T_world2projVTK;
        Eigen::Matrix4f T_world2proj;

        //intrinsic projector transformation (differs from calibration values due to VTK visualization specifics)
        Eigen::Matrix3f T_intrProjVTK;

        //transformation from camera coordinates (rgb frame) to projector coordinates
        //TODO: same as T_cam2projVTK?
        Eigen::Matrix4f T_cam2proj;

        //intrinsic projector transformation from calibration
        Eigen::Matrix3f T_intrProj;

        //transformation matrix between ar marker and camera
        Eigen::Matrix4f T_cam2AR;

        //transformations of the projector and camera in the world coordinates determined by AR and Localization
        Eigen::Matrix4f T_projInWorldFromAR;
        Eigen::Matrix4f T_camInWorldFromAR;
        Eigen::Matrix4f T_camlinkInWorldFromAR;
        Eigen::Matrix4f T_camInWorldFromLoc;

        vector< vector<cv::Point> > projectionContour;
        cv::Mat projectorImage;

        bool waitForLines;
        std::string locationPrefix;

//        std::string currentObject;
        int currentObjectIndex;
        ros::Duration selection_thresh;
        ros::Duration selectionDuration;
        ros::Time selectionBegin;
        ros::Time lastSelectionTime;
        ros::Duration idleDuration;
        ros::Duration idle_thresh;

        QTimer timer;
        bool timerRunning;

        geometry_msgs::TransformStamped arMarker1;
        geometry_msgs::TransformStamped arMarker2;

        double previousValueSpinBoxOrientation[3];

        int noOfArrows;

        float currRMSVal;
        ros::Time lastLocTime;

};

#endif // _KINPRO_MAIN_WINDOW_H
