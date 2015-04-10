#include <kinpro/transformationProcessing.hpp>

/**
 * @brief TransformationProcessor::TransformationProcessor      Constructor for the transformation calculation thread
 */
TransformationProcessor::TransformationProcessor(){
    this->transformationsSet = false;
}

/**
 * @brief TransformationProcessor::newTFTransform       SLOT to be called when new tf trasnform was received
 * @param transform     the tf transform between map and odom
 */
void TransformationProcessor::newTFTransform(tf::StampedTransform transform)
{
    //set the translation vector
    t_map2worldVTK(0) = transform.getOrigin().x();
    t_map2worldVTK(1) = transform.getOrigin().y();
    t_map2worldVTK(2) = transform.getOrigin().z();

    //set the rotation matrix
    Eigen::Quaternion<float> qmap2world;
    qmap2world.x() = transform.getRotation().x();
    qmap2world.y() = transform.getRotation().y();
    qmap2world.z() = transform.getRotation().z();
    qmap2world.w() = transform.getRotation().w();
    R_map2worldVTK = qmap2world.matrix();

    //set the transforamtion matrix
    this->setTransformationMatrix(R_map2worldVTK, t_map2worldVTK, T_map2worldVTK);

    //emit the transform
    emit newMap2WorldVTKTransform(T_map2worldVTK);
}

/**
 * @brief TransformationProcessor::newPositionReceived      SLOT to be called when a new pose was received
 * @param msg
 */
void TransformationProcessor::newPoseReceived(nav_msgs::Odometry msg)
{
    //make sure the initial transformations were set
    if(transformationsSet) {

        //set the translation vector
        t_world2camlinkVTK(0) = msg.pose.pose.position.x;
        t_world2camlinkVTK(1) = msg.pose.pose.position.y;
        t_world2camlinkVTK(2) = msg.pose.pose.position.z;

        //set the rotation matrix
        Eigen::Quaternion<float> qworld2camlink;
        qworld2camlink.x() = msg.pose.pose.orientation.x;
        qworld2camlink.y() = msg.pose.pose.orientation.y;
        qworld2camlink.z() = msg.pose.pose.orientation.z;
        qworld2camlink.w() = msg.pose.pose.orientation.w;
        R_world2camlinkVTK = qworld2camlink.matrix();

        //set the transformation matrix
        this->setTransformationMatrix(R_world2camlinkVTK, t_world2camlinkVTK, T_world2camlinkVTK);

        //emit the transform
        emit newWorld2CamlinkVTKTransform(T_world2camlinkVTK);

        //calculate the transformation between the world and the camera (VTK)
        T_world2camVTK  = T_map2worldVTK * T_world2camlinkVTK * T_camlink2camVTK;
        emit newWorld2CamVTKTransform(T_world2camVTK);

        //calculate the transformation between the world and the projector
        T_world2proj    = T_world2camVTK * T_cam2projVTK;
        emit newWorld2ProjTransform(T_world2proj);

        //calculate the transformation between the world and the projector (VTK)
        T_world2projVTK = T_world2proj * T_VTKcam;
        emit newWorld2ProjVTKTransform(T_world2projVTK);

        emit transformDone();
    }else {
        emit transformationProcessingReady();
    }
}

/**
 * @brief TransformationProcessor::newCam2ProjTransformReceived     SLOT to be called when a new transformation between camera and projector is available
 * @param msg   The Multiarray containing the 4x4 transformation matrix
 */
void TransformationProcessor::newCam2ProjTransformReceived(std_msgs::Float32MultiArray msg) {
    T_cam2projVTK <<    msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                        msg.data[4], msg.data[5], msg.data[6], msg.data[7],
                        msg.data[8], msg.data[9], msg.data[10], msg.data[11],
                        msg.data[12], msg.data[13], msg.data[14], msg.data[15];
}

/**
 * @brief TransformationProcessor::setTransformations       SLOT to be called to set the transformations
 * @param ui        the GUI
 */
void TransformationProcessor::setTransformations(Ui::MainWindow &ui, bool useGUICam2ProjTransform)
{
//    cout << "Setting Transformations" << endl;

    //transformation from world to camera_link
    float rollworld2camlink, pitchworld2camlink, yawworld2camlink;
    t_world2camlinkVTK(0) = 0;
    t_world2camlinkVTK(1) = 0;
    t_world2camlinkVTK(2) = 0;

    yawworld2camlink = 0;
    pitchworld2camlink = 0;
    rollworld2camlink = 0;

    this->setRotationMatrixFromYPR(yawworld2camlink, pitchworld2camlink, rollworld2camlink, R_world2camlinkVTK);
    this->setTransformationMatrix(R_world2camlinkVTK, t_world2camlinkVTK, T_world2camlinkVTK);
    emit newWorld2CamlinkVTKTransform(T_world2camlinkVTK);

    //transformation from camera_link to camera_rgb_optical_frame
    t_camlink2camVTK(0) = 0.0;
    t_camlink2camVTK(1) = -0.045;
    t_camlink2camVTK(2) = 0.0;

    float rollcamlink2cam, pitchcamlink2cam, yawcamlink2cam;
    yawcamlink2cam = -(M_PI/2);
    pitchcamlink2cam = 0.0;
    rollcamlink2cam = -(M_PI/2);

    this->setRotationMatrixFromYPR(yawcamlink2cam, pitchcamlink2cam, rollcamlink2cam, R_camlink2camVTK);
    this->setTransformationMatrix(R_camlink2camVTK, t_camlink2camVTK, T_camlink2camVTK);
    emit newCamlink2CamVTKTransform(T_camlink2camVTK);

    if(useGUICam2ProjTransform) {
        //transfromation from camera_rgb_optical_frame to projector frame
        t_cam2projVTK(0) = line2float(*ui.lineTransCamProjX);            //TODO: load calibration values
        t_cam2projVTK(1) = line2float(*ui.lineTransCamProjY);
        t_cam2projVTK(2) = line2float(*ui.lineTransCamProjZ);

        R_cam2projVTK <<    line2float(*ui.lineRotCamProj_00),     line2float(*ui.lineRotCamProj_01),     line2float(*ui.lineRotCamProj_02),
                            line2float(*ui.lineRotCamProj_10),     line2float(*ui.lineRotCamProj_11),     line2float(*ui.lineRotCamProj_12),
                            line2float(*ui.lineRotCamProj_20),     line2float(*ui.lineRotCamProj_21),     line2float(*ui.lineRotCamProj_22);

        this->setTransformationMatrix(R_cam2projVTK, t_cam2projVTK, T_cam2projVTK);
        emit newCam2ProjVTKTransform(T_cam2projVTK);
    }

    //transformation from world to camera_rgb_optical_frame
    T_world2camVTK = T_world2camlinkVTK * T_camlink2camVTK;
    emit newWorld2CamVTKTransform(T_world2camVTK);

    //transformation for VTK camera (180° yaw)
    this->setRotationMatrixFromYPR(M_PI, 0.0, 0.0, R_VTKcam);

    this->setTransformationMatrix(R_VTKcam, Eigen::Vector3f(0,0,0), T_VTKcam);
    emit newVTKCamTransform(T_VTKcam);

    //transformation from world to projector
    T_world2proj    = T_world2camlinkVTK * T_camlink2camVTK * T_cam2projVTK;
    emit newWorld2ProjTransform(T_world2proj);

    T_world2projVTK = T_world2proj * T_VTKcam;
    emit newWorld2ProjVTKTransform(T_world2projVTK);

    //intrinsic projector transformation for use with VTK
    T_intrProjVTK <<    line2float(*ui.lineIntrinsicParamsProj_fx),   0,                                             line2float(*ui.lineIntrinsicParamsProj_cx),
                        0,                                             line2float(*ui.lineIntrinsicParamsProj_fy),   line2float(*ui.lineIntrinsicParamsProj_cy),
                        0,                                             0,                                             1;
    emit newIntrProjVTKTransform(T_intrProjVTK);

    //intrinsic projector transformation from calibration used for calibration validation
    T_intrProj <<   1447.40731,     0,              437.37754,
                    0,              1447.40731,     515.55742,
                    0,              0,              1;

    emit newIntrProjTransform(T_intrProj);

    T_cam2proj = T_cam2projVTK;
    emit newCam2ProjTransform(T_cam2proj);

    transformationsSet = true;
}

/**
 * @brief TransformationProcessor::setTransformationMatrix      Set a transformation matrix from rotation matrix and translation vector
 * @param in_R          rotation matrix             input (3x3)
 * @param in_t          translation vector          input (3x1)
 * @param out_T         transformation matrix       output (4x4)
 */
void TransformationProcessor::setTransformationMatrix(Eigen::Matrix3f in_R, Eigen::Vector3f in_t, Eigen::Matrix4f &out_T) {
    out_T <<    in_R(0,0),  in_R(0,1),  in_R(0,2),  in_t(0),
                in_R(1,0),  in_R(1,1),  in_R(1,2),  in_t(1),
                in_R(2,0),  in_R(2,1),  in_R(2,2),  in_t(2),
                0,          0,          0,          1;
}

/**
 * @brief TransformationProcessor::setRotationMatrixFromYPR         Create a rotation matrix from yaw, pitch, roll angles
 * @param yaw       yaw angle           input
 * @param pitch     pitch angle         input
 * @param roll      roll angle          input
 * @param out_R     rotation matrix     output
 */
void TransformationProcessor::setRotationMatrixFromYPR(float yaw, float pitch, float roll, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();
}

/**
 * @brief TransformationProcessor::setRotationMatrixFromYPR      Create a rotation matrix from combined yaw, pitch, roll vector
 * @param ypr       vector containing the yaw, pitch and roll value     input
 * @param out_R     rotation matrix                                     output
 */
void TransformationProcessor::setRotationMatrixFromYPR(Eigen::Vector3f ypr, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(ypr(0), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(ypr(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(ypr(2), Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();
}

/**
 * @brief TransformationProcessor::setIdentityMatrix     Set a 3x3 identity matrix
 * @param mat
 */
void TransformationProcessor::setIdentityMatrix(Eigen::Matrix3f &mat) {
    Eigen::Matrix3f m;
    m << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    mat = m;
}

/**
 * @brief TransformationProcessor::setIdentityMatrix     Set a 4x4 identity matrix
 * @param mat
 */
void TransformationProcessor::setIdentityMatrix(Eigen::Matrix4f &mat) {
    Eigen::Matrix4f m;
    m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    mat = m;
}
