#include <kinpro/transformationProcessing.hpp>

TransformationProcessor::TransformationProcessor(){
    this->transformationsSet = false;
}

void TransformationProcessor::newTFTransform(tf::StampedTransform transform) {
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
    emit newMap2WorldVTKTransform(T_map2worldVTK);
}

void TransformationProcessor::newPositionReceived(nav_msgs::Odometry msg) {
    if(transformationsSet) {

        t_world2camlinkVTK(0) = msg.pose.pose.position.x;
        t_world2camlinkVTK(1) = msg.pose.pose.position.y;
        t_world2camlinkVTK(2) = msg.pose.pose.position.z;

        Eigen::Quaternion<float> qworld2camlink;
        qworld2camlink.x() = msg.pose.pose.orientation.x;
        qworld2camlink.y() = msg.pose.pose.orientation.y;
        qworld2camlink.z() = msg.pose.pose.orientation.z;
        qworld2camlink.w() = msg.pose.pose.orientation.w;

        R_world2camlinkVTK = qworld2camlink.matrix();

        this->setTransformationMatrix(R_world2camlinkVTK, t_world2camlinkVTK, T_world2camlinkVTK);
        emit newWorld2CamlinkVTKTransform(T_world2camlinkVTK);

        T_world2camVTK  = T_map2worldVTK * T_world2camlinkVTK * T_camlink2camVTK;
        emit newWorld2CamVTKTransform(T_world2camVTK);
        //        T_world2proj    = T_map2worldVTK * T_world2camlinkVTK * T_camlink2camVTK * T_cam2projVTK;
        T_world2proj    = T_world2camVTK * T_cam2projVTK;
        emit newWorld2ProjTransform(T_world2proj);
        //        T_world2projVTK = T_map2worldVTK * T_world2camlinkVTK * T_camlink2camVTK * T_cam2projVTK * T_VTKcam;
        T_world2projVTK = T_world2proj * T_VTKcam;
        emit newWorld2ProjVTKTransform(T_world2projVTK);

        emit transformDone();
    }else {
        emit transformationProcessingReady();
    }
}

void TransformationProcessor::setTransformations(Ui::MainWindow &ui)
{
    cout << "Setting Transformations" << endl;

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


    //transfromation from camera_rgb_optical_frame to projector frame
    t_cam2projVTK(0) = line2float(*ui.lineTransCamProjX);            //TODO: load calibration values
    t_cam2projVTK(1) = line2float(*ui.lineTransCamProjY);
    t_cam2projVTK(2) = line2float(*ui.lineTransCamProjZ);

    R_cam2projVTK <<    line2float(*ui.lineRotCamProj_00),     line2float(*ui.lineRotCamProj_01),     line2float(*ui.lineRotCamProj_02),
                        line2float(*ui.lineRotCamProj_10),     line2float(*ui.lineRotCamProj_11),     line2float(*ui.lineRotCamProj_12),
                        line2float(*ui.lineRotCamProj_20),     line2float(*ui.lineRotCamProj_21),     line2float(*ui.lineRotCamProj_22);

    this->setTransformationMatrix(R_cam2projVTK, t_cam2projVTK, T_cam2projVTK);
    emit newCam2ProjVTKTransform(T_cam2projVTK);

    //transformation from world to camera_rgb_optical_frame
    T_world2camVTK = T_world2camlinkVTK * T_camlink2camVTK;
    emit newWorld2CamVTKTransform(T_world2camVTK);

    //transformation for VTK camera (180° yaw)
    this->setRotationMatrixFromYPR(M_PI, 0.0, 0.0, R_VTKcam);

//    T_VTKcam << R_VTKcam(0,0),  R_VTKcam(0,1),  R_VTKcam(0,2),  0,
//                R_VTKcam(1,0),  R_VTKcam(1,1),  R_VTKcam(1,2),  0,
//                R_VTKcam(2,0),  R_VTKcam(2,1),  R_VTKcam(2,2),  0,
//                0,              0,              0,              1;
    this->setTransformationMatrix(R_VTKcam, Eigen::Vector3f(0,0,0), T_VTKcam);
    emit newVTKCamTransform(T_VTKcam);

    //transformation from world to projector
    T_world2proj    = T_world2camlinkVTK * T_camlink2camVTK * T_cam2projVTK;
    emit newWorld2ProjTransform(T_world2proj);
//    T_world2projVTK = T_world2camlinkVTK * T_camlink2camVTK * T_cam2projVTK * T_VTKcam;
    T_world2projVTK = T_world2proj * T_VTKcam;
    emit newWorld2ProjVTKTransform(T_world2projVTK);


    //intrinsic projector transformation for use with VTK
    T_intrProjVTK <<    line2float(*ui.lineIntrinsicParamsProj_fx),   0,                                             line2float(*ui.lineIntrinsicParamsProj_cx),
                        0,                                             line2float(*ui.lineIntrinsicParamsProj_fy),   line2float(*ui.lineIntrinsicParamsProj_cy),
                        0,                                             0,                                             1;
    emit newIntrProjVTKTransform(T_intrProjVTK);

    //intrinsic projector transformation from calibration used for calibration validation
//    T_intrProj <<   1515.51089,     0,              437.37754,
//                    0,              1447.40731,     515.55742,
//                    0,              0,              1;
    //modified intrinsic projector matrix, TODO: why is the transformation (in VTK) more corrrect when using fy as fx, does this also apply when the real projector is used?
    T_intrProj <<   1447.40731,     0,              437.37754,
                    0,              1447.40731,     515.55742,
                    0,              0,              1;
//    //new calibration, TODO: why is the transformation (in VTK) more corrrect when using fy as fx, does this also apply when the real projector is used?
//    T_intrProj <<   1466.64727,     0,              485.94118,
//                    0,              1466.64727,     500.66169,
//                    0,              0,              1;

    emit newIntrProjTransform(T_intrProj);

    //might be redundant if same tansformation as T_cam2projVTK is used
//    T_cam2proj <<   0.9999,    -0.0104,    -0.0106,     0.027,
//                    0.0073,     0.9661,    -0.2582,     0.049,
//                    0.0129,     0.2581,     0.9660,     0.020,
//                    0,          0,          0,          1;
    T_cam2proj = T_cam2projVTK;
    emit newCam2ProjTransform(T_cam2proj);

    transformationsSet = true;
}

void TransformationProcessor::setTransformationMatrix(Eigen::Matrix3f in_R, Eigen::Vector3f in_t, Eigen::Matrix4f &out_T) {
    out_T <<    in_R(0,0),  in_R(0,1),  in_R(0,2),  in_t(0),
                in_R(1,0),  in_R(1,1),  in_R(1,2),  in_t(1),
                in_R(2,0),  in_R(2,1),  in_R(2,2),  in_t(2),
                0,          0,          0,          1;
}

void TransformationProcessor::setRotationMatrixFromYPR(float yaw, float pitch, float roll, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();
}

void TransformationProcessor::setRotationMatrixFromYPR(Eigen::Vector3f ypr, Eigen::Matrix3f &out_R) {
    Eigen::AngleAxisf yawAngle(ypr(0), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(ypr(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(ypr(2), Eigen::Vector3f::UnitX());

    Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;

    out_R = q.matrix();
}

void TransformationProcessor::setIdentityMatrix(Eigen::Matrix3f &mat) {
    Eigen::Matrix3f m;
    m << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    mat = m;
}

void TransformationProcessor::setIdentityMatrix(Eigen::Matrix4f &mat) {
    Eigen::Matrix4f m;
    m << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    mat = m;
}
