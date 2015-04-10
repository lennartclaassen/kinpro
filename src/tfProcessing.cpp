#include <kinpro/tfProcessing.hpp>

/**
 * @brief TFProcessor::TFProcessor      Contrutcor for the tf thread
 */
TFProcessor::TFProcessor(){
    targetFrame = std::string("/map");
    sourceFrame = std::string("/odom");
}

/**
 * @brief TFProcessor::newPositionReceived
 * @param msg
 */
void TFProcessor::newPoseReceived(nav_msgs::Odometry msg)
{
    //get the tf transformation between the map and the odometry frame
    try{
        ls.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(0.5));
        ls.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);

        emit signalNewTFTransform(transform);
    }catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error for map to odom transform: " << ex.what() << " (Is the localization running and initialised?)");
    }
}
