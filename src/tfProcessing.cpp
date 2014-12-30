#include <kinpro/tfProcessing.hpp>

TFProcessor::TFProcessor(){
    targetFrame = std::string("/map");
    sourceFrame = std::string("/odom");
}

void TFProcessor::newPositionReceived(nav_msgs::Odometry msg) {
    try{
        ls.waitForTransform(targetFrame, sourceFrame, ros::Time(0), ros::Duration(0.5));
        ls.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);

        emit signalNewTFTransform(transform);
    }catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error for map to odom transform: " << ex.what());
    }
}
