#include <perception_3d/nothing_layer.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::NothingLayer, perception_3d::Sensor)

namespace perception_3d
{

NothingLayer::NothingLayer(){
}

NothingLayer::~NothingLayer(){
}

void NothingLayer::onInitialize()
{ 
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

void NothingLayer::selfMark(){
}

void NothingLayer::selfClear(){
}

void NothingLayer::resetdGraph(){
}

double NothingLayer::get_dGraphValue(const unsigned int index){
  return 1000000.0;
}

bool NothingLayer::isCurrent(){
  current_ = true;
  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr NothingLayer::getObservation(){
  return sensor_current_observation_;
}

}//end of name space
