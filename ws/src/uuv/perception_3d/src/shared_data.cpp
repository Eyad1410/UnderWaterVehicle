
#include "perception_3d/shared_data.h"

namespace perception_3d
{

SharedData::SharedData(){
  is_static_layer_ready_ = false;
  static_map_size_ = 0;
  static_ground_size_ = 0;
  aggregate_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  current_allowed_max_linear_speed_ = -1.0;
}

void SharedData::requireUpdate(){
  for(auto i=map_require_update_.begin();i!=map_require_update_.end();i++){
    (*i).second = true;
  }
}

bool SharedData::isAllUpdated(){
  for(auto i=map_require_update_.begin();i!=map_require_update_.end();i++){
    if((*i).second)
      return false;
  }
  return true;
}

}//end of name space