#ifndef PERCEPTION_3D_SHARED_DATA_H_
#define PERCEPTION_3D_SHARED_DATA_H_

#include "rclcpp/rclcpp.hpp"

/*For kd tree*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>


/*For planner*/
#include <perception_3d/dynamic_graph.h>
#include <perception_3d/static_graph.h>

namespace perception_3d
{

class SharedData{
  public:

    SharedData();
    
    //@ reset map center to request every one to update dynamic graph
    void requireUpdate();
    //@ return false if any false in map_require_update_
    bool isAllUpdated();
    
    //@ Variable that compute from local planner for path blocked check
    pcl::PointCloud<pcl::PointXYZI> pcl_prune_plan_;

    //@ Variable that loaded in static layer for other layer to use
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_map_;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_ground_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ground_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map_;
    bool is_static_layer_ready_;
    std::map<std::string, bool> map_require_update_;

    //@ Variable in static layer that inform new map cloud in coming
    size_t static_ground_size_;
    size_t static_map_size_;

    //@ Static graph in static layer for path planning
    std::shared_ptr<perception_3d::StaticGraph> sGraph_ptr_;    
    
    //@ We only make aggregate observation as shared
    //@ Add kd tree as shared make system unstable, because multi process are calling the tree
    //@ I am not able to make a more stable structure
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_observation_;
    
    //@ limit the current speed of trajectories
    double current_allowed_max_linear_speed_;

  private:



};


}//end of name space

#endif