
/*Debug*/
#include <chrono>

#include <pluginlib/class_list_macros.hpp>

/*path for trajectory*/
#include <base_trajectory/trajectory.h>
/*For tf2::matrix3x3 as quaternion to euler*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/common/transforms.h>

/*Data shared (if needed) between trajectory theories*/
#include <trajectory_generators/trajectory_shared_data.h>

namespace trajectory_generators
{

class TrajectoryGeneratorTheory{

  public:

    TrajectoryGeneratorTheory();

    void initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node);

    void setSharedData(std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data);

    virtual bool hasMoreTrajectories() = 0;
    virtual bool nextTrajectory(base_trajectory::Trajectory& _traj) = 0;
    //@ initialise is used for stacked generators to call every time to initialize the genertator
    virtual void initialise() = 0;

  protected:

    rclcpp::Node::SharedPtr node_;
    //@onInitialize is used to read ros param for the generator
    virtual void onInitialize() = 0;
    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data_;
    std::string name_;


};


}//end of name space