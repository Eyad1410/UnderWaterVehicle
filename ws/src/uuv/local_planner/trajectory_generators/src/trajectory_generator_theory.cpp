
#include <trajectory_generators/trajectory_generator_theory.h>

namespace trajectory_generators
{

TrajectoryGeneratorTheory::TrajectoryGeneratorTheory(){

}

void TrajectoryGeneratorTheory::initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node){
  name_ = name;
  node_ = weak_node.lock();
  onInitialize();
}

void TrajectoryGeneratorTheory::setSharedData(std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data){
  shared_data_ = shared_data;
}


}//end of name space