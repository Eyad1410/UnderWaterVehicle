/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 

/*For pcl::PointXYZ*/
#include <pcl/common/geometry.h>
#include <math.h>

/*
for graph
type edge_t is defined here
type graph_t is defined here
*/

/*For perception*/
#include <perception_3d/perception_3d_ros.h>

typedef struct {
  unsigned int self_index;
  float g, h, f;
  unsigned int parent_index; 
  bool is_closed, is_opened;
} Node_t;

typedef std::pair<double, unsigned int> f_p_;

class AstarList{
  public:
    AstarList(perception_3d::StaticGraph& static_graph);

    void Initial();
    void setGraph(perception_3d::StaticGraph& static_graph);
    void updateNode(Node_t& a_node);
    void closeNode(Node_t& a_node);
    float getGVal(Node_t& a_node);
    Node_t getNode_wi_MinimumF();
    Node_t getNode(unsigned int node_index);
    bool isClosed(unsigned int node_index);
    bool isOpened(unsigned int node_index);
    bool isFrontierEmpty();
  private:

    /*Static graph is for path planning and list*/
    perception_3d::StaticGraph static_graph_;
    
    /*
    We handle closed list and accessory by unordered::map to get time complexity of O(1)
    Key: node id
    content: Node
    */
    std::unordered_map<unsigned int, Node_t> as_list_;

    /*
    We push back a pair <f,node_index> by the priority, hence ,the time complexity of checking minimum f is O(1)
    */
    //std::priority_queue<f_p_> f_priority_queue_;
    std::set<f_p_> f_priority_set_;
};

class A_Star_on_Graph{

    public:
      A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up, 
        perception_3d::StaticGraph& static_graph,
        std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros);
      
      ~A_Star_on_Graph();
      
      void updateGraph(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up, 
                                  perception_3d::StaticGraph& static_graph);

      void getPath( unsigned int start, unsigned int goal, std::vector<unsigned int>& path);
      
      void setupTurningWeight(double m_weight){turning_weight_ = m_weight;}
    private:
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up_;

      perception_3d::StaticGraph static_graph_; 
      /*Provide dynamic graph for obstacle avoidance*/
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros_;
      
      /*Create the list*/
      AstarList* ASLS_;

      //@ turning weight of the node
      double turning_weight_;

      double getThetaFromParent2Expanding(pcl::PointXYZ m_pcl_current_parent, pcl::PointXYZ m_pcl_current, pcl::PointXYZ m_pcl_expanding);
};

