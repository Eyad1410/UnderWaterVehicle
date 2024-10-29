// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"
 
// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/convert.h"

#include "tf2_ros/transform_broadcaster.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop


// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace std;

// Create the node class named ParamReader which inherits the attributes
// and methods of the rclcpp::Node class.

static double kSemimajorAxis = 6378137;
static double kSemiminorAxis = 6356752.3142;
static double kFirstEccentricitySquared = 6.69437999014 * 0.001;
static double kSecondEccentricitySquared = 6.73949674228 * 0.001;
static double kFlattening = 1 / 298.257223563;

inline
double rad2Deg(const double radians)
{
  return (radians / M_PI) * 180.0;
}

inline
double deg2Rad(const double degrees)
{
  return (degrees / 180.0) * M_PI;
}

class LocalOdom2Tf : public rclcpp::Node
{

  public:
    // Constructor creates a node named minimal_publisher. 
    // The published message count is initialized to 0.
    LocalOdom2Tf():Node("enu_transform_node"){init();};
 
  private:

    void init();

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x,
                    double* y, double* z);
    void ecef2Geodetic(const double x, const double y, const double z, double* latitude,
                    double* longitude, double* altitude);

    // Get clock for throttle log
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* odom msg */
    nav_msgs::msg::Odometry odom_msg_;


};

void LocalOdom2Tf::init(){
  
  //Subscriber and Publisher
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("mavros/local_position/odom",  rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(), std::bind(&LocalOdom2Tf::odomCb, this, _1));

  //TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  clock_ = this->get_clock();

}


void LocalOdom2Tf::geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x,
                    double* y, double* z)
{
  // Convert geodetic coordinates to ECEF.
  // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
  double lat_rad = deg2Rad(latitude);
  double lon_rad = deg2Rad(longitude);
  double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
  *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
  *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
  *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
}

void LocalOdom2Tf::ecef2Geodetic(const double x, const double y, const double z, double* latitude,
                    double* longitude, double* altitude)
{
  // Convert ECEF coordinates to geodetic coordinates.
  // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
  // to geodetic coordinates," IEEE Transactions on Aerospace and
  // Electronic Systems, vol. 30, pp. 957-961, 1994.

  double r = sqrt(x * x + y * y);
  double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
  double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
  double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
  double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
  double S = cbrt(1 + C + sqrt(C * C + 2 * C));
  double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
  double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
  double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
      + sqrt(
          0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
              - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
  double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
  double V = sqrt(
      pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
  double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
  *altitude = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
  *latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
  *longitude = rad2Deg(atan2(y, x));
}

void LocalOdom2Tf::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped ros_msg_odom2tf;
  ros_msg_odom2tf.header.stamp = clock_->now();
  ros_msg_odom2tf.header.frame_id = "map";
  ros_msg_odom2tf.child_frame_id = msg->child_frame_id;
  ros_msg_odom2tf.transform.translation.x = msg->pose.pose.position.x;
  ros_msg_odom2tf.transform.translation.y = msg->pose.pose.position.y;
  ros_msg_odom2tf.transform.translation.z = msg->pose.pose.position.z;
  ros_msg_odom2tf.transform.rotation.x = msg->pose.pose.orientation.x;
  ros_msg_odom2tf.transform.rotation.y = msg->pose.pose.orientation.y;
  ros_msg_odom2tf.transform.rotation.z = msg->pose.pose.orientation.z;
  ros_msg_odom2tf.transform.rotation.w = msg->pose.pose.orientation.w;
  tf_broadcaster_->sendTransform(ros_msg_odom2tf);
  
}

// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Start processing data from the node as well as the callbacks and the timer
  rclcpp::spin(std::make_shared<LocalOdom2Tf>());
 
  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;
}
