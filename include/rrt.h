#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>
#include <cmath>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace rrt_planner 
{

class Node
{
public:
    Node(int x = 0, int y = 0, int id = 0, int pid = 0);
    int x_, y_, id_, pid_;

};

class RRT 
{
public:
    RRT();
    ros::Subscriber map_sub;
    ros::Subscriber start_sub;
    ros::Subscriber goal_sub;
    ros::Publisher path_pub;
    ros::Publisher node_pub;

protected:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& map);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& map);
    bool plan();
    Node randomNode();
    Node findNearestPoint(std::vector<Node> list, Node& node);
    bool isAnyObstacleInPath(const Node& n1, const Node& n2);
    bool checkGoal(const Node& new_node, const Node& goal);
    double indexToCoordinate(int index);
    void publishPath(std::vector<Node>& sample_path, Node& start);

    nav_msgs::OccupancyGrid::ConstPtr map_;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr start_;
    geometry_msgs::PoseStamped::ConstPtr goal_;
    bool map_initialized = false;
    bool start_initialized = false;
    int nx_, ny_, ns_;
    int max_dist_ = 5;
};

}