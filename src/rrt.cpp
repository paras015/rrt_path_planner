#include "rrt.h"

namespace rrt_planner
{

Node::Node(int x, int y, int id, int pid) : x_(x), y_(y), id_(id), pid_(pid)
{
}


RRT::RRT() 
{
    ros::NodeHandle n;
    map_sub = n.subscribe("/map", 1, &RRT::mapCallback, this);
    start_sub = n.subscribe("/initialpose", 1, &RRT::startCallback, this);
    goal_sub = n.subscribe("/move_base_simple/goal", 1, &RRT::goalCallback, this);
    path_pub = n.advertise<nav_msgs::Path>("/path", 5);
    node_pub = n.advertise<visualization_msgs::MarkerArray>("/nodes", 5);
}

void RRT::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    ROS_INFO("Map");
    map_ = map;
    nx_ = int(map_->info.width);
    ny_ = int(map_->info.height);
    ns_ = int(map_->info.width) * int(map_->info.height);
    ROS_INFO_STREAM(nx_ << " " << ny_ << " " << ns_);
    map_initialized = true;
}

void RRT::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start)
{
    ROS_INFO("Start");
    start_ = start;
    start_initialized = true;
}

void RRT::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    ROS_INFO("Goal");
    goal_ = goal;
    if (map_initialized && start_initialized) {
        ROS_INFO("INI");
        bool found = RRT::plan();
        if (found) {
            ROS_INFO("found");
        }
        ROS_INFO("not found");
    }
}

Node RRT::randomNode()
{
    std::random_device random_device;
    std::mt19937 engine(random_device());
    std::uniform_real_distribution<float> dist(0, 1);

    // if (dist(engine) > 0.05)
    // {
    std::uniform_int_distribution<int> dist_ind(0, ns_ - 1);
    const int id = dist_ind(engine);
    int x = id % nx_;
    int y = id / nx_;
    // }
    ROS_INFO_STREAM("Random node x - " << x << " Random node y - " << y << "Random node id -" << id);
    return Node(x, y, id, 0);    
}

Node RRT::findNearestPoint(std::vector<Node> list, Node& new_node)
{
    Node nearest_node;
    double min_dist = std::numeric_limits<double>::max();

    for (Node node_ : list) {
        double new_dist = std::hypot(node_.x_ - new_node.x_, node_.y_ - new_node.y_);
        if (new_dist < min_dist)
        {
            nearest_node = node_;
            new_node.pid_ = nearest_node.id_;
            min_dist = new_dist;
        }
    }

    if (min_dist > max_dist_) {
        double theta = atan2(new_node.y_ - nearest_node.y_, new_node.x_ - nearest_node.x_);
        new_node.x_ = nearest_node.x_ + (int)(max_dist_ * cos(theta));
        new_node.y_ = nearest_node.y_ + (int)(max_dist_ * sin(theta));
        new_node.id_ = new_node.x_ + nx_ * new_node.y_;
    }

    if (isAnyObstacleInPath(new_node, nearest_node))
        new_node.id_ = -1;

    return new_node;
}

bool RRT::isAnyObstacleInPath(const Node& n1, const Node& n2)
{
    double theta = atan2(n2.y_ - n1.y_, n2.x_ - n1.x_);
    double dist_ = std::hypot(n1.x_ - n2.x_, n1.y_ - n2.y_);

    if (dist_ > max_dist_)
        return true;

    double resolution_ = map_->info.resolution;
    int n_step = (int)(dist_);
    ROS_INFO_STREAM("Checking Obstacles ---- ");
    ROS_INFO_STREAM("n1 x - " << n1.x_ << " n1 y - " << n1.y_);
    ROS_INFO_STREAM("n2 x - " << n2.x_ << " n2 y - " << n2.y_);
    for (int i = 0; i < n_step; i++)
    {
        int line_x = (float)n1.x_ + (float)(i * cos(theta));
        int line_y = (float)n1.y_ + (float)(i * sin(theta));
        ROS_INFO_STREAM("line x - " << line_x << " line y - " << line_y << " Value - " << int(map_->data[line_y * nx_ + line_x]));
        if (int(map_->data[line_y * nx_ + line_x]) >= 65)
            return true;
    }

  return false;
}

bool RRT::checkGoal(const Node& new_node, const Node& goal)
{
  auto dist_ = std::hypot(new_node.x_ - goal.x_, new_node.y_ - goal.y_);
  if (dist_ > max_dist_)
    return false;

  if (!isAnyObstacleInPath(new_node, goal))
  {
    return true;
  }
  return false;
}

double RRT::indexToCoordinate(int index)
{
    double coord = (index * map_->info.resolution) + (int)map_->info.origin.position.x;
    return coord;
}

void RRT::publishPath(std::vector<Node>& sample_path, Node& start)
{
    nav_msgs::Path path;
    path.header.frame_id = "map";
    int curr_ind = sample_path.back().id_;
    int parent_id = sample_path.back().pid_;
    while (curr_ind != start.id_) {
        for (int i = 0; i < sample_path.size(); i++) {
            if (sample_path[i].id_ == parent_id) {
                curr_ind = sample_path[i].id_;
                parent_id = sample_path[i].pid_;
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = indexToCoordinate(sample_path[i].x_);
                pose.pose.position.y = indexToCoordinate(sample_path[i].y_);
                path.poses.push_back(pose);
                break;
            }
        }
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = indexToCoordinate(start.x_);
    pose.pose.position.y = indexToCoordinate(start.y_);
    path.poses.push_back(pose);
    path_pub.publish(path);

    int id = 0;
    visualization_msgs::MarkerArray node_arr;
    for (auto node_ : sample_path)
    {
        visualization_msgs::Marker mark;
        mark.header.frame_id = "map";
        mark.id = id;
        mark.type = 3;
        mark.pose.position.x = indexToCoordinate(node_.x_);
        mark.pose.position.y = indexToCoordinate(node_.y_);
        mark.scale.x = 0.01;
        mark.scale.y = 0.01;
        mark.color.r = 255;
        mark.color.g = 99;
        mark.color.b = 71;
        mark.color.a = 0.5;
        node_arr.markers.push_back(mark);
        id++;
    }
    node_pub.publish(node_arr);
}

bool RRT::plan()
{
    int start_x_ind = (start_->pose.pose.position.x - (int)map_->info.origin.position.x) / map_->info.resolution;
    int start_y_ind = (start_->pose.pose.position.y - (int)map_->info.origin.position.y) / map_->info.resolution;

    int goal_x_ind = (goal_->pose.position.x - (int)map_->info.origin.position.x) / map_->info.resolution;
    int goal_y_ind = (goal_->pose.position.y - (int)map_->info.origin.position.y) / map_->info.resolution;
    // int val = map_->data[goal_y_ind * map_->info.width + goal_x_ind];
    Node goal(goal_x_ind, goal_y_ind, goal_y_ind * nx_ + goal_x_ind, 0);
    ROS_INFO_STREAM("goal x - " << goal.x_ << " goal y - " << goal.y_ << "goal id - " << goal.id_);
    std::vector<Node> sample_list;
    Node start(start_x_ind, start_y_ind, start_y_ind * nx_ + start_x_ind);
    sample_list.push_back(start);
    int iter = 0;
    while (iter < 10000)
    {
        Node sample_node = randomNode();
        if (int(map_->data[sample_node.id_]) >= 65)
            continue;
        
        bool found = false;
        for (int i = 0; i < sample_list.size(); i++) {
            if ((sample_list[i].x_ == sample_node.x_) && (sample_list[i].y_ == sample_node.y_)) {
                found = true;
                break;
            }
        }
        if (found)
            continue;
        
        Node new_node = findNearestPoint(sample_list, sample_node);
        if (new_node.id_ == -1)
            continue;
        
        else
        {
            sample_list.push_back(new_node);
        }

        if (checkGoal(new_node, goal))
        {  
            goal.pid_ = new_node.id_;
            sample_list.push_back(goal);
            ROS_INFO_STREAM(" node x - " << new_node.x_ << " node y - " << new_node.y_ << " node id - " << new_node.id_);
            publishPath(sample_list, start);
            return true;
        }
        iter++;
        ROS_INFO_STREAM("iter - " << iter);
    }
    return false;
}

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "rrt_planner");
    
    rrt_planner::RRT rrt_plan;
    ros::spin();
    return 0;
}