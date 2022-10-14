#ifndef ADJUST_LOCAL_GOAL_H
#define ADJUST_LOCAL_GOAL_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdlib.h>

struct Map_info
{
    float dis;
    int dr;
    int dc;
    int i;
    int j;
    float map_cost;
};

class AdjustLocalGoal
{
public:
    AdjustLocalGoal();
    void process(void);

private:
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr &);
    void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);
    void adjust_local_goal(void);

    int HZ;
    float MAP_COST_GAIN;
    float DISTANCE_GAIN;
    int local_goal_index_x = 0;
    int local_goal_index_y = 0;
    int adjust_local_goal_index_x;
    int adjust_local_goal_index_y;

    int row;
    int column;
    float resolution;
    float cost;
    int divide;
    int max_divide_x;
    int max_divide_y;
    int divide_index_x;
    int divide_index_y;
    int min_dr;
    int min_dc;
    int min_i;
    int min_j;
    int dis_min_dr;
    int dis_min_dc;
    int dis_min_i;
    int dis_min_j;

    int divide_resolution;
    int map_cost;
    int _map_cost;
    int in;
    int jn;
    float dx;
    float dy;
    float dis;
    float min_dis;
    float dis_min_dis;
    bool enable_change;
    bool local_goal_updated;
    bool local_map_updated;
    bool change_goal;
    // std::vector<std::vector<std::vector<std::vector<int>>>> divide_map;
    // std::vector<std::vector<std::vector<std::vector<int>>>> map;
    std::vector<int> map_costs;
    std::vector<Map_info> map_infos;
    Map_info map_info;

    ros::Publisher local_goal_pub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber local_map_sub;

    ros::NodeHandle nh;

    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::PoseStamped local_goal;




};

#endif //__ADJUST_LOCAL_GOAL_H