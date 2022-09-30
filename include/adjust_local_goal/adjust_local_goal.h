#ifndef ADJUST_LOCAL_GOAL_H
#define ADJUST_LOCAL_GOAL_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

class AdjustLocalGoal
{
public:
    AdjustLocalGoal();
    void process(void);

private:
    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr &);
    void local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);
    void calc_safe_zone(int i, int j);
    void adjust_local_goal(void);

    int HZ;
    int local_goal_index_x = 0;
    int local_goal_index_y = 0;
    int adjust_local_goal_index_x;
    int adjust_local_goal_index_y;

    int row;
    int column;
    float resolution;
    int divide;
    int max_divide_x;
    int max_divide_y;
    int divide_index_x;
    int divide_index_y;

    bool local_goal_updated;
    bool local_map_updated;
    bool change_goal;
    std::vector<std::vector<int>> map;

    ros::Publisher local_goal_pub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber local_map_sub;

    ros::NodeHandle local_nh;
    ros::NodeHandle nh;

    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::PoseStamped local_goal;




};

#endif //__ADJUST_LOCAL_GOAL_H
