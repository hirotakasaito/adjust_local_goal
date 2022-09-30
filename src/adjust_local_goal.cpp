#include "adjust_local_goal/adjust_local_goal.h"

AdjustLocalGoal::AdjustLocalGoal(void)
    :local_nh("~")
{
    local_nh.param("HZ", HZ, {10});
    local_nh.param("HZ", divide, {1});

    ROS_INFO("===param===");

    ROS_INFO_STREAM("HZ: " <<HZ);
    ROS_INFO_STREAM("divide: " <<10);

    local_goal_sub = nh.subscribe("/local_goal", 1, &AdjustLocalGoal::local_goal_callback, this);
    local_map_sub = nh.subscribe("/local_map/expand", 1, &AdjustLocalGoal::local_map_callback, this);

    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/adjust_local_goal", 1);

}

void AdjustLocalGoal::local_goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    local_goal = *msg;
    local_goal_updated = true;
}

void AdjustLocalGoal::local_map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    local_map = *msg;
    if(local_map.data.size() != 0) local_map_updated = true;
}

void AdjustLocalGoal::adjust_local_goal(void)
{
    column = local_map.info.height;
    row = local_map.info.width;;
    resolution = local_map.info.resolution;
    map.resize(row, std::vector<int>(column));

    local_goal_index_x = int(local_goal.pose.position.x / resolution + column/2);
    local_goal_index_y = int(-1 * local_goal.pose.position.y / resolution + row/2);

    // ROS_INFO_STREAM("local_goal_index_x" <<local_goal_index_x);
    // ROS_INFO_STREAM("local_goal_index_y" <<local_goal_index_y);

    for(int i=0; i<row; i++)
    {
        for(int j=0; j<column; j++)
        {
            map[i][j] = local_map.data[j*row+i];
        }
    }

    // ROS_INFO_STREAM(map[local_goal_index_x][local_goal_index_y]);
    if(map[local_goal_index_x][local_goal_index_y] == 100)
    {
       ROS_INFO_STREAM("change goal");
       // ROS_INFO_STREAM(map[local_goal_index_x][local_goal_index_y]);
       change_goal = true;
       calc_safe_zone(local_goal_index_x,local_goal_index_y);
    }

    geometry_msgs::PoseStamped adjust_local_goal;

    if(change_goal)
    {
        adjust_local_goal.pose.position.x = (adjust_local_goal_index_x - column/2)*resolution;
        adjust_local_goal.pose.position.y = (adjust_local_goal_index_y - row/2)*resolution*(-1);

        adjust_local_goal.pose.orientation.x = local_goal.pose.orientation.x;
        adjust_local_goal.pose.orientation.y = local_goal.pose.orientation.y;
        adjust_local_goal.pose.orientation.z = local_goal.pose.orientation.z;
        adjust_local_goal.pose.orientation.w = local_goal.pose.orientation.w;
        adjust_local_goal.header.frame_id = "base_link";
        local_goal_pub.publish(adjust_local_goal);
    }

    else
    {
        adjust_local_goal = local_goal;
        local_goal_pub.publish(adjust_local_goal);
    }

    change_goal = false;
}

void AdjustLocalGoal::calc_safe_zone(int i,int j)
{
    divide_index_x = i%divide;
    divide_index_y = j%divide;
    max_divide_x = row%divide;
    max_divide_y = column%divide;
    float min_dis = 1e5;
    float dis = 0.0;
    float dx = 0.0;
    float dy = 0.0;

    // for(int zi=divide*divide_index_x; zi<divide*divide_index_x+divide; zi++)
    // {
    //     for(int zj=divide*divide_index_y; zj<divide*divide_index_y+divide; zj++)
    //     {
    //         if(local_map.data[zi*row +zj] == 0)
    //         {
    //             cx = std::pow(zj*resolution,2.0);
    //             cy = std::pow(zi*resolution,2.0);
    //             dx = cx - local_goal.pose.position.x;
    //             dy = cy - local_goal.pose.position.y;
    //             dis = std::sqrt(std::pow(dx,2.0) + std::pow(dy,2.0));
    //             if(min_dis > dis)
    //             {
    //                 min_dis = dis;
    //                 local_goal_index_x = zj;
    //                 local_goal_index_y = zi;
    //             }
    //         }
    //     }
    // }
    float robot_r = 2.0;

    for(int zi=0; zi<column; zi++)
    {
        for(int zj=0; zj<row; zj++)
        {
            if(map[zi][zj] == 0)
            {
                dx = (zi - local_goal_index_x)*resolution;
                dy = (zj - local_goal_index_y)*resolution;
                dis = std::sqrt(std::pow(dx,2.0) + std::pow(dy,2.0));
                if(dis < robot_r)
                {
                    continue;
                }

                else if(min_dis > dis)
                {
                    min_dis = dis;
                    adjust_local_goal_index_x = zi;
                    adjust_local_goal_index_y = zj;

                    ROS_INFO_STREAM(dis);
                }
                // ROS_INFO_STREAM("x" <<local_goal_index_x);
                // ROS_INFO_STREAM("y" << local_goal_index_y);

            }
        }
    }
}

// void AdjustLocalGoal::next_zone(divide_index_x, divide_index_y)
// {
//
    // for()
// }

void AdjustLocalGoal::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok())
    {

        if(local_map_updated && local_goal_updated)
        {
            adjust_local_goal();
            local_map_updated = false;
            local_goal_updated = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "adjust_local_goal");
    AdjustLocalGoal alg;
    alg.process();
    return 0;
}

