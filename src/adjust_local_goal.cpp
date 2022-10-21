#include "adjust_local_goal/adjust_local_goal.h"

AdjustLocalGoal::AdjustLocalGoal(void)
    :nh("~")
{
    nh.param("HZ", HZ, {50});
    nh.param("DIVIDE", divide, {10});
    nh.param("MAP_COST_GAIN", MAP_COST_GAIN, {10.0});
    nh.param("DISTANCE_GAIN", DISTANCE_GAIN, {1.0});

    ROS_INFO("===param===");

    ROS_INFO_STREAM("HZ: " <<HZ);
    ROS_INFO_STREAM("divide: " <<divide);
    ROS_INFO_STREAM("MAP_COST_GAIN: " <<MAP_COST_GAIN);
    ROS_INFO_STREAM("DISTANCE_GAIN: " <<DISTANCE_GAIN);

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
    double pi = M_PI;
    divide_resolution = row / divide;

    // local_goal_index_x = int(local_goal.pose.position.x / resolution + column/2);
    // local_goal_index_y = int(-1 * local_goal.pose.position.y / resolution + row/2);

    local_goal_index_x = int((local_goal.pose.position.x*std::cos(-pi/2) - local_goal.pose.position.y*std::sin(-pi/2)) / resolution) + row/2;
    local_goal_index_y = int((local_goal.pose.position.x*std::cos(-pi/2) + local_goal.pose.position.y*std::sin(-pi/2))/resolution) + column/2;
    ROS_INFO_STREAM("local_goal_index_x");
    ROS_INFO_STREAM(local_goal_index_x);
    ROS_INFO_STREAM("local_goal_index_y");
    ROS_INFO_STREAM(local_goal_index_y);

    float min_cost = 5e5;
    float max_dis = 0.0;
    float max_map_cost = 0.0;

    if(local_map.data[local_goal_index_y*row + local_goal_index_x] == 100)
    {
       ROS_INFO_STREAM("change goal");
       change_goal = true;

        for(int dr=0; dr<divide; dr++)
        {
            for(int dc=0; dc<divide; dc++)
            {
                map_cost = 0;
                min_dis = 5e5;
                for(int i=0; i<divide_resolution; i++)
                {
                    for(int j=0; j<divide_resolution; j++)
                    {
                        if(local_map.data[(divide_resolution*dc+j)*row+(divide_resolution*dr+i)] == 0)
                        {
                            dx = ((divide_resolution*dr+i) - local_goal_index_x)*resolution;
                            dy = ((divide_resolution*dc+j) - local_goal_index_y)*resolution;
                            dis = std::sqrt(std::pow(dx,2.0) + std::pow(dy,2.0));

                            if(dis < min_dis)
                            {
                                min_dis = dis;
                                dis_min_dr = dr;
                                dis_min_dc = dc;
                                dis_min_i = i;
                                dis_min_j = j;
                            }


                            enable_change = true;
                        }

                        _map_cost = local_map.data[(divide_resolution*dc+j)*row+(divide_resolution*dr+i)];
                        ROS_INFO_STREAM(min_dis);
                        if(_map_cost == -1) _map_cost = 100;
                        map_cost += _map_cost;

                    }
                }

                if(enable_change)
                {
                    enable_change = false;
                    map_info.dr = dis_min_dr;
                    map_info.dc = dis_min_dc;
                    map_info.i = dis_min_i;
                    map_info.j = dis_min_j;
                    map_info.dis = min_dis;
                    map_info.map_cost = map_cost;
                    map_infos.push_back(map_info);
                }

                if(min_dis != 5e5 && min_dis > max_dis)
                {
                    max_dis = min_dis;
                }

                if(map_cost > max_map_cost)
                {
                    max_map_cost = map_cost;
                }

            }
        }


        for(const auto& map_info : map_infos)
        {
            // normalize_map_cost = map_info.map_cost / max_map_cost;
            // normalize_dis = map_info.dis / max_dis;

            // cost = MAP_COST_GAIN * normalize_map_cost + DISTANCE_GAIN * normalize_dis;
            cost = MAP_COST_GAIN * map_info.map_cost + DISTANCE_GAIN * map_info.dis;

            if(cost < min_cost)
            {
                min_cost = cost;
                min_dr = map_info.dr;
                min_dc = map_info.dc;
                min_i = map_info.i;
                min_j = map_info.j;
            }
        }
    }

    geometry_msgs::PoseStamped adjust_local_goal;

    if(change_goal)
    {
        change_goal = false;
        enable_change = false;

        adjust_local_goal_index_x = divide_resolution * min_dr + min_i - column/2;
        adjust_local_goal_index_y = divide_resolution * min_dc + min_j - row/2;

        adjust_local_goal.pose.position.x = adjust_local_goal_index_x*std::cos(pi/2) - adjust_local_goal_index_y*std::sin(pi/2)*resolution;
        adjust_local_goal.pose.position.y = adjust_local_goal_index_y*std::cos(-pi/2) + adjust_local_goal_index_y*std::sin(-pi/2)*resolution;
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
}

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

