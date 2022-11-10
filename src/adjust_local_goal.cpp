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

    tf_listener = new tf2_ros::TransformListener(tf_buffer);

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
    row = local_map.info.width;
    resolution = local_map.info.resolution;
    divide_resolution = row / divide;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "local_map";
    static_transformStamped.child_frame_id = "base_link_from_local_map";
    static_transformStamped.transform.translation.x =row/2 * resolution;
    static_transformStamped.transform.translation.y =column/2 * resolution;
    static_transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, M_PI/2);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tf_buffer.lookupTransform("local_map", "base_link_from_local_map",
                                                         ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }

    geometry_msgs::PoseStamped local_goal_from_map;
    tf2::doTransform(local_goal, local_goal_from_map, transformStamped);

    local_goal_index_x = int(local_goal_from_map.pose.position.x/resolution);
    local_goal_index_y = int(local_goal_from_map.pose.position.y/resolution);

    if(local_goal_index_y < 0.0 || local_goal_index_x < 0.0)
    {
        return;
    }

    float min_cost = 5e10;

    if(local_map.data[local_goal_index_y*row + local_goal_index_x] == 100)
    {
       ROS_INFO_STREAM("change goal");
       change_goal = true;

        for(int dr=0; dr<divide; dr++)
        {
            for(int dc=0; dc<divide; dc++)
            {
                map_cost = 0.0;
                min_dis = 5e5;
                for(int i=0; i<divide_resolution; i++)
                {
                    for(int j=0; j<divide_resolution; j++)
                    {
                        if(local_map.data[(divide_resolution*dr+i)*row+(divide_resolution*dc+j)] == 0)
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

                        _map_cost = local_map.data[(divide_resolution*dr+i)*row+(divide_resolution*dc+j)];
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

            }
        }


        for(const auto& map_info : map_infos)
        {
            cost = MAP_COST_GAIN * map_info.map_cost + DISTANCE_GAIN * map_info.dis;
            enable_change = true;

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
    map_infos.clear();

    if(change_goal && enable_change)
    {
        change_goal = false;
        enable_change = false;

        geometry_msgs::PoseStamped adjust_local_goal_from_map;

        geometry_msgs::PoseStamped adjust_local_goal;
        adjust_local_goal_from_map.pose.position.x = (divide_resolution * min_dr + min_i)*resolution;
        adjust_local_goal_from_map.pose.position.y = (divide_resolution * min_dc + min_j)*resolution;

        tf2::Transform transform;
        tf2::convert(transformStamped.transform, transform);
        geometry_msgs::Transform tf;
        tf2::convert(transform.inverse(), tf);
        transformStamped.transform = tf;

        tf2::doTransform(adjust_local_goal_from_map, adjust_local_goal, transformStamped);
        adjust_local_goal.pose.position.z = local_goal.pose.position.z;
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
    local_map_updated = false;
    local_goal_updated = false;
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

