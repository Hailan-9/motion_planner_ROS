/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    a_star_gridmap_search.cpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-05
 */
#include "astar_gridmap_searcher/a_star_gridmap_search.hpp"
#include "astar_gridmap_searcher/astar.hpp"
#include "astar_gridmap_searcher/algorithm_timer.h"
using namespace std;




int main(int argc, char *argv[])
{
    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "astar_node");
    // 节点和参数私有，参考其命名空间与节点名称
    ros::NodeHandle nh("~");

    ROS_INFO("init----1");

    astar_PathPub = nh.advertise<nav_msgs::Path>("astar_path",10);
    // 可视化起点终点
    astar_Pub_Startpoint = nh.advertise<visualization_msgs::Marker>("start_point",1);
    astar_Pub_Goalpoint = nh.advertise<visualization_msgs::Marker>("goal_point",1);

    astar_MapSub = nh.subscribe("/map",1,mapCallback);// 订阅栅格地图
    initPoseSub = nh.subscribe("/initialpose",1,initPoseCallback);// 订阅起点
    goalPoseSub = nh.subscribe("/move_base_simple/goal",1,goalPoseCallback);// 订阅目标点

    ROS_INFO("init----2");
    
    ros::Rate loop_rate(10);
    ros::Duration(3).sleep();
    while (ros::ok())
    {
        ros::spin();
        loop_rate.sleep();
    }

    return 0;
}





/**************************函数定义********************************/

// 世界坐标系（也就是地图坐标系）-->栅格地图坐标系
Eigen::Vector2d world2Gridmap(double w_x, double w_y)
{
    // ROS_INFO("function---world2Gridmap----");
    Eigen::Vector2d result;
    if (w_x < origin_x || w_y < origin_y)
    {
        result << -1,-1;
        return result;
    }

    int temp_x = int((w_x - origin_x) / resolution);
    int temp_y = int((w_y - origin_x) / resolution);

    if (temp_x < width && temp_y < height)
    {
        result << temp_x,temp_y;
        return result;
    }
}
// 栅格地图坐标系-->世界坐标系（也就是地图坐标系）
Eigen::Vector2d gridmap2World(double gm_x, double gm_y)
{
    // ROS_INFO("function---gridmap2World----");
    Eigen::Vector2d result;
    if (gm_x > width || gm_y > height)
    {
        cout <<"-111111111111111111111111"<<endl;
        result << -1, -1;
        return result;
    }

    double temp_x = gm_x * resolution + origin_x;
    double temp_y = gm_y * resolution + origin_y;

    if (temp_x > origin_x && temp_y > origin_y)
    {
        result << temp_x, temp_y;
        return result;
    }
}
// 路径搜索是在（二维）栅格地图下进行搜索的
void startFindPath()
{
    ROS_INFO("startFindPath------------");
    // 获得起点 终点------栅格地图坐标系下
    Point astar_Start(startPoint(0),startPoint(1));
    
    cout <<"gridmap frame---start: "<<astar_Start.x<<","<<astar_Start.y<<endl;

    Point astar_Goal(goalPoint(0),goalPoint(1));

    cout <<"gridmap frame---goal: "<<astar_Goal.x<<","<<astar_Goal.y<<endl;

    cout <<"*****Start find path with Astar*****"<<endl;

    Astar astar;

    // 时间
    Algorithm_Timer algorithm_Timer;
    algorithm_Timer.begin();

    astar.InitAstar(mapData);
    list<Point*> path = astar.GetPath(astar_Start,astar_Goal,false);

    std::string astar_task = "Astar_search";
    algorithm_Timer.end(astar_task);
    
    Eigen::Vector2d temp_p;
    if (path.empty())
    {
        ROS_WARN("Astar算法没能找到路径-----");
    }
    for(const auto &p : path)
    {
        temp_p <<p->x, p->y;
        pathPoint.emplace_back(temp_p);
    }

    publishPath();
}



void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgPtr)
{
    ROS_INFO("mapppppppppppCallback-----------------");

    origin_x = msgPtr->info.origin.position.x;
    origin_y = msgPtr->info.origin.position.y;
    ROS_INFO("mapinfo-----------------");
    cout<<origin_x<<" "<<origin_y<<endl;

    resolution = msgPtr->info.resolution;
    width = msgPtr->info.width;
    height = msgPtr->info.height;

    for (int i = 0; i < height; i++)
    {
        vector<int> temp_v;

        for (int j = 0; j < width; j++)
        {
            temp_v.emplace_back( int(msgPtr->data[i*width + j]));
        }
        
        mapData.emplace_back(temp_v);
    }

    cout <<"MapInfo"<<endl;
    cout <<msgPtr->data.size()<<endl;;
    cout <<mapData.size()<<" "<<mapData[0].size()<<endl;
}

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgPtr)
{
    ROS_INFO("initCallback-----------------");
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = resolution * 2;
    node_vis.scale.y = resolution * 2;
    node_vis.scale.z = resolution * 2;

    geometry_msgs::Point pt;
    pt.x = msgPtr->pose.pose.position.x;
    pt.y = msgPtr->pose.pose.position.y;

    node_vis.points.emplace_back(pt);
    astar_Pub_Startpoint.publish(node_vis);

    startPoint = world2Gridmap(pt.x, pt.y);
    cout <<"worldmap frame---start: "<<pt.x<<","<<pt.y<<endl;
    if (startPoint(0) == -1 && startPoint(1) == -1)
    {
        ROS_WARN("please set the correct or valid startPoint again!!!");
    }

}
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgPtr)
{
    ROS_INFO("goalCallback-----------------");
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = resolution * 2;
    node_vis.scale.y = resolution * 2;
    node_vis.scale.z = resolution * 2;

    geometry_msgs::Point pt;
    pt.x = msgPtr->pose.position.x;
    pt.y = msgPtr->pose.position.y;

    node_vis.points.emplace_back(pt);
    astar_Pub_Goalpoint.publish(node_vis);

    goalPoint =world2Gridmap(pt.x, pt.y);
    cout <<"worldmap frame---goal: "<<pt.x<<","<<pt.y<<endl;
    if (goalPoint(0) == -1 && goalPoint(1) == -1)
    {
        ROS_WARN("please set the correct or valid goalPoint again!!!");
    }
    else
    {
        startFindPath();
    }
}

// 发布的轨迹用来可视化，需要从栅格地图坐标系转到世界地图坐标系上
void publishPath()
{
    ROS_INFO("start---PublishPath------------");

    nav_msgs::Path astarPathMsg;
    Eigen::Vector2d res_pos;
    for (int i = 0; i < pathPoint.size(); i++)
    {
        geometry_msgs::PoseStamped pathPose;

        // pathPose.header.frame_id = "odom"
        res_pos = gridmap2World(pathPoint[i](0),pathPoint[i](1));
        pathPose.pose.position.x = res_pos(0);
        pathPose.pose.position.y = res_pos(1);
        pathPose.pose.position.z = 0.0;


        // pathPose.pose.orientation.w = 1.0;

        astarPathMsg.header.stamp = ros::Time::now();
        astarPathMsg.header.frame_id = "odom";
        astarPathMsg.poses.emplace_back(pathPose);
    }
    cout <<"test--" << res_pos(0) <<" "<< res_pos(1)<<endl;
    
    astar_PathPub.publish(astarPathMsg);
    ROS_INFO("start---PublishPath-------overover-----");

}

