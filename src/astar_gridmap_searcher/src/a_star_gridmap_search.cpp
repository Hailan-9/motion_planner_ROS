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

#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include <fstream>


using namespace std;
using namespace A_Star;
std::ofstream outfile;
std::ofstream outfile1;

bool init_finish = false;
#define Test_heuristic_functions_separately

// 定义全局变量
Astar astar;


int main(int argc, char *argv[])
{

    // 创建输出文件
    time_t currentTime;
    time(&currentTime);
    currentTime = currentTime + 8 * 3600; //	格林尼治标准时间+8个小时
    tm *t = gmtime(&currentTime);	
    string filename = "/home/zs/motion_planner/record/data" + to_string(t->tm_mon + 1) + "-" +to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + "-" + to_string(t->tm_min) + ".txt";
    string filename1 = "/home/zs/motion_planner/record/data_visitednode" + to_string(t->tm_mon + 1) + "-" +to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + "-" + to_string(t->tm_min) + ".txt";
    
    outfile.open(filename.c_str());
    outfile1.open(filename1.c_str());



    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "astar_node");
    // 话题和参数私有，参考其命名空间与节点名称！
    ros::NodeHandle nh("~");

    ROS_INFO("init----1");


    astar_PathPub = nh.advertise<nav_msgs::Path>("astar_path",1);
    // 可视化起点终点
    astar_Pub_Startpoint = nh.advertise<visualization_msgs::Marker>("start_point",1);
    astar_Pub_Goalpoint = nh.advertise<visualization_msgs::Marker>("goal_point",1);

    astar_MapSub = nh.subscribe("/map",1,mapCallback);// 订阅栅格地图
    initPoseSub = nh.subscribe("/initialpose",1,initPoseCallback);// 订阅起点
    goalPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(goalPoseCallback, _1, nh));// 订阅目标点

    ROS_INFO("init----2");
    
    ros::Rate loop_rate(10);
    ros::Duration(4).sleep();
    while (ros::ok())
    {
        ros::spin();
        loop_rate.sleep();
    }
    

    return 0;
}





/**************************函数定义********************************/



// 路径搜索是在（二维）栅格地图下进行搜索的，不同地图坐标系之间的转换在astar源文件中完成。
// 本cpp中位置均是世界地图坐标系下的，即传给astar.cpp和接受该cpp的数据
// 本源文件中涉及到坐标均是世界地图坐标系下
void startFindPath(ros::NodeHandle& nh)
{
    ROS_INFO("startFindPath------------");
    // 获得起点 终点------世界地图坐标系下
    Point astar_Start;
    astar_Start.set_w_pos(startPoint);
    
    Point astar_Goal;
    astar_Goal.set_w_pos(goalPoint);


    // 时间
    Algorithm_Timer algorithm_Timer;
    algorithm_Timer.begin();
    astar.setParam(nh);

    if (!init_finish)
    {
        astar.InitAstar(mapData,map_origin);
        astar.init();
        init_finish = true;
    }

    astar.reset();

    int search_success = astar.searchPath(astar_Start, astar_Goal,1);

    if(search_success == REACH_END)
    {
        pathPoint = astar.GetPath();

        std::string astar_task = "Astar_search";
        algorithm_Timer.end(astar_task);
    }
    else
    {
        ROS_WARN("failed to search-----------");
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
    map_size <<msgPtr->info.width, msgPtr->info.height;
    map_origin << msgPtr->info.origin.position.x, msgPtr->info.origin.position.y;

    // 地图宽
    for (int i = 0; i < map_size(0); i++)
    {
        vector<int> temp_v;
        // 地图高
        for (int j = 0; j < map_size(1); j++)
        {
            temp_v.emplace_back( int(msgPtr->data[j*map_size(0) + i]));
        }
        mapData.emplace_back(temp_v);
    }

    // 输出data信息
    for (int i(0); i < map_size(1); i++)
    {
        for (int j(0); j < map_size(0); j++)
        {
            outfile1<< int(msgPtr->data[i*map_size(0) + j]) <<" ";
        }
        outfile1<<endl;
    }


    cout <<"MapInfo"<<endl;
    cout <<msgPtr->data.size()<<endl;
    cout <<map_size(0) <<" "<<map_size(1)<<endl;
    cout <<mapData.size()<<" "<<mapData[0].size()<<endl;
}

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgPtr)
{
    ROS_INFO("initCallback-----------------");
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.type = visualization_msgs::Marker::POINTS;
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

    node_vis.scale.x = resolution * 0.5;
    node_vis.scale.y = resolution * 0.5;
    node_vis.scale.z = resolution * 0.001;

    geometry_msgs::Point pt;
    pt.x = msgPtr->pose.pose.position.x;
    pt.y = msgPtr->pose.pose.position.y;

    node_vis.points.emplace_back(pt);
    astar_Pub_Startpoint.publish(node_vis);
    // 世界地图坐标系下
    startPoint <<pt.x, pt.y;
    cout <<"worldmap frame---start: "<<pt.x<<","<<pt.y<<endl;
    if (startPoint(0) <map_origin(0) && startPoint(1) < map_origin(1))
    {
        ROS_WARN("please set the correct or valid startPoint again!!!");
    }

}
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msgPtr, ros::NodeHandle& nh)
{
    ROS_INFO("goalCallback-----------------");
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    node_vis.type = visualization_msgs::Marker::POINTS;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 1;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = resolution * 0.5;
    node_vis.scale.y = resolution * 0.5;
    node_vis.scale.z = resolution * 0.001;

    geometry_msgs::Point pt;
    pt.x = msgPtr->pose.position.x;
    pt.y = msgPtr->pose.position.y;

    node_vis.points.emplace_back(pt);
    astar_Pub_Goalpoint.publish(node_vis);

    goalPoint <<pt.x, pt.y;
    cout <<"worldmap frame---goal: "<<pt.x<<","<<pt.y<<endl;\

    #ifndef Test_heuristic_functions_separately
        if (goalPoint(0) < map_origin(0) && goalPoint(1) < map_origin(1))
        {
            ROS_WARN("please set the correct or valid goalPoint again!!!");
        }
        else
        {
            startFindPath(nh);
        }

    #else
        startPoint <<1, 4;
        goalPoint <<5, 6;
        startFindPath(nh);

    #endif



}

// 发布的轨迹用来可视化，需要从栅格地图坐标系转到世界地图坐标系上
void publishPath()
{
    ROS_INFO("start---PublishPath------------");

    nav_msgs::Path astarPathMsg;

    for (int i = 0; i < pathPoint.size(); i++)
    {
        geometry_msgs::PoseStamped pathPose;


        pathPose.pose.position.x = pathPoint[i](0);
        pathPose.pose.position.y = pathPoint[i](1);
        pathPose.pose.position.z = 0.0;


        astarPathMsg.header.stamp = ros::Time::now();
        astarPathMsg.header.frame_id = "odom";
        astarPathMsg.poses.emplace_back(pathPose);
    }

    
    astar_PathPub.publish(astarPathMsg);
    ROS_INFO("start---PublishPath-------overover-----");

}

