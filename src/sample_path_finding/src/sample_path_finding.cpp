/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    sample_path_finding.cpp
 * @brief   描述
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-28
 */

#include "sample_path_finding/rrt.hpp"
#include "sample_path_finding/sample_path_finding.hpp"
#include "astar_gridmap_searcher/algorithm_timer.h"


#include <time.h>
#include <stdio.h>
#include <sys/time.h>
#include <fstream>


std::ofstream outfile_rrt;
using namespace std;
// #define Test_separately

// 定义全局变量
RRT rrt;
bool init_finish = false;
string sample_path_find_choice;

int main(int argc, char *argv[])
{
    // 创建输出文件
    time_t currentTime;
    time(&currentTime);
    currentTime = currentTime + 8 * 3600; //	格林尼治标准时间+8个小时
    tm *t = gmtime(&currentTime);	
    string filename = "/home/zs/sample_path_finding/record/datalog" + to_string(t->tm_mon + 1) + "-" +to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + "-" + to_string(t->tm_min) + ".txt";
    // string filename1 = "/home/zs/motion_planner/record/data_visitednode" + to_string(t->tm_mon + 1) + "-" +to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + "-" + to_string(t->tm_min) + ".txt";
    
    outfile_rrt.open(filename.c_str());
    // outfile1.open(filename1.c_str());



    /* code */
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "rrt_node");
    // 话题和参数私有，参考其命名空间与节点名称！
    ros::NodeHandle nh("~");
    // 在 C++ 中，虽然字符串字面量的类型是 const char[]，但由于 C++ 标准库中提供了 std::string 类型，C++ 
    // 允许在一些上下文中将字符串字面量隐式转换为 std::string 类型。这是通过调用 std::string 的构造函数来实现的。
    // 比如const char* cstr = "hello"; // 字符串字面量
    // std::string str = cstr;     // 隐式转换为 std::string
    // 在 C++ 中，字符串字面量的类型是 const char[]。这意味着字符串字面量被解释为一个以空字符 '\0' 结尾的字符数组。这也被称为 C 风格字符串。
    // string("rrt") 这个表达式的目的是使用 std::string 构造函数将 "rrt" 字符串字面量转换为一个 std::string 对象

    ROS_INFO("init----1");


    rrt_PathPub = nh.advertise<nav_msgs::Path>("rrt_path",1);
    // 可视化起点终点
    rrt_Pub_Startpoint = nh.advertise<visualization_msgs::Marker>("start_point",1);
    rrt_Pub_Goalpoint = nh.advertise<visualization_msgs::Marker>("goal_point",1);

    rrt_MapSub = nh.subscribe("/map",1,mapCallback);// 订阅栅格地图
    initPoseSub = nh.subscribe("/initialpose",1,initPoseCallback);// 订阅起点
    goalPoseSub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, boost::bind(goalPoseCallback, _1, nh));// 订阅目标点

    ROS_INFO("init----2");

    ros::Duration(4).sleep();
    while (ros::ok())
    {
        // 回头函数，响应各回调函数
        ros::spin();
    }
    
    return 0;
}










/* *************************函数定义************************* */

// 基于采样的算法，采样点在世界坐标系中
void startFindPath(ros::NodeHandle& nh)
{
    ROS_INFO("start---Find---Path------------");
    // 获得起点 终点------世界地图坐标系下
    Point rrt_Start;
    rrt_Start.set_w_pos(startPoint);
    
    Point rrt_Goal;
    rrt_Goal.set_w_pos(goalPoint);


    // 时间
    Algorithm_Timer algorithm_Timer;
    algorithm_Timer.begin();

    rrt.setParam(nh);

    if (!init_finish)
    {
        rrt.init(mapData,map_origin);
        init_finish = true;
    }
    cout <<"test rrt"<<endl;
    rrt.reset(); 
    
    int search_success;
    nh.param("sample_path_find_choice",sample_path_find_choice,string("rrt"));

    if(sample_path_find_choice == string("rrt"))
    {
        cout<<sample_path_find_choice<<endl;

        search_success = rrt.pathFind(rrt_Start, rrt_Goal);
    }
    else if(sample_path_find_choice == string("rrtStar"))
    {
        cout<<sample_path_find_choice<<endl;
        // search_success = rrt.jpsSearchPath(rrt_Start, rrt_Goal,1);
    }
    else
    {

    }

    if(search_success == REACH_END)
    {
        pathPoint = rrt.getPath();

        std::string rrt_task = "rrt_search";
        algorithm_Timer.end(rrt_task);
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
    rrt_Pub_Startpoint.publish(node_vis);
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
    outfile_rrt <<"*************************new start************************" << endl;
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
    rrt_Pub_Goalpoint.publish(node_vis);

    goalPoint <<pt.x, pt.y;
    cout <<"worldmap frame---goal: "<<pt.x<<","<<pt.y<<endl;\

    #ifndef Test_separately
        if (goalPoint(0) < map_origin(0) && goalPoint(1) < map_origin(1))
        {
            ROS_WARN("please set the correct or valid goalPoint again!!!");
        }
        else
        {
            startFindPath(nh);
        }
        cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    #else
        startPoint <<1.91, 5.68;
        goalPoint <<3.27, 6.38;
        startFindPath(nh);

    #endif



}

// 发布的轨迹用来可视化，需要从栅格地图坐标系转到世界地图坐标系上
void publishPath()
{
    ROS_INFO("start---PublishPath------------");

    nav_msgs::Path rrtPathMsg;

    for (int i = 0; i < pathPoint.size(); i++)
    {
        geometry_msgs::PoseStamped pathPose;


        pathPose.pose.position.x = pathPoint[i](0);
        pathPose.pose.position.y = pathPoint[i](1);
        pathPose.pose.position.z = 0.0;

        rrtPathMsg.header.stamp = ros::Time::now();
        rrtPathMsg.header.frame_id = "odom";
        rrtPathMsg.poses.emplace_back(pathPose);
    }

    
    rrt_PathPub.publish(rrtPathMsg);
    ROS_INFO("start---PublishPath-------overover-----");

}