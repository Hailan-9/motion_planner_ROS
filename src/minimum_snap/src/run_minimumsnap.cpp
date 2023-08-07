
#include <iostream>
#include "minimum_snap/minimumsnap.hpp"
#include "minimum_snap/cppTypes.h"
#include <fstream>
#include <string>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <sys/time.h>


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>



using namespace std;
/* 文本输出 */
std::ofstream outfile;


/* 总时间 */
struct timeval startT, endT;
long long plan_total_time;




int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "minimumsnap_traj_generator");
    ros::NodeHandle nh;

    ros::Publisher minimumsnap_Pub = nh.advertise<nav_msgs::Path>("minimumsnap_msg",10);

    nav_msgs::Path path;
    geometry_msgs::PoseStamped poseStamped;

    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();


    





	// 创建输出文件
	time_t currentTime;
	time(&currentTime);
	currentTime = currentTime + 8 * 3600; //	格林尼治标准时间+8个小时
	tm *t = gmtime(&currentTime);	
	string filename = "/home/zs/motion_planner/src/minimum_snap/record/data" + to_string(t->tm_mon + 1) + "-" +to_string(t->tm_mday) + "-" + to_string(t->tm_hour) + "-" + to_string(t->tm_min) + ".txt";
	outfile.open(filename.c_str());




    gettimeofday(&startT, NULL);
    MinimumSnap *minimumSnap_Test = new MinimumSnap();
    const int order = 3;
    //每维数据用一行
    DMat<double> wayPoint(3,4);
    DMat<double> _start_end_State(3,2*order);
    // _start_end_State<<10.0, 0.0, 0.0,0.0,  30.0, 0.0, 0.0, 0.0,
    //                   0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0,0.0,
    //                   0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0,0.0;
    _start_end_State<<10.0, 0.0, 0.0,  30.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0,   0.0, 0.0, 0.0;
    
    cout<<"++++++"<<endl;
    cout<<_start_end_State<<endl;
    cout<<"++++++"<<endl;


    // 包含起点---终点---中间点在内的所以经过的路标点
    wayPoint<<  10.0, 20,  25.0,  30.0,
                0.0,  2,   5,    0.0,
                0.0,  0.2,   0.4,   0.0;
    double total_Time = 1.0;
    
    minimumSnap_Test->SolveQp(wayPoint, _start_end_State, order, 1.0, 10, 5);

    //轨迹输出到文件
    minimumSnap_Test->PublishTrajectory();

    gettimeofday(&endT, NULL);
    plan_total_time = (endT.tv_sec - startT.tv_sec)*1000000 + (endT.tv_usec - startT.tv_usec);
    cout<<"plan_Total_Time: "<<plan_total_time/1000.0<<endl;

    for(int i = 0; i < minimumSnap_Test->pos_List.size();i++)
    {
        outfile<<"pos: "<<minimumSnap_Test->pos_List[i][0]<<" "<<minimumSnap_Test->pos_List[i][1]
        <<" "<<minimumSnap_Test->pos_List[i][2]<<endl;
        
        poseStamped.pose.position.x = minimumSnap_Test->pos_List[i](0);
        poseStamped.pose.position.y = minimumSnap_Test->pos_List[i](1);
        poseStamped.pose.position.z = minimumSnap_Test->pos_List[i](2);

        path.poses.emplace_back(poseStamped);


        outfile<<"vel: "<<minimumSnap_Test->vel_List[i][0]<<" "<<minimumSnap_Test->vel_List[i][1]
        <<" "<<minimumSnap_Test->vel_List[i][2]<<endl;

        outfile<<"acc: "<<minimumSnap_Test->acc_List[i][0]<<" "<<minimumSnap_Test->acc_List[i][1]
        <<" "<<minimumSnap_Test->acc_List[i][2]<<endl;
    }

    //休眠时间  给发布者节点在master中注册的时间
    ros::Duration(2).sleep();
    ros::Rate rate(1);

    while (ros::ok())
    {
        minimumsnap_Pub.publish(path);
        rate.sleep();
        ros::spinOnce();
        ROS_INFO("pub---trajectory");
    }




    return 0;

}


