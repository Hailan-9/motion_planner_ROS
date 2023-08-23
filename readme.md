# motion-planner-ROS
记录学习机器人、无人车运动规划与控制算法的历程，实现的算法均基于ROS1进行了可视化。

## 介绍
本仓库主要对各种经典的无人车、轮式移动机器人运动规划与控制算法进行了ROS、C++的实现。

**功能介绍**
- 可以通过Rviz选择2D空间点，然后自动生成光滑的多项式曲线；
- 具有前端轨迹搜索，到后端光滑轨迹生成功能。

**功能包介绍：**
- astar_gridmap_searcher: 在栅格地图上，实现astar和jps算法的前端路径搜索
- 
- 


## 编译

开发环境为：ubuntu20.04、ROS noetic

## 运行
**astar或jps前端路径搜索**
```shell
roslaunch astar_gridmap_searcher run_Astar_searcher.launch search_choice:="astar"

roslaunch astar_gridmap_searcher run_Astar_searcher.launch search_choice:="jps"
```
## 致谢

