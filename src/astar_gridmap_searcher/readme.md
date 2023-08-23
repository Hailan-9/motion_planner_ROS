
# 项目介绍
基于ROS，采用c++编码，实现了Astar和JPS路径搜索算法，并进行了可视化显示。

## Table of Contents

- [Astar](#Astar)
	- [具体细节](#具体细节)
	- [算法笔记](#算法笔记)
	- [名词细节](#名词细节)
	- [Heuristic design](#Heuristic-design)
	- [工程优化 trick](#工程优化-trick)
	- [Dijkstra VS A*](#Dijkstra-VS-A*)
	- [算法测试](#算法测试)


- [JPS](#JPS)
	- [算法流程&细节](#算法流程&细节)
	- [相关概念](#相关概念)
	- [参考](#参考)

## Astar
### 具体细节
1、将几种主流的启发式函数均加入其中，可以适用于不同的地图环境
2、参考融合了一些fastplanner中Astar算法中的一些工程上的优秀代码实现，比如优先级队列、哈希表等
3、g和h均加入了权值，可以实现多种算法，即f = a\*g + b\*h。a b区别不同，算法也就不同。比如0 1，这时变成了most greedy；1 kesei(>1) 变成了tunable greediness等

### 算法笔记

### 名词细节
加入过openlist中的所有节点，无论后面迭代过程有无被弹出，均称为被访问过的或者被探索过的 visited
加入过openlist后被弹出来的节点，被称为被扩展的节点 expanded节点
也可以这样理解：
expanded表示开放列表中所有访问过的节点，也就是openlist里被弹出的节点
visited表示开放列表中还未访问到的节点，也就是openlist里还未被弹出的节点
加入openlist的节点经历两个状态 inopenlist(visited) ---> incloselist
(expanded)

### Heuristic design
欧几里得距离也就是二范数  总是满足最优性 admissible
曼哈顿距离也就是一范数 看情况 若机器人可以对角线移动，则不是admissible；若只能前后左右移动，是admissible

### 工程优化 trick
tie breaker

### Dijkstra VS A*
dijkstra的优点是完备性、最优性，缺点是没有目标和方向，搜索速度很慢。
A*相当于是在前者的基础上加了启发式函数，有了一定的目标和方向。并且若h（n）<=h（n）\*，则最优性是满足的。

1. 若h=0，A*则退化为dijkstra
2. 如果h（n）<< << h(n)\*，A\*算法可以找到最短路径（也就是满足了最优性）,但是搜索效率很低，也就是目标导向性不是很强，这样的话会导致搜索的节点变多，也就导致了搜索时间变长。
3. 当h（n）== h（n）\*时，A\*到达了最优状态，最优性和快速性同时满足！
4. 若h（n）>> >> g（n），那么这时候f主要取决于h（n），A*算法退化为贪婪最佳优先搜索算法 greedy best first search。h权值越大，搜索目的性越强，效率更高，但是很容易陷入局部最优中。


### 算法测试
使用对角距离作为启发函数，在有障碍物的情况下，效果并不是很好，


## JPS

Dijkstra算法和A\*算法的搜索结果，可以发现两者搜索到的路径并不相同，也就是长的不一样，但是路径长度是一样的，（这样认为就是同一条路径）这就是路径的对称性，Dijkstra和A*都会探索很多条对称的路径，而JPS就是专门用来打破对称性的，只选择其中一个路径。
### 算法流程&细节

### 相关概念
劣质邻居节点 inferior neighbors
自然邻居节点 natural neighbors
强制邻居节点 forced neighbors

两个重要的规则
look ahead rule
jumping rule
<!-- 对于对角线跳跃规则而言： -->
跳跃时：
先考察---进行水平和垂直方向的跳跃，再考察---进行对角方向的跳跃。

A*和JPS几乎没有区别，算法总体流程是一样的，在A\*里使用的技巧和启发式函数等，都可以在JPS里使用。唯一的差别是扩展邻居节点这个地方。

在多障碍物复杂环境下，JPS优于A*算法，因为JPS可以很快地查询障碍物的情况。

### 参考
1. [Online Graph Pruning for Pathfinding on Grid Maps 基于栅格地图的在线图表修剪路径搜索](https://blog.csdn.net/weixin_47689403/article/details/126914206)
2. [路径规划 | 图搜索算法：JPS](https://blog.csdn.net/qq_42688495/article/details/116019557)
3. [JPS 跳点算法源码分析](https://blog.csdn.net/qq_40606107/article/details/120789713)
4. [jps3d](https://github.com/KumarRobotics/jps3d)
5. [jps在线可视化及原理介绍](https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html)
6. [JPS/JPS+ 寻路算法](https://www.cnblogs.com/KillerAery/p/12242445.html)
7. [每周一篇论文-规划算法Jump Point Search-Online Graph Pruning for Pathfinding on Grid Maps](https://blog.csdn.net/weixin_43673156/article/details/127720056)

## 在线开源资料
[图搜索经典算法在线可视化](http://qiao.github.io/PathFinding.js/visual/)
[Introduction to the A* Algorithm](https://www.redblobgames.com/pathfinding/a-star/introduction.html)