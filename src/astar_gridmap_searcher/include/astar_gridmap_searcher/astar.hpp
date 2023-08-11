#pragma once


/* A_star算法 hpp*/

#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <unordered_map>
#include <boost/functional/hash.hpp>

using namespace std;
namespace A_Star
{



/* 常量定义 */ 
/* -----------------------这些代价对搜索结果的影响很大----------------------- */
// 原来搜索速度很慢，找到原因了，原来的两个值是10 14 对于目标点，h相对于g很小，所以变成了dijstra算法，
// 而且写的好有点问题，所以一运行就很卡了，因为new了超级多
const float kCost_straight = 1;
const float kCost_diagonal = 1.4;
// 左移10位 使其无穷大
#define INFINITY 1 << 30
// 节点状态
// 被扩展的节点，也就是每次找出代价F最小的节点，存入closeset中
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
// 没有被探索的节点
#define NOT_EXPAND 'c'

enum 
{
    REACH_END = 1,
    NO_PATH = 2
};

// 变量均基于栅格地图
struct Point
{
    float x;
    float y;
    float F, G, H;
    Eigen::Vector2f pos;
    char point_State{};
    Point *parent;
    //有参构造函数  参数初始化列表---
    Point(float _x, float _y) : x(_x), y(_y), F(0), G(0), H(0), point_State(NOT_EXPAND),\
    parent(NULL)
    {
        pos <<x, y;
    }
    void set_pos()
    {
        pos <<x, y;
    }
    void set_pos(float p_x, float p_y)
    {
        x = p_x;
        y = p_y;
        pos <<p_x, p_y;
    }
    // 默认构造函数
    Point()
    {
        parent = NULL;
        point_State = NOT_EXPAND;
    }
    ~Point() {};
};

typedef Point* PointPtr;

// 优先级队列---priority_queue中，元素之间的比较（通常使用 < 运算符或提供的自定义比较器）用于确定元素的优先级关系,也就是优先级的判定标准。
// 在c++中，class和struct的唯一区别就是默认的访问权限不同，
// 成员的默认权限，class默认为私有，struct默认为公有,所以下面这个地方用struct或者class都可以,看个人习惯
/* ---第二种方法---自定义优先级队列的比较器,要使用重载函数---调用运算符 operator() 来实现自定义比较器*/
// 一般定义数据类型，使用struct；实现某项功能使用class
struct NodeComparator
{
    bool operator() (PointPtr node1, PointPtr node2)
    {
        // 第二个对象的元素 小于 第一个对象的元素时，才真，说明较小的值具有较高的优先级
        return node1->F > node2->F;
    }
};





// todolist---哈希矩阵 哈希值
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
    // 重载函数调用运算符，也称为仿函数
    std::size_t operator()(T const& matrix) const
    {
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i)
        {
            // 矩阵的存储一般列优先，也就是存完第一列的各行后存第二列的各行
            // matrix.data() 按照存储的顺序一个一个取值，比如列优先或者行优先
            auto elem = *(matrix.data() + i);
            // 异或，对应位不同返回1
            seed ^= std::hash<typename T::Scalar>()(elem) + 
            0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class NodeHashTable 
{
    private:
        // data
        // 实现无序映射的一个容器，其构造函数传值顺序 好像有点问题 这个问题还没有解决！！todolist 需要自己再理解一下
        std::unordered_map<Eigen::Vector2f, PointPtr, matrix_hash<Eigen::Vector2f>> data_2d;
        // std::unordered_map<Vec3<int>, PathNodePtr, matrix_hash<Vec3<int>>> data_3d;
    public:
        NodeHashTable(/* param */) {

        }
        ~NodeHashTable() {

        }

        void insert(Eigen::Vector2f idx, PointPtr node)
        {
            data_2d.insert(std::make_pair(idx, node));
        }
        // void insert(Vec2<int> idx, int time_idx, PathNodePtr node)
        // {
        //     data_3d.insert(std::make_pair(
        //         Eigen::Vector3i(idx(0),idx(1),time_idx), node));
        // }
        PointPtr find(Eigen::Vector2f idx)
        {
            auto iter = data_2d.find(idx);
            return iter == data_2d.end() ? NULL : iter->second;
        }
        // PathNodePtr find(Vec2<int> idx, int time_idx)
        // {
        //     auto iter = data_3d.find(
        //         Eigen::Vector3i(idx(0), idx(1), time_idx));
        //     return iter == data_3d.end() ? NULL : iter->second;
        // }

        void clear()
        {
            data_2d.clear();
            // data_3d.clear();
        }
};





class Astar
{
    public:
        Astar() {};//构造函数声明和定义，空定义
        ~Astar();//析构函数声明
        void setParam(const ros::NodeHandle& nh);
        //搜索路径 因为searchPath函数需要修改openlist和closelist 所以不能是常量成员函数 也就是不能在最后+ const
        int searchPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner);
        // 得到只有位置的路径点
        vector<Eigen::Vector2f>  GetPath();
        void InitAstar(const vector<vector<int>> &_map);
        void init();
        void reset();
        //isIgnoreCorner 是否可以绊住情况下对角线通过


    private:

        //得到G
        float getG(Point *parentPoint, Point *curPoint);
        //得到H
        float getH(Point *curPoint, Point *endPoint);
        //得到F
        float getF(Point *curPoint);

        //判断当前节点的邻居节点是否可以到达
        bool isCanreach(const Point* curPoint,const Eigen::Vector2f& targetPoint, bool isIgnoreCorner) const;
        void retrievePath(PointPtr end_node);



        /* ------------------main data structure------------------ */
        int use_node_num;// 路径搜索中实际探索的节点数
        int iter_num;// 迭代搜索的次数 也就是最外层循环次数
        // 将被探索过的节点均存入其中，目的是方便查找
        NodeHashTable expanded_nodes;
        //地图 使用二维数组来表示 100表示障碍物-黑色 -1表示未知情况-灰色 0表示无障碍物可通行-白色
        vector<vector<int>> map1;
        // 优先级队列 F值也就是代价越小优先级越高！
        std::priority_queue<PointPtr, vector<PointPtr>, NodeComparator> openList;
        // 路径搜索过程中的节点资源池，里面存放了分配数量的指向节点类型的指针，使用new开辟的
        // 若是搜索过程中，用的节点数量超过了资源池的容量，那么就是run out of memory
        vector<PointPtr> path_node_pool;
        // 存储前端搜索得到的路径点 point类型
        vector<PointPtr> path_nodes;

        /* ------------------parameter------------------ */
        double weight_g;
        double weight_h;
        // 资源池分配的容量，也就是手动开辟节点指针的个数 new
        int allocate_num;

};


}