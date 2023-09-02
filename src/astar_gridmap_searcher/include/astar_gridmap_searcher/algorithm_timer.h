/**
 * @copyright Copyright (c) 2023-hailan
 * 
 * @file    algorithm_timer.h
 * @brief   计算各种算法耗费的时间，作为优化的耗时参考
 * @author  hailan(https://github.com/Hailan-9)
 * @version 0.1
 * @date    2023-08-05
 */

#pragma once

#include <string>
#include <iostream>
#include <chrono>

class Algorithm_Timer
{
    public:
        Algorithm_Timer()
        {
            begin();
        }

        void begin()
        {
            start_ = std::chrono::system_clock::now();
        }
        void end(const std::string& task_name)
        {
            end_ = std::chrono::system_clock::now();
            std::chrono::duration<double> use_time = end_ - start_;

            std::cout.precision(3);
            std::cout << task_name << " algorithm "<<" use time(ms): " << use_time.count()*1000 <<std::endl;
        }
        double end() 
        {
            end_ = std::chrono::system_clock::now();
            std::chrono::duration<double> use_time = end_ - start_;
            return use_time.count() * 1000.0;
        }


    private:
        std::chrono::time_point<std::chrono::system_clock> start_, end_;


};