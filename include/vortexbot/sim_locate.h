#pragma once

#include<iostream>
#include<vector>
#include<algorithm>
#include<string>
#include<cmath>
#include"vortexbot/traj_generate.h"
#include"vortexbot/trajOnmi_generate.h"

using namespace std;

class SimLocate
{
    public:
        SimLocate(double car_length,traj init_pos);
        //@brief 利用euler方法进行模拟定位
        //@param 控制量（速度，前轮偏角），采样时间
        void updateEulerPosition(double vel,double delta,double sample_time);

        //@brief 利用rungeKutta方法进行模拟定位
        //@param 控制量（速度，前轮偏角），采样时间
        void updateRungeKuttaPosition(double vel,double delta,double sample_time);

        //@brief 获得机器人当前所在位置
        //@param 用于保存当前位置
        //@return 返回成功的标志
        bool getCurrentPosition(traj & current_state);

        //@brief 获得机器人从初始位置到当前位置的所有轨迹
        //@param 用于保存当前真实轨迹        
        void getRealTraj(vector<traj>& real_traj);

        //机器人的真实轨迹
        vector<traj>  real_traj_;
        
    private:



        //车身长度
        double car_length_;
};

class SimLocateOnmi
{
    public:
        SimLocateOnmi(trajOnmi init_pos);
        //@brief 利用euler方法进行模拟定位
        //@param 控制量（速度，前轮偏角），采样时间
        void updateEulerPosition(double vx,double vy,double vw,double sample_time);

        //@brief 利用rungeKutta方法进行模拟定位
        //@param 控制量（速度，前轮偏角），采样时间
        void updateRungeKuttaPosition(double vx,double vy,double vw,double sample_time);

        //@brief 获得机器人当前所在位置
        //@param 用于保存当前位置
        //@return 返回成功的标志
        bool getCurrentPosition(trajOnmi & current_state);

        //@brief 获得机器人从初始位置到当前位置的所有轨迹
        //@param 用于保存当前真实轨迹        
        void getRealTraj(vector<trajOnmi>& real_traj);

        //机器人的真实轨迹
        vector<trajOnmi>  real_traj_;
        
    private:

};