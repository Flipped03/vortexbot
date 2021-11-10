#pragma once

#include<iostream>
#include<vector>
#include<algorithm>
#include<string>
#include"Eigen/Core"
#include<cmath>

using namespace std;


struct trajOnmi{
    double x;
    double y;
    double phi;
    double vx;
    double vy;
    double vw;
};


class TrajOnmiGenerate{
    public:
        //构造函数
        TrajOnmiGenerate();
        TrajOnmiGenerate(int n,double sample_time);

        //返回轨迹
        void getTraj(vector<trajOnmi> & traj1,vector<double> & time1);
    
    private:
        vector<trajOnmi>  ref_traj_;
        vector<double>  ref_time_;
};