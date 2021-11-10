#include "vortexbot/trajOnmi_generate.h"


TrajOnmiGenerate::TrajOnmiGenerate()
{
    TrajOnmiGenerate(500,0.05);
}
TrajOnmiGenerate::TrajOnmiGenerate(int n,double sample_time)
{
    ref_time_.resize(n);
    ref_traj_.resize(n);
    for(int i=0;i<n;i++)
    {
        // ref_time_[i]=i*sample_time;
        // ref_traj_[i].x=25*sin(0.2*ref_time_[i]);
        // ref_traj_[i].y=25+10-25*cos(0.2*ref_time_[i]);
        // ref_traj_[i].phi=0.2*ref_time_[i];
        // ref_traj_[i].vel=5;
        // ref_traj_[i].delta=0.103627;
        // ref_time_[i]=i*sample_time;
        // ref_traj_[i].x=25*sin(0.2*ref_time_[i]);
        // ref_traj_[i].y=25*0.2*ref_time_[i];
        // ref_traj_[i].phi=0;
        // ref_traj_[i].vel=5;
        // ref_traj_[i].delta=0;
            ref_time_[i]=i*sample_time;
            ref_traj_[i].x=25*sin(0.2*ref_time_[i]);
            ref_traj_[i].y=25+10-25*cos(0.2*ref_time_[i]);
            ref_traj_[i].phi=0.2*ref_time_[i];
            ref_traj_[i].vx=5;
            ref_traj_[i].vy=0;
             ref_traj_[i].vw=0;
    }
}

void TrajOnmiGenerate::getTraj(vector<trajOnmi> & traj1, vector<double> & time1)
{
    //traj1.clear();
    //time1.clear();
    traj1.resize(ref_time_.size());
    time1.resize(ref_time_.size());
    for(int i=0;i<ref_time_.size();i++)
    {
        time1[i]=ref_time_[i];
        traj1[i]=ref_traj_[i];
    }

}