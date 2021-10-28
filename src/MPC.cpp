#include<iostream>
#include"mpc_control.h"
#include"traj_generate.h"
#include<vector>

using namespace std;

int main()
{
    double sample_time=0.05;
    int simulate_N=500;
    int Np=10,Nc=10;
    double car_length=2.6;

    traj init_pos{1.0,5.0,0.0,0.0,0.0};

    //生成轨迹
    TrajGenerate traj_generator(simulate_N,sample_time);
    vector<traj> traj_ref;
    vector<double> time_ref;
    traj_generator.getTraj(traj_ref,time_ref);

    //初始化MPC控制器
    MPCControl mpc_controller(sample_time,car_length,Np,Nc);
    //mpc_controller.updateMatrix(traj1[10].vel,traj1[10].phi,traj1[10].delta);
    //mpc_controller.qpSlover();
    
    //用于存放当前的位置
    traj current_pos;

    //用于存放最优控制量
    double control_vel;
    double control_delta;

    vector<traj> errors_ref_traj;
    errors_ref_traj.resize(Np);


        //mpc_controller.updateErrorsRefTraj(errors_ref_traj);
        
        //mpc_controller.updateMatrix(traj_ref[i].vel,traj_ref[i].phi,traj_ref[i].delta);
        mpc_controller.qpSlover();

        mpc_controller.getFirstControl(control_vel,control_delta);
    
        

    vector<traj> traj_real;
    //sim_locater.getRealTraj(traj_real);
    cout<<endl<<endl<<endl<<endl;
    cout<<"this is difference between traj_real and traj_ref : "<<endl;

    for(int j=0;j<traj_ref.size();j++)
        cout<<traj_real[j].x-traj_ref[j].x<<" "<<traj_real[j].y-traj_ref[j].y<<" "<<endl;
           //cout<<traj_real[j].x<<" "<<traj_ref[j].x<<" "<<endl;
    

    vector<double> real_control_vel;
    vector<double> real_control_delta;
    
    mpc_controller.getRealControl(real_control_vel,real_control_delta);
    //cout<<real_control_vel.size();
    //for(int j=0;j<real_control_vel.size();j++)
        //cout<<real_control_vel[j]<<" "<<real_control_delta[j]<<" "<<endl;

    return 0;
}

//参考网址
//https://blog.csdn.net/qq_42258099/article/details/95353986  自动驾驶——模型预测控制（MPC）理解与实践
//https://github.com/Flipped03/mpc_test/blob/master/traj_tracking/src/main.cpp
//https://zhuanlan.zhihu.com/p/141871796