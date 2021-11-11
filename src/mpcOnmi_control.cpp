#include<iostream>
#include"vortexbot/mpcOnmi_control.h"
#include"vortexbot/mpc_control.h"
#include "Eigen/LU"

using namespace std;

// Eigen::MatrixXd powMatrix(Eigen::MatrixXd ma,int n)
// {
//     Eigen::MatrixXd result_matrix=Eigen::MatrixXd::Identity(ma.rows(),ma.cols());
//     for(int i=0;i<n;i++)
//     {
//         result_matrix*=ma;
//     }
//     return result_matrix;
// }

MPCOnmiControl::MPCOnmiControl(double sample_time,int Np,int Nc)
{
    //控制时域和预测时域
    Np_=Np;
    Nc_=Nc;
    sample_time_=sample_time;

    //状态变量个数和控制变量个数
    basic_state_size_=2;
    basic_control_size_=2; 

    //松弛因子
    relax_factor_=5.0;
    relax_factor_max_=10.0;

    //约束参数
    //转角单个方向的范围 0.0174
    wheel_single_vw_max_degree_ = 40.0/180.0*M_PI;
    //最大转角转速
    wheel_single_vw_max_vel_ = 40.0/180.0*M_PI;
    //最大速度
    vy_max_ = 10.0;
    //最大纵向加速度
    ay_max_ = 1.0;
    //当前位置误差
    vector_error_=Eigen::MatrixXd::Zero(basic_state_size_+basic_control_size_,1);

    // parameters for mpc solver; number of iterations
    mpc_max_iteration_ = 500;
    // parameters for mpc solver; threshold for computation
    mpc_eps_ = 0.05;

}

MPCOnmiControl:: ~ MPCOnmiControl()
{
    ;
}

void MPCOnmiControl::updateMatrix()
{    
    //找到离实际位置最近的参考点
    double vx_ref=0.0,vy_ref=0.0,phi_ref=0.0,vw_ref=0.0;
    double phi_now=current_state_.phi;
    updateNearestRefState(nearest_ref_traj_pos_,nearest_ref_traj_index_);
    vx_ref=nearest_ref_traj_pos_.vx;
    vy_ref=nearest_ref_traj_pos_.vy;
    phi_ref=nearest_ref_traj_pos_.phi;
    vw_ref=nearest_ref_traj_pos_.vw;
    
    //更新线性化的vehicle状态空间方程，与离散方程
    matrix_a_=Eigen::MatrixXd::Zero(basic_state_size_,basic_state_size_);
    matrix_a_(0,1)=-vx_ref*sin(phi_ref)+vy_ref*cos(phi_ref);
    //matrix_a_(1,2)=vx_ref*cos(phi_ref);
    /*
    0 -vx*sin phi+vy*cos phi
    0 0
    */

    matrix_ad_=Eigen::MatrixXd::Zero(basic_state_size_,basic_state_size_);
    //matrix_ad_(0,1)=(-vx_ref*sin(phi_ref)+vy_ref*cos(phi_ref))*sample_time_;
    //matrix_ad_(0,1)=-vy_ref*sin(phi_now-phi_ref)*sample_time_;
    matrix_ad_(0,1)=0;
    matrix_ad_(0,0)=1.0;
    matrix_ad_(1,1)=1.0;
    /*
    1 -vx*sin phi+vy*cos phi
    0 1
    */

    matrix_b_=Eigen::MatrixXd::Zero(basic_state_size_,basic_control_size_);
    matrix_b_(0,0)=-1.0;
    matrix_b_(1,1)=1.0;

    matrix_bd_=matrix_b_*sample_time_;
    /*
    T*cosPhi 0 
            0         1
    */

    matrix_c_=Eigen::MatrixXd::Identity(basic_state_size_,basic_state_size_);

    //更新状态空间方程（考虑将控制量也作为状态变量）
    matrix_A_.resize(basic_control_size_+basic_state_size_,basic_control_size_+basic_state_size_);
    matrix_A_<<matrix_ad_,matrix_bd_,Eigen::MatrixXd::Zero(basic_control_size_,basic_state_size_),Eigen::MatrixXd::Identity(basic_control_size_,basic_control_size_);
    /*
    a b 
    0  I
    */

    matrix_B_.resize(basic_control_size_+basic_state_size_,basic_control_size_);
    matrix_B_<<matrix_bd_,Eigen::MatrixXd::Identity(basic_control_size_,basic_control_size_);
    /*
    b 
    I
    */
    matrix_C_.resize(basic_state_size_,basic_control_size_+basic_state_size_);
    matrix_C_<<Eigen::MatrixXd::Identity(basic_state_size_,basic_state_size_),Eigen::MatrixXd::Zero(basic_state_size_,basic_control_size_);
    /*
    I  0
    */

    //cout<<matrix_A_<<endl<<endl;
    //cout<<matrix_B_<<endl<<endl;
    //cout<<matrix_C_<<endl<<endl;

    //更新预测矩阵
    matrix_Ap_=Eigen::MatrixXd::Zero(Np_*matrix_C_.rows(),matrix_A_.cols());
    matrix_Bp_=Eigen::MatrixXd::Zero(Np_*matrix_C_.rows(),Nc_*matrix_B_.cols());

    for(int i=0;i<Np_;i++)
    {
        for(int j=0;j<Nc_;j++)
        {
            if(j==0)
                matrix_Ap_.block(i*matrix_C_.rows(),j*matrix_A_.cols(),matrix_C_.rows(),matrix_A_.cols())=matrix_C_*powMatrix(matrix_A_,i+1);

            if(i>=j)
                matrix_Bp_.block(i*matrix_C_.rows(),j*matrix_B_.cols(),matrix_C_.rows(),matrix_B_.cols())=matrix_C_*powMatrix(matrix_A_,i-j)*matrix_B_;

        }
    }
    /*
    CA
    CA^2 
    ...
    CA^np
    */
    /*
    CB                   0                        ...      0
    CAB                 CB                              0
    ...
    CA^(np-1)B  CA^(np-2)B ...       CA^(np-nc)B
    */
    //更新权重矩阵
    matrix_q_=100*Eigen::MatrixXd::Identity(basic_state_size_*Np_,basic_state_size_*Np_);
    matrix_r_=1*Eigen::MatrixXd::Identity(basic_control_size_*Nc_,basic_control_size_*Nc_);
    //cout<<matrix_q_<<endl<<matrix_r_;

    computeErrors(nearest_ref_traj_pos_);

    //更新二次规划矩阵
    matrix_H_=Eigen::MatrixXd::Zero(basic_control_size_*Nc_+1,basic_control_size_*Nc_+1);
    matrix_H_.block(0,0,matrix_r_.rows(),matrix_r_.cols())=(matrix_Bp_.transpose())*matrix_q_*matrix_Bp_+matrix_r_;// 表示返回从矩阵(i, j)开始，每行取p个元素，每列取q个元素所组成的临时新矩阵对象，原矩阵的元素不变
    matrix_H_(matrix_H_.rows()-1,matrix_H_.cols()-1)=relax_factor_;
    matrix_G_=Eigen::MatrixXd::Zero(1,basic_control_size_*Nc_+1);
    matrix_G_.leftCols(basic_control_size_*Nc_)=2*(matrix_Ap_*vector_error_).transpose()*matrix_q_*matrix_Bp_;
    //cout<<matrix_H_<<endl;
    //cout<<matrix_G_<<endl;


    //----------更新约束矩阵----------------------
    //更新控制增量及松弛因子的上下界
    matrix_du_ub_=Eigen::MatrixXd::Zero(basic_control_size_*Nc_+1,1);
    matrix_du_lb_=Eigen::MatrixXd::Zero(basic_control_size_*Nc_+1,1);
    for(int i=0;i<Nc_;i++)
    {
        matrix_du_ub_(basic_control_size_*i,0)=ay_max_*sample_time_;
        matrix_du_ub_(basic_control_size_*i+1,0)=wheel_single_vw_max_vel_*sample_time_;
    }
    matrix_du_lb_=-1.0*matrix_du_ub_;
    matrix_du_ub_(basic_control_size_*Nc_,0)=relax_factor_max_;
    //cout<<matrix_du_lb_;

    //更新控制量的上下界
    matrix_u_ub_.resize(basic_control_size_*Nc_,1);
    matrix_u_lb_.resize(basic_control_size_*Nc_,1);
    for(int i=0;i<Nc_;i++)
    {
        matrix_u_ub_(basic_control_size_*i,0)=vy_max_-vy_ref;
        matrix_u_ub_(basic_control_size_*i+1,0)=wheel_single_vw_max_degree_-vw_ref;
    }
     for(int i=0;i<Nc_;i++)
    {
        matrix_u_lb_(basic_control_size_*i,0)= -vy_max_-vy_ref;
        matrix_u_lb_(basic_control_size_*i+1,0)=-wheel_single_vw_max_degree_-vw_ref;
    }
    
    //更新增量的线性不等式约束
    matrix_constraint_A_=Eigen::MatrixXd::Zero(basic_control_size_*Nc_,basic_control_size_*Nc_+1);
    for(int i=0;i<Nc_;i++)
    {
        for(int j=0;j<=i;j++)
        {
            matrix_constraint_A_.block(i*basic_control_size_,j*basic_control_size_,basic_control_size_,basic_control_size_)=Eigen::MatrixXd::Identity(basic_control_size_,basic_control_size_);
        }
    }

    //更新上个时刻的控制量，以便约束矩阵A的上下界的构造
    Eigen::MatrixXd matrix_previous_control_offset;
    matrix_previous_control_offset.resize(basic_control_size_*Nc_,1);
    for(int i=0;i<Nc_;i++)
    {
        matrix_previous_control_offset(basic_control_size_*i,0)= real_control_vy_offset_.back();
        matrix_previous_control_offset(basic_control_size_*i+1,0)=real_control_vw_offset_.back();
    }

    //更新约束矩阵A对应的上下界
    matrix_constraint_ubA_.resize(basic_control_size_*Nc_,1);
    matrix_constraint_lbA_.resize(basic_control_size_*Nc_,1);

    matrix_constraint_ubA_=matrix_u_ub_-matrix_previous_control_offset;
    matrix_constraint_lbA_=matrix_u_lb_-matrix_previous_control_offset;

    // Eigen::MatrixXd matrix_previous_control;
    // matrix_previous_control.resize(basic_control_size_*Nc_,1);
    // for(int i=0;i<Nc_;i++)
    // {
    //     matrix_previous_control(basic_control_size_*i,0)=real_control_vel_.back();
    //     matrix_previous_control(basic_control_size_*i+1,0)=real_control_delta_.back();
    // }

    // //更新约束矩阵A对应的上下界
    // matrix_constraint_ubA_.resize(basic_control_size_*Nc_,1);
    // matrix_constraint_lbA_.resize(basic_control_size_*Nc_,1);

    // matrix_constraint_ubA_=matrix_u_ub_-matrix_previous_control;
    // matrix_constraint_lbA_=matrix_u_lb_-matrix_previous_control;

}

//计算当前位置和对应轨迹参考点的差  note:控制量误差也必须计算
void MPCOnmiControl::computeErrors(trajOnmi & traj_ref)
{
    vector_error_=Eigen::MatrixXd::Zero(basic_state_size_+basic_control_size_,1);
    //vector_error_(0,0)=(current_state_.y-traj_ref.y)*cos(traj_ref.phi) - (current_state_.x-traj_ref.x)*sin(traj_ref.phi);
    vector_error_(0,0)=(current_state_.y-traj_ref.y)*cos(current_state_.phi) - (current_state_.x-traj_ref.x)*sin(current_state_.phi);
    dy.push_back((current_state_.y-traj_ref.y)*cos(current_state_.phi) - (current_state_.x-traj_ref.x)*sin(current_state_.phi));
    dy2.push_back((current_state_.y-traj_ref.y)*cos(traj_ref.phi) - (current_state_.x-traj_ref.x)*sin(traj_ref.phi));
    dphi.push_back(current_state_.phi-traj_ref.phi);
    pf.push_back(traj_ref.phi);
    preal.push_back(current_state_.phi);
    //vector_error_(1,0)=current_state_.y-traj_ref.y;
    vector_error_(1,0)=current_state_.phi-traj_ref.phi;
    vector_error_(2,0)=current_state_.vy-traj_ref.vy;
    vector_error_(3,0)=current_state_.vw-traj_ref.vw;
}
//设置机器人当前位置信息
void MPCOnmiControl::updateState(trajOnmi & current_pos)
{
    current_state_=current_pos;
}
bool MPCOnmiControl::qpSlover()
{
    USING_NAMESPACE_QPOASES
    //需要先将矩阵转换为一维数组，然后才能利用qpOASES进行求解

    //转换H矩阵
    double h_matrix[matrix_H_.rows()*matrix_H_.cols()];
    for(int i=0;i<matrix_H_.rows();i++)
    {
        for(int j=0;j<matrix_H_.cols();j++)
        {
            h_matrix[i*matrix_H_.cols()+j]=matrix_H_(i,j);
            //cout<<h_matrix[i*matrix_H_.cols()+j]<<" ";
        }
    }

    //转换g矩阵
    double g_matrix[matrix_G_.rows()*matrix_G_.cols()];
    for(int i=0;i<matrix_G_.rows();i++)
    {
        for(int j=0;j<matrix_G_.cols();j++)
        {
            g_matrix[i*matrix_G_.cols()+j]=matrix_G_(i,j);
            //cout<<g_matrix[i*matrix_G_.cols()+j]<<" ";
        }
    }

    //转换du_lb矩阵，控制增量下边界
    double  lower_bound[matrix_du_lb_.rows()*matrix_du_lb_.cols()];
    for(int i=0;i<matrix_du_lb_.rows();i++)
    {
        for(int j=0;j<matrix_du_lb_.cols();j++)
        {
            lower_bound[i*matrix_du_lb_.cols()+j]=matrix_du_lb_(i,j);
            //cout<<lower_bound[i*matrix_du_lb_.cols()+j]<<" ";
        }
    }

    //转换du_ub矩阵，控制增量上边界
    double  upper_bound[matrix_du_ub_.rows()*matrix_du_ub_.cols()];
    for(int i=0;i<matrix_du_ub_.rows();i++)
    {
        for(int j=0;j<matrix_du_ub_.cols();j++)
        {
            upper_bound[i*matrix_du_ub_.cols()+j]=matrix_du_ub_(i,j);
            //cout<<upper_bound[i*matrix_du_ub_.cols()+j]<<" ";
        }
    }

    double affine_constraint_matrix[matrix_constraint_A_.rows()*matrix_constraint_A_.cols()];
    for(int i=0;i<matrix_constraint_A_.rows();i++)
    {
        for(int j=0;j<matrix_constraint_A_.cols();j++)
        {
            affine_constraint_matrix[i*matrix_constraint_A_.cols()+j]=matrix_constraint_A_(i,j);
            //cout<<affine_constraint_matrix[i*matrix_constraint_A_.cols()+j]<<" ";
        }
    }

    //转换lbA矩阵，控制量下边界
    double  constraint_lower_bound[matrix_constraint_lbA_.rows()*matrix_constraint_lbA_.cols()];
    for(int i=0;i<matrix_constraint_lbA_.rows();i++)
    {
        for(int j=0;j<matrix_constraint_lbA_.cols();j++)
        {
            constraint_lower_bound[i*matrix_constraint_lbA_.cols()+j]=matrix_constraint_lbA_(i,j);
            //cout<<constraint_lower_bound[i*matrix_constraint_lbA_.cols()+j]<<" ";
        }
    }


    //转换ubA矩阵，控制量上边界
    double  constraint_upper_bound[matrix_constraint_ubA_.rows()*matrix_constraint_ubA_.cols()];
    for(int i=0;i<matrix_constraint_ubA_.rows();i++)
    {
        for(int j=0;j<matrix_constraint_ubA_.cols();j++)
        {
            constraint_upper_bound[i*matrix_constraint_ubA_.cols()+j]=matrix_constraint_ubA_(i,j);
            //cout<<constraint_upper_bound[i*matrix_constraint_ubA_.cols()+j]<<" ";
        } 
    }

    //qp求解过程
    QProblem qp_problem(basic_control_size_*Nc_+1,basic_control_size_*Nc_);
    Options my_options;
    my_options.printLevel=PL_NONE;
    qp_problem.setOptions(my_options);
    int nWSR=mpc_max_iteration_;//备注，一定要单独给定，不能直接把mpc_max_iteration_给init函数，否则qp不可解
    auto return_result=qp_problem.init(h_matrix,g_matrix,affine_constraint_matrix,lower_bound,upper_bound,constraint_lower_bound,constraint_upper_bound,nWSR,0);
    //auto return_result=qp_problem.init(h_matrix,g_matrix,upper_bound,constraint_lower_bound,constraint_upper_bound,nWSR);
    if (return_result != qpOASES::SUCCESSFUL_RETURN) {
        if (return_result == qpOASES::RET_MAX_NWSR_REACHED) {
            cout << "error :qpOASES solver failed due to reached max iteration"<<endl;
        } else {
        cout << "error :qpOASES solver failed due to infeasibility or other internal "
                "reasons" <<endl;
        } 
    }

    double result[basic_control_size_*Nc_+1];
    qp_problem.getPrimalSolution(result);
    
    //qp_problem.printOptions();
    optim_control_.resize(basic_control_size_*Nc_);
    //cout<<"size "<<basic_control_size_*Nc_<<endl;
    for(int i=0;i<basic_control_size_*Nc_;i++)
    {
        optim_control_[i]=result[i];
        //cout<<optim_control_[i]<<endl;
    }

    optim_relax_factor_=result[basic_control_size_*Nc_];

    //保存实际控制量，得到的最优控制增量+上一时刻的实际控制量
    ddduuu.push_back(optim_control_[1]);
    real_control_vy_offset_.push_back(optim_control_[0]+real_control_vy_offset_.back());
    real_control_vw_offset_.push_back(optim_control_[1]+real_control_vw_offset_.back());
    real_control_vy_.push_back(real_control_vy_offset_.back()+ref_traj_[nearest_ref_traj_index_].vy);
    real_control_vw_.push_back(real_control_vw_offset_.back()+ref_traj_[nearest_ref_traj_index_].vw);
    // real_control_vel_.push_back(optim_control_[0]+real_control_vel_.back());
    // real_control_delta_.push_back(optim_control_[1]+real_control_delta_.back());
    return qp_problem.isSolved();
}

bool MPCOnmiControl::getFirstControl(double& vy,double& vw)
{
    if(optim_control_.size()<2)
    {
        cout<<"Error : not enough optim_control "<<endl;
        return false;
    }


    vy=real_control_vy_.back();
    vw=real_control_vw_.back();

    return true;
}

bool MPCOnmiControl::getRealControl(vector<double>& real_vy,vector<double>& real_vw)
{
    if(real_control_vy_.size()==0 | real_control_vw_.size()==0)
    {
        cout<<"MPCControl::getRealControl(vector<double>& real_vel,vector<double>& real_delta) :real_control size is zero "<<endl;
        return false;
    }

    real_vy.clear();
    real_vw.clear();
    real_vy.resize(real_control_vy_.size());
    real_vw.resize(real_control_vw_.size());
    copy(real_control_vy_.begin(),real_control_vy_.end(),real_vy.begin());
    copy(real_control_vw_.begin(),real_control_vw_.end(),real_vw.begin());

    for(int i=0;i<real_control_vy_.size();i++)
    {
      ; // cout<<real_control_vel_[i]<<" "<<real_vel[i]<<endl;

    }

    return true;
}

void MPCOnmiControl::setRefTraj(vector<trajOnmi> & ref_traj)
{
    ref_traj_.resize(ref_traj.size());
    for(int i=0;i<ref_traj.size();i++)
    {
        ref_traj_[i]=ref_traj[i];
    }
    real_control_vy_offset_.push_back(0.0-ref_traj_[0].vy);
    real_control_vw_offset_.push_back(0.0-ref_traj_[0].vw);
    real_control_vy_.push_back(0.0);
    real_control_vw_.push_back(0.0);
}

double MPCOnmiControl::calculateDistance(trajOnmi pos1,trajOnmi pos2)
{
    return  sqrt((pos1.x-pos2.x)*(pos1.x-pos2.x)+(pos1.y-pos2.y)*(pos1.y-pos2.y));
}


void MPCOnmiControl::updateNearestRefState(trajOnmi& traj_ref_pos,int & traj_ref_index)
{
    double dis_min=calculateDistance(current_state_,ref_traj_[0]);

    //遍历找最小
    for(int i=0;i<ref_traj_.size();i++)
    {
        double dis_cached=this->calculateDistance(current_state_,ref_traj_[i]);
        if(dis_cached<=dis_min)
        {
            dis_min=dis_cached;
            traj_ref_index=i;
            traj_ref_pos=ref_traj_[i];
        }
    }
}



bool  MPCOnmiControl::isGoalReached()
{
    if(calculateDistance(current_state_,ref_traj_.back())<goal_threshold_)
        return true;
    else
        return false;

}


//参考资料
//https://blog.csdn.net/u013914471/article/details/83748571