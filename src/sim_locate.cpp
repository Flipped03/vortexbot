#include"vortexbot/sim_locate.h"

SimLocate::SimLocate(double car_length,traj init_pos)
{
    car_length_=car_length;
    real_traj_.clear();
    real_traj_.push_back(init_pos);
}
void SimLocate::updateEulerPosition(double vel,double delta,double sample_time)
{
    traj  next_pos;
    next_pos.x=real_traj_.back().x+vel*cos(real_traj_.back().phi)*sample_time;
    next_pos.y=real_traj_.back().y+vel*sin(real_traj_.back().phi)*sample_time;
    next_pos.phi=real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time;
    next_pos.vel=vel;
    next_pos.delta=delta;

    real_traj_.push_back(next_pos);
}

void SimLocate::updateRungeKuttaPosition(double vel,double delta,double sample_time)
{
    traj  next_pos;
    next_pos.x=real_traj_.back().x+vel*cos(real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time/2.0)*sample_time;
    next_pos.y=real_traj_.back().y+vel*sin(real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time/2.0)*sample_time;
    next_pos.phi=real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time;
    next_pos.vel=vel;
    next_pos.delta=delta;

    real_traj_.push_back(next_pos);
    
    cout<<"next_pos.x: "<<next_pos.x<<"next_pos.y: "<<next_pos.y<<endl;
}


bool SimLocate::getCurrentPosition(traj & current_state)
{
    if(real_traj_.size()==0)
    return false;
    current_state=real_traj_.back();
    return true;

}

void SimLocate::getRealTraj(vector<traj>& real_traj)
{
    real_traj.resize(real_traj_.size());
    for(int i=0;i<real_traj_.size();i++)
        real_traj[i]=real_traj_[i];
}

/****************SimLocateOnmi****************************/
SimLocateOnmi::SimLocateOnmi(trajOnmi init_pos)
{
    real_traj_.clear();
    real_traj_.push_back(init_pos);
}
void SimLocateOnmi::updateEulerPosition(double vx,double vy,double vw,double sample_time)
{
    // traj  next_pos;
    // next_pos.x=real_traj_.back().x+vel*cos(real_traj_.back().phi)*sample_time;
    // next_pos.y=real_traj_.back().y+vel*sin(real_traj_.back().phi)*sample_time;
    // next_pos.phi=real_traj_.back().phi+vel*tan(delta)/car_length_*sample_time;
    // next_pos.vel=vel;
    // next_pos.delta=delta;

    // real_traj_.push_back(next_pos);

    trajOnmi  next_pos;
    next_pos.x=real_traj_.back().x  +  vx*cos(real_traj_.back().phi)*sample_time + vy*sin(real_traj_.back().phi)*sample_time;
    next_pos.y=real_traj_.back().y  +  vx*sin(real_traj_.back().phi)*sample_time - vy*cos(real_traj_.back().phi)*sample_time;
    next_pos.phi=real_traj_.back().phi + vw*sample_time;
    next_pos.vx=vx;
    next_pos.vy=vy;
    next_pos.vw=vw;

    real_traj_.push_back(next_pos);
}

void SimLocateOnmi::updateRungeKuttaPosition(double vx,double vy,double vw,double sample_time)
{
    trajOnmi  next_pos;
    next_pos.x=real_traj_.back().x + vx*cos(real_traj_.back().phi + vw*sample_time)*sample_time + vy*sin(real_traj_.back().phi + vw*sample_time)*sample_time;
    next_pos.y=real_traj_.back().y + vx*sin(real_traj_.back().phi + vw*sample_time)*sample_time - vy*cos(real_traj_.back().phi + vw*sample_time)*sample_time;
    next_pos.phi=real_traj_.back().phi + vw*sample_time;
    if(next_pos.phi>=2*M_PI)  next_pos.phi=next_pos.phi-2*M_PI;
    if(next_pos.phi<0)  next_pos.phi=next_pos.phi+2*M_PI;
    next_pos.vx=vx;
    next_pos.vy=vy;
    next_pos.vw=vw;

    real_traj_.push_back(next_pos);
    
    cout<<"next_pos.x: "<<next_pos.x<<"next_pos.y: "<<next_pos.y<<endl;
}


bool SimLocateOnmi::getCurrentPosition(trajOnmi & current_state)
{
    if(real_traj_.size()==0)
    return false;
    current_state=real_traj_.back();
    return true;

}

void SimLocateOnmi::getRealTraj(vector<trajOnmi>& real_traj)
{
    real_traj.resize(real_traj_.size());
    for(int i=0;i<real_traj_.size();i++)
        real_traj[i]=real_traj_[i];
}
