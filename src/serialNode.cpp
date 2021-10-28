#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <serial/serial.h>
#include <sstream>

#include <qpOASES.hpp>
#include "Eigen/LU"

#include"vortexbot/mpc_control.h"
#include"vortexbot/traj_generate.h"
#include"vortexbot/sim_locate.h"

#include <fstream>
#include <iostream>

using namespace std;

ros::Publisher chatter_pub;
ros::Publisher odom_pub;
serial::Serial ser;

union floatData
{
	float d;
	uint8_t data[4];
}qq0,qq1,qq2,qq3,yaw_angle;

uint8_t getChecksum(const std::vector<uint8_t>& pack, int length) {
  int i;
  uint8_t sum = 0;
  for (i = 0; i < length; ++i) {
    sum += pack[i];
  }
  return sum;
}

void parseSerial(const std::vector<uint8_t>& buf , const ros::Time& curr_time)
{
    if(buf[0]==0x55 &buf[1]==0xaa)
	{
	   int length=static_cast<int>(buf[2]);
	   if(getChecksum(buf,3+length)==buf[3+length])
	   {
            for(int i=0;i<4;i++)
            {
                yaw_angle.data[i]=buf[i+19];
            }
	   }
	}
}

double northing, easting;
void gpsCallback(const sensor_msgs::NavSatFixConstPtr& fix)
{
    
    std::string zone;
    gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    if(odom_pub)
    {
        nav_msgs::Odometry odom;

        odom.header.stamp = fix->header.stamp;

        odom.pose.pose.position.x = easting;
        odom.pose.pose.position.y = northing;
        odom.pose.pose.position.z = fix->altitude;

        odom_pub.publish(odom);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"vortex_serial");    //vortex_serial代表node的名字
    ros::NodeHandle n;

    chatter_pub = n.advertise<std_msgs::String>("chatter",1000);   //1000代表缓冲最近的1000条消息，queue
    odom_pub = n.advertise<nav_msgs::Odometry>("/wheel_odom",100);

    ros::Subscriber gps_sub = n.subscribe("/fix", 1000, gpsCallback);

    ros::Rate loop_rate(10);
    int count = 0;

    //打开串口USB1
    try
    {
        ser.setPort("/dev/ttyUSB1");
        ser.setBaudrate(115200);
        ser.setBytesize(serial::eightbits);
        ser.setStopbits(serial::stopbits_one);
        ser.setParity(serial::parity_none);
        serial::Timeout to = serial::Timeout::simpleTimeout(20);
        ser.setTimeout(to);
        ser.open();
    } catch(const serial::IOException& e){
        std::cerr << e.what() << '\n';
        ROS_ERROR("Unable to open port");
        return -1;
    }

    //生成轨迹
    traj init_pos{0.0,7.0,0.0,0.0,0};
    TrajGenerate traj_generator(500,0.05);
    vector<traj> traj_ref;
    vector<double> time_ref;
    traj_generator.getTraj(traj_ref,time_ref);

    //初始化MPC控制器
    int Np=15,Nc=10;
    double car_length=2.6;
    double sample_time=0.05;
     MPCControl mpc_controller(sample_time,car_length,Np,Nc);

    //初始化模拟定位器
    SimLocate sim_locater(car_length,init_pos);
    //将轨迹写入mpc_controller
    mpc_controller.setRefTraj(traj_ref);



    //原始轨迹写入ｔｘｔ
    ofstream outfile;  
    outfile.open("/home/li/vortex_ws/src/vortexbot/traj_ref.txt"); 
    if(outfile.is_open())  
    {  
        cout << "111123"<< endl;
        cout <<traj_ref.size()<< endl;
        for(int i=0;i<traj_ref.size();i++)
        {
            outfile<<traj_ref[i].x<<","<<traj_ref[i].y<<endl;;
        }
        outfile.close();   
    }
    
    int max_iteration = 2*1000;
     while(!mpc_controller.isGoalReached() && (--max_iteration>0))
     {
            //用于存放最优控制量
            double control_vel;
            double control_delta;
            //用于存放当前的位置
            traj current_pos{northing, easting,0,1.0,0};
            sim_locater.getCurrentPosition(current_pos);
            //告诉mpc当前机器人位置
            mpc_controller.updateState(current_pos);
            //更新MPC矩阵
            mpc_controller.updateMatrix();
            //求解ＭＰＣ
            mpc_controller.qpSlover();
            //获取ＭＰＣ的控制量
            mpc_controller.getFirstControl(control_vel,control_delta);
            // //根据控制量 模拟机器人运动
            sim_locater.updateRungeKuttaPosition(control_vel,control_delta,sample_time);
     }

    ofstream outfile1;  
    outfile1.open("/home/li/vortex_ws/src/vortexbot/real_ref.txt"); 
    if(outfile1.is_open())  
    {  
        cout << "967845"<< endl;
        cout <<sim_locater.real_traj_.size()<< endl;
        for(int i=0;i<sim_locater.real_traj_.size();i++)
        {
            outfile1<<sim_locater.real_traj_[i].x<<","<<sim_locater.real_traj_[i].y<<endl;;
        }
        outfile1.close();   
    }
    ofstream control_vel;  
    control_vel.open("/home/li/vortex_ws/src/vortexbot/control_vel.txt"); 
    if(control_vel.is_open())  
    {  
        cout << "967845"<< endl;
        cout <<mpc_controller.real_control_vel_.size()<< endl;
        for(int i=0;i<mpc_controller.real_control_vel_.size();i++)
        {
            control_vel<<mpc_controller.real_control_vel_[i]<<","<<mpc_controller.real_control_delta_[i]<<endl;
        }
        control_vel.close();   
    }
    while(ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss<<"hello world"<<count;
        msg.data = ss.str();
        ROS_INFO("%s",msg.data.c_str());

        if(ser.available()){
            ros::Time curr_time = ros::Time::now();
            std::vector<uint8_t> res;
            ser.read(res,ser.available());
	        cout << "222"<< endl;
            if(!res.empty())
                parseSerial(res,curr_time);
		cout << yaw_angle.d << endl;
        }
	cout << "123"<< endl;
    



        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
