#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
 
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"
#include "urdf/model.h"
#include <math.h>
#include <serial/serial.h>
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#define pi 3.141592653

int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"kinetic_ARM_UN");
    ros::NodeHandle nh;
    ros::Publisher joint_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);
    tf2_ros::StaticTransformBroadcaster broadcaster;
 
    geometry_msgs::TransformStamped ts;
    KDL::Vector v1(1,1,1);
 
    KDL::Tree my_tree;
    sensor_msgs::JointState joint_state;
 
    std::string robot_desc_string;
    nh.param("robot_description", robot_desc_string, std::string());
 
    if(!kdl_parser::treeFromString(robot_desc_string, my_tree))
    //if(!kdl_parser::treeFromFile("/home/zhitong/catkin_ws3/src/LIUZU/urdf/LIUZU.urdf", my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
    }
    else
    {
        ROS_INFO("成功生成kdl树!");
    }
 
    std::vector<std::string> joint_name = {
  "joint1", "joint2", "joint3", "joint4","joint5", "joint6"
};
 
    std::vector<double> joint_pos = {
0,0,0,0,0,0
};
    std::string urdf_param = "/robot_description";
    double timeout = 0.005;
    double eps = 1e-5;
    std::string chain_start  = "base_link"; 
    std::string chain_end = "link6"; 

    //逆运动学求解器
    TRAC_IK::TRAC_IK tracik_arm_solver(chain_start, chain_end, urdf_param, timeout, eps);

    KDL::Chain chain_arm;

    KDL::JntArray ll, ul; //关节下限, 关节上限
    bool valid = tracik_arm_solver.getKDLChain(chain_arm);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL chain found on arm link chain");
    }
    valid = tracik_arm_solver.getKDLLimits(ll, ul);
    if(!valid)
    {
        ROS_ERROR("There was no valid KDL joint limits found on rf link chain");
    }
    
    //正运动学求解器
    KDL::ChainFkSolverPos_recursive fk_solver_arm(chain_arm);

    ROS_INFO("rf关节数量: %d", chain_arm.getNrOfJoints());

    KDL::JntArray nominal(6);
 
    ROS_INFO("the nominal size is:%d",nominal.data.size());
 
    for(size_t j = 0; j < 6; j ++)
    {
        nominal(j)=0.0;
        //nominal(j) = (ll(j) + ul(j))/2.0;
    }
    
     //定义初始状态末端点齐次矩阵
    KDL::Frame end_effector_pose_start;//start
    KDL::Frame end_effector_pose_now;//now

    //定义逆运动学解算结果存储数组
    KDL::JntArray result_last(6);//last time
    KDL::JntArray result_now(6);//now

    result_last(0)=0;
    result_last(1)=pi/6;
    result_last(2)=-pi/6;
    result_last(3)=0;
    result_last(4)=0;
    result_last(5)=0;

    ros::Rate r(5);
 
    auto print_frame_lambda = [](KDL::Frame f)
    {
        double x, y, z, roll, pitch, yaw;
        x = f.p.x();
        y = f.p.y();
        z = f.p.z();
        f.M.GetRPY(roll, pitch, yaw);
        std::cout << "x:" << x << " y:" << y << " z:" << z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
    };
    
    //正运动学
    fk_solver_arm.JntToCart(result_last,end_effector_pose_start);//RF

    //逆运动学
    int rc = tracik_arm_solver.CartToJnt(result_last, end_effector_pose_start, result_now);//RF

    ROS_INFO("result   1:%f,2:%f,3:%f,4:%f,5:%f,6:%f",result_now(0),result_now(1),result_now(2),result_now(3),result_now(4),result_now(5));

    print_frame_lambda(end_effector_pose_start);
 
    ROS_INFO("更新关节状态");
    joint_state.header.stamp = ros::Time::now();
    
    joint_state.name.resize(6);
    joint_state.position.resize(6);
 
    for(size_t i = 0; i < 6; i ++)
    {
        joint_state.name[i] = joint_name[i];
        joint_state.position[i] = 0;
         joint_state.position[i] = result_now(i);
    }
    
    //正运动学
    fk_solver_arm.JntToCart(result_last,end_effector_pose_now);//RF

    //数据存储
    FILE *fp;

    if( (fp=fopen("/home/c123/catkin_zhitong/src/file.txt","ab"))==NULL )
    {
        printf("cannot open file");
        return 0;
    }

    while(ros::ok())
    {
    //x=0.02*(t-sint);
    //y=0.02*(1-cost);
    for(int i=1;i<=20;i++)
    { 
        double t=2*pi*i/20;
        double x=0.010*(t-sin(t));
        double z=0.010*(1-cos(t));
        //右边
        end_effector_pose_now.p.data[0]=end_effector_pose_start.p.data[0]-0.01*pi+x;
        end_effector_pose_now.p.data[2]=end_effector_pose_start.p.data[2]+z;
        int rc_rf = tracik_arm_solver.CartToJnt(result_last, end_effector_pose_now, result_now);
        print_frame_lambda(end_effector_pose_now);

        joint_state.header.stamp = ros::Time::now();

        joint_state.position[0] = result_now(0);
        joint_state.position[1] = result_now(1);
        joint_state.position[2] = result_now(2);
	
        //将数据写入hexapod的arduino话题
            for(int j=0;j<3;j++)
            {
                //写入文件
                fprintf(fp,"%.2f",joint_state.position[j]*180.0/pi);
	            fputs(",",fp);
            }
	fputs("\n",fp);
        
        joint_pub.publish(joint_state);

        result_last=result_now;
        r.sleep();
    }
    
 
    for(int i=1;i<=20;i++)
    { 
        double t=2*pi*i/20;
        double x=0.010*(t-sin(t));
        double z=0.010*(1-cos(t));

        end_effector_pose_now.p.data[0]=end_effector_pose_start.p.data[0]+0.01*pi-x;
        end_effector_pose_now.p.data[2]=end_effector_pose_start.p.data[2];
        int rc = tracik_arm_solver.CartToJnt(result_last, end_effector_pose_now, result_now);
        print_frame_lambda(end_effector_pose_now);
 
        joint_state.header.stamp = ros::Time::now();

        joint_state.position[0] = result_now(0);
        joint_state.position[1] = result_now(1);
        joint_state.position[2] = result_now(2);
	
        //将数据写入hexapod的arduino话题
            for(int j=0;j<3;j++)
            {
                //写入文件
                fprintf(fp,"%.2f",joint_state.position[j]*180.0/pi);
	            fputs(",",fp);
            }
	fputs("\n",fp);
        
        joint_pub.publish(joint_state);

        result_last=result_now;
        r.sleep();
    }
	// fclose(fp);
	// for(int i=0;i<=1000;i++)
	// 	r.sleep();
    }
    return 0;
 
}

