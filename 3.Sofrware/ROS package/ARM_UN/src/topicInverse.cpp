#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "robot_msg/robotarm_7dof_jointstate.h"

    robot_msg::robotarm_7dof_jointstate robotarm_jointstate;

    void doMsg(const sensor_msgs::JointState::ConstPtr& joint_states){

        robotarm_jointstate.position[0]=joint_states->position[0];
        robotarm_jointstate.position[1]=joint_states->position[1];
        robotarm_jointstate.position[2]=joint_states->position[2];
        robotarm_jointstate.position[3]=joint_states->position[3];
        robotarm_jointstate.position[4]=joint_states->position[4];
        robotarm_jointstate.position[5]=joint_states->position[5];
        robotarm_jointstate.zhuazi = 0;
        
        ROS_INFO("我听见:");
    }


int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"topicInverse");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;

    //定义一个机械臂关节变量消息
    ros::Publisher robotarm_jointpub=nh.advertise<robot_msg::robotarm_7dof_jointstate>("robotarm_joint",1);

    //4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("joint_states",10,doMsg);
    //5.处理订阅的消息(回调函数)
    ros::Rate r(20);
    while(ros::ok()){
        robotarm_jointpub.publish(robotarm_jointstate);
    //     6.设置循环调用回调函数
    ros::spinOnce();//循环读取接收的数据，并调用回调函数处理
    r.sleep();

    }
    

    return 0;
}