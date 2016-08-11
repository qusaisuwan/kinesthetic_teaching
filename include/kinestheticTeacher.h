#ifndef KinestheticTeacher_HPP
#define KinestheticTeacher_HPP


#include <mutex>
#include <std_msgs/Float64MultiArray.h>
#include <kukadu/kukadu.hpp>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <Eigen/Dense>

using namespace std;
using namespace arma;
using namespace kukadu;


class KinestheticTeacher{

ros::Subscriber sensorUpdateSub;
std::mutex sensorMutex;
std_msgs::Float64MultiArray sensorVal;
std::shared_ptr<ros::NodeHandle> myNode;
KUKADU_SHARED_PTR<KukieControlQueue> robotinoQueue;
std::shared_ptr<MoveItKinematics> mvKin;
std::shared_ptr<kukadu_thread> qThread;
std::shared_ptr<std::thread> moveThread;
bool teacherRunning;
void sensorUpdate(std_msgs::Float64MultiArray msg);
Eigen::MatrixXd capMatrix(Eigen::MatrixXd input, double maxCap,int jointsNumber);


public:
KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic);
void init();
void runArm();
void stopArm();
void teachingThread();
void generateNextPositionCommand();
};

#endif
