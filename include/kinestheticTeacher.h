#ifndef KinestheticTeacher_HPP
#define KinestheticTeacher_HPP


#include <mutex>
#include <std_msgs/Float64MultiArray.h>
#include <kukadu/kukadu.hpp>
#include <cmath>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <Eigen/Dense>




class KinestheticTeacher{

ros::Subscriber sensorUpdateSub;
std::mutex sensorMutex;
std::mutex kukaduMutex;
std::mutex transformedValueMutex;
std::mutex filterSmoothingMutex;
std::mutex filterDC1Mutex;
std::mutex filterDC2Mutex;
std::vector<double> sensorVal;
std::vector<double> projectedFilteredReadings;
std::shared_ptr<ros::NodeHandle> myNode;
KUKADU_SHARED_PTR<kukadu::KukieControlQueue> robotinoQueue;
std::shared_ptr<kukadu::MoveItKinematics> mvKin;
std::shared_ptr<kukadu_thread> qThread;
std::shared_ptr<std::thread> moveThread;
std::shared_ptr<std::thread> filterSmoothingThread;
std::shared_ptr<std::thread> filterDC1Thread;
std::shared_ptr<std::thread> filterDC2Thread;

bool teacherRunning;
bool filtersRunning;
void sensorUpdate(std_msgs::Float64MultiArray msg);
Eigen::MatrixXd capMatrix(Eigen::MatrixXd input, double maxCap,int i,int j);
void lowpassFilter(std::vector<double> &filter,std::vector<double> newVal, double forgettingFactor);
std::vector<double> smoothingFilterMemory;
std::vector<double> DCFilter1Memory;
std::vector<double> DCFilter2Memory;
int sign(double x);
bool firstReading;
static auto constexpr  FILTER_FREQ = 50.0;


static auto constexpr  SMOOTHING = 0.7;
static auto constexpr  DC_FILTER = 0.996;

static auto constexpr SCALEXFORCE = 1.1;
static auto constexpr SCALEYFORCE = 1.1;
static auto constexpr SCALEZFORCE = 0.1;
static auto constexpr SCALEXTORQUE = 10.0;
static auto constexpr SCALEYTORQUE = 80.0;
static auto constexpr SCALEZTORQUE = 150.0;
std::vector<double> removeBias(std::vector<double> values,std::vector<double> bias);
std::vector<double> scaleReadings(std::vector<double> msg);
std::vector<double> projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma);
std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);
void filterSmoothingThreadHandler();
void filterDC1ThreadHandler();
void filterDC2ThreadHandler();
void teachingThreadHandler();

std::vector<double> getProcessedReading();
public:
KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic);
void init();
void runArm();
void stopArm();

void generateNextPositionCommand();
void printFilteredSensorVal();
};

#endif
