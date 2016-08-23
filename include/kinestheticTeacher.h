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
    static auto constexpr  FILTER_FREQ = 50.0;

    static auto constexpr  SMOOTHING = 0.9;
    static auto constexpr  DC_FILTER1 = 0.99999;
    static auto constexpr  DC_FILTER2 = 0.9998;

    static auto constexpr  FORGET_SHRINK = 0.99;
    static auto constexpr  FORGET_EXPAND = 0.1;
    static auto constexpr  ACCEPTANCE_INTERVAL = 0.90;

    static auto constexpr FORCE_MIN_LIMIT = 0.3;   // Limits that auto scaling cannot go below
    static auto constexpr TORQUE_MIN_LIMIT = 0.02; // Limits that auto scaling cannot go below

    static auto constexpr FORCES_MOVING_MULTIPLIER = 0.1;
    static auto constexpr TORQUES_MOVING_MULTIPLIER = 0.26;
    static auto constexpr Z_FORCE_MOVING_MULTIPLIER = 0.1; //temp

    static auto constexpr BASE_XY_MOVING_MULTIPLIER = 0.0;//0.6
    static auto constexpr BASE_Z_MOVING_MULTIPLIER = 0.1;//0.2
    static auto constexpr ARM_ALLJOINTS_MOVING_MULTIPLIER = 0.15;

    static auto constexpr MAXIMUM_JOINT_STEP = 0.1;

    bool teacherRunning;
    bool filtersRunning;
    bool firstReading;

    ros::Subscriber sensorUpdateSub;

    std::mutex sensorMutex;
    std::mutex kukaduMutex;
    std::mutex transformedValueMutex;
    std::mutex filterSmoothingMutex;
    std::mutex filterDC1Mutex;
    std::mutex filterDC2Mutex;

    std::vector<double> sensorVal;
    std::vector<double> projectedFilteredReadings;
    std::vector<double> smoothingFilterMemory;
    std::vector<double> DCFilter1Memory;
    std::vector<double> DCFilter2Memory;
    std::vector<std::pair<double,double>> limitsFilterMemory;

    std::shared_ptr<ros::NodeHandle> myNode;
    KUKADU_SHARED_PTR<kukadu::KukieControlQueue> robotinoQueue;
    std::shared_ptr<kukadu::MoveItKinematics> mvKin;
    std::shared_ptr<kukadu_thread> qThread;
    std::shared_ptr<std::thread> moveThread;
    std::shared_ptr<std::thread> filterSmoothingThread;
    std::shared_ptr<std::thread> filterDC1Thread;
    std::shared_ptr<std::thread> filterDC2Thread;

    enum ControllerType {JACOBIAN,IK};

    int sign(double x);
    void sensorUpdate(std_msgs::Float64MultiArray msg);
    std::vector<double> capVec(std::vector<double> input, double maxCap);
    void lowpassFilter(std::vector<double> &filter,std::vector<double> newVal, double forgettingFactor);
    void limitsFilter(std::vector<std::pair<double,double>> &filter,std::vector<std::vector<double>> newDataSet, double forgettingFactorShrink, double forgettingFactorExpand, double percentageWithinLimits);
    std::vector<double> removeBias(std::vector<double> values,std::vector<double> bias);
    std::vector<double> scaleAndLimitReadings(std::vector<double> msg);
    std::vector<double> projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma);
    std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);
    std::vector<double> getProcessedReading();
    bool isColliding(std::vector<double> jointStates);
    bool isDifferentialCommandSafe(arma::vec diffCommand,arma::vec currentJointStates);
    arma::vec getNextDifferentialCommand(Eigen::MatrixXd jacobian, ControllerType myType);
    void generateNextCommand();

    void filterSmoothingThreadHandler();
    void filterDC1ThreadHandler();
    void filterDC2ThreadHandler();
    void teachingThreadHandler();

public:

    KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic);
    void init();
    void runArm();
    void stopArm();
    void printFilteredSensorVal();
    void ptp(std::vector<double> target); //remove

};

#endif
