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
#include "Filter.h"


namespace UIBK_Teaching
{

class KinestheticTeacher{

    static auto constexpr  FILTER_FREQ = 50.0;



    static auto constexpr FORCES_MOVING_MULTIPLIER = 0.14;
    static auto constexpr TORQUES_MOVING_MULTIPLIER = 0.21;

    static auto constexpr BASE_XY_MOVING_MULTIPLIER = 0.17;//0.6
    static auto constexpr BASE_Z_MOVING_MULTIPLIER = 0.6;//0.2
    static auto constexpr ARM_ALLJOINTS_MOVING_MULTIPLIER = 0.22;

    static auto constexpr MAXIMUM_JOINT_STEP = 0.1; //Movement cap
    static auto constexpr MAX_JOINT_MOVEMENT_ALLOWED_FOR_IK = 0.25; // IK solution acceptence threshold
    bool teacherRunning;
    bool filterRunning;
    bool firstReading;

    std::string recordingPath;
    Filter myFilter;
    ros::Subscriber sensorUpdateSub;

    std::mutex sensorMutex;
    std::mutex kukaduMutex;


    std::vector<double> sensorVal;


    std::shared_ptr<ros::NodeHandle> myNode;
    KUKADU_SHARED_PTR<kukadu::KukieControlQueue> robotinoQueue;
    std::shared_ptr<kukadu::MoveItKinematics> mvKin;
    std::shared_ptr<kukadu_thread> qThread;
    KUKADU_SHARED_PTR<kukadu_thread>recordingThread;
    std::shared_ptr<std::thread> moveThread;
    std::shared_ptr<std::thread> filterThread;



    std::shared_ptr<kukadu::SensorStorage> store;

    enum ControllerType {JACOBIAN,INVERSE,IK,HYBRID};

    Eigen::VectorXd stdToEigenVec(std::vector<double> myVec);
    std::vector<double> eigenToStdVec(Eigen::VectorXd myVec);

    std::vector<double> scaleForcesTorques(std::vector<double> myVec);
    std::vector<double> scaleJointCommands(std::vector<double> myVec);
    int sign(double x);
    bool isBigJump(std::vector<double> myVec);
    void sensorUpdate(std_msgs::Float64MultiArray msg);
    std::vector<double> capVec(std::vector<double> input, double maxCap);
    bool isColliding(std::vector<double> jointStates);
    bool isDifferentialCommandSafe(arma::vec diffCommand,arma::vec currentJointStates);
    arma::vec getNextDifferentialCommand(Eigen::MatrixXd jacobian,arma::vec currentJointState, ControllerType myType);
    void generateNextCommand();
    void filterHandler();
    void teachingThreadHandler();

public:

    KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic);
    void init();
    void startTeaching();
    void stopTeaching();
    void startRecording();
    void stopRecording();
    void play();
    void stopArm();
    void ptp(std::vector<double> target); //remove

};
}
#endif
