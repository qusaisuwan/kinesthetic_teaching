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



namespace UIBK_Teaching
{

class KinestheticTeacher{


    static auto constexpr FORCE_MIN_LIMIT = 0.3;   // Limits that auto scaling cannot go below
    static auto constexpr TORQUE_MIN_LIMIT = 0.02; // Limits that auto scaling cannot go below

    static auto constexpr FORCES_MOVING_MULTIPLIER = 0.1;
    static auto constexpr TORQUES_MOVING_MULTIPLIER = 0.26;
    static auto constexpr Z_FORCE_MOVING_MULTIPLIER = 0.2; //temp

    static auto constexpr BASE_XY_MOVING_MULTIPLIER = 0.05;//0.6
    static auto constexpr BASE_Z_MOVING_MULTIPLIER = 0.1;//0.2
    static auto constexpr ARM_ALLJOINTS_MOVING_MULTIPLIER = 0.15;

    static auto constexpr MAXIMUM_JOINT_STEP = 0.1;

    bool teacherRunning;

    ros::Subscriber sensorUpdateSub;

    std::mutex sensorMutex;
    std::mutex kukaduMutex;


    std::vector<double> sensorVal;


    std::shared_ptr<ros::NodeHandle> myNode;
    KUKADU_SHARED_PTR<kukadu::KukieControlQueue> robotinoQueue;
    std::shared_ptr<kukadu::MoveItKinematics> mvKin;
    std::shared_ptr<kukadu_thread> qThread;
    std::shared_ptr<std::thread> moveThread;


    enum ControllerType {JACOBIAN,IK};

    int sign(double x);
    void sensorUpdate(std_msgs::Float64MultiArray msg);
    std::vector<double> capVec(std::vector<double> input, double maxCap);
    bool isColliding(std::vector<double> jointStates);
    bool isDifferentialCommandSafe(arma::vec diffCommand,arma::vec currentJointStates);
    arma::vec getNextDifferentialCommand(Eigen::MatrixXd jacobian, ControllerType myType);
    void generateNextCommand();


    void teachingThreadHandler();

public:

    KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic);
    void init();
    void runArm();
    void stopArm();

    void ptp(std::vector<double> target); //remove

};
}
#endif
