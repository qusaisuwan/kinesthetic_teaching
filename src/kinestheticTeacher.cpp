#include "kinestheticTeacher.h"
#include "Eigen/Jacobi"

KinestheticTeacher::KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic){//(ros::NodeHandle &node)
    myNode =  std::shared_ptr<ros::NodeHandle> (&node);
    teacherRunning=false;
    sensorVal.data={0.0,0.0,0.0,0.0,0.0,0.0};

    // Kukadu

     robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", *myNode));
     vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
     mvKin = std::make_shared<MoveItKinematics>(robotinoQueue, node, "robotino", controlledJoints, "arm_link5");
     robotinoQueue->setKinematics(mvKin);
     robotinoQueue->setPathPlanner(mvKin);

     sensorUpdateSub = myNode->subscribe(sensorTopic, 1, &KinestheticTeacher::sensorUpdate,this);
}


void KinestheticTeacher::init(){


   qThread = robotinoQueue->startQueue();
   if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
       robotinoQueue->stopCurrentMode();
       robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
   }

   moveThread =std::make_shared<std::thread>(&KinestheticTeacher::teachingThread, this);
}


void KinestheticTeacher::generateNextPositionCommand(){
    sensorMutex.lock();
    auto msg = sensorVal;
    sensorMutex.unlock();

//    auto pose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//    cout << "pose in 0 joint position: " << pose.position.x << " " << pose.position.y << " " << pose.position.z << endl;

    auto jacobian = mvKin->getJacobian();
    Eigen::MatrixXd jacobianTranspose= jacobian.transpose();
   // auto jacobianInverse= jacobian.inverse();
//    cout << jacobian << endl;
//    cout << jacobianInverse << endl;
//    cout << jacobianTranspose << endl;
    Eigen::VectorXd forceVector(msg.data.size());

    for (int i=0;i< msg.data.size(); ++i)
        forceVector(i)= msg.data[i];
        //cout << "force vector is is " << forceVector << endl;
//    cout << forceVector<< endl;
//    cout << jacobianTranspose << endl;

    double scaling=0.04;
    Eigen::MatrixXd multiplierMatrix=capMatrix(jacobianTranspose, 2.0,robotinoQueue->getJointNames().size(),6);
    Eigen::MatrixXd jointSpaceDifferential = scaling * multiplierMatrix * forceVector;
    Eigen::MatrixXd test(8,6);

    for (int i=0;i < 8*6;i++){
        test(i)=10.0;
    }
    cout << "test " << test << endl;
    Eigen::MatrixXd res=capMatrix(jacobianTranspose, 2.0,robotinoQueue->getJointNames().size(),6);
    cout << "res " << res << endl;
    // jointSpaceDifferential = jacobianTranspose * forceVector;

    auto jointNextState=  armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);

    for (int i=0;i< jointNextState.size(); ++i){

       // jointNextState.at(i) = jointNextState.at(i) + jointSpaceDifferential(i);
    }

    vec target = (stdToArmadilloVec(jointNextState));

    //auto endJacobian = mvKin->getJacobian(armadilloToStdVec(end));
   // cout << "target is " << target << endl;
    // auto jointPlan = mvKin->planJointTrajectory({target});

//    cout << "planning in joint space" << endl;
//    auto jointPlan = mvKin->planJointTrajectory({start, end});
//    cout << "done with planning" << endl;

      // vector<vec> dcJointPlan= fixJointPlanTemp(jointPlan);

//        cout << "from: " << dcJointPlan.front().t() << "to: " << dcJointPlan.back().t() << endl;
//       cout << "joint planning worked (path of length " << dcJointPlan.size() << " generated) (press key to execute it)" << endl;
//       robotinoQueue->setNextTrajectory(dcJointPlan);
//       robotinoQueue->synchronizeToQueue(1);
   // cout  << target << endl;
       usleep(100000);

      // robotinoQueue->move(target);

    /* for moving with the queue!!!! */
//    vector<double> nextJointPositions = {0.0,0.0,0.0,0.0,0.0,0.0};\
 
 //JointPositions << endl\

    //robotinoQueue->submitNextJointMove(stdToArmadilloVec(nextJointPositions));


}

  Eigen::MatrixXd KinestheticTeacher::capMatrix(Eigen::MatrixXd input, double maxCap,int x, int y){//cap 6*jointsNumber matrix

      for (int i =0;i< x*y;i++)
          input(i)= std::isnan(input(i)) || std::isinf(input(i)) ? 0 : max(maxCap,input(i));
      return input;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeThinU | Eigen::ComputeThinV);

}

  void KinestheticTeacher::sensorUpdate(std_msgs::Float64MultiArray msg){
      sensorMutex.lock();
      sensorVal=msg;
      sensorMutex.unlock();
  }

  void KinestheticTeacher::runArm(){
    teacherRunning=true;
}

void KinestheticTeacher::teachingThread(){
    while(1){
        if (teacherRunning){

           generateNextPositionCommand();
        }

    }

}

void KinestheticTeacher::stopArm(){
    teacherRunning=false;
    robotinoQueue->stopQueue();
    moveThread->join();
    qThread->join();
}
