#include "Filter.h"

//Filter::Filter(){

//}
namespace UIBK_Teaching{
std::vector<double> Filter::scaleAndLimitReadings(std::vector<double> msg){
    int torquesStartingIndex=limitsFilterMemory.size()/2;
    double forceMin=FORCE_MIN_LIMIT;
    double torqueMin=TORQUE_MIN_LIMIT;

    for (int i=0;i<torquesStartingIndex;i++)
        msg.at(i)= min(1.0,max(-1.0,(sign(msg.at(i)) == 1 ? msg.at(i)/max(forceMin,fabs(limitsFilterMemory.at(i).second)) : msg.at(i)/max(forceMin,fabs(limitsFilterMemory.at(i).first)))));

    for (int i=torquesStartingIndex ;i<limitsFilterMemory.size();i++)
        msg.at(i)= min(1.0,max(-1.0,(sign(msg.at(i)) == 1 ? msg.at(i)/max(torqueMin,fabs(limitsFilterMemory.at(i).second)) : msg.at(i)/max(torqueMin,fabs(limitsFilterMemory.at(i).first)))));

    return msg;
}

std::vector<double> Filter::removeBias(std::vector<double> values,std::vector<double> bias){
    std::vector<double> res;
    for(int i=0;i< values.size();i++)
        res.push_back(values.at(i)-bias.at(i));
    return res;

}
void Filter::lowpassFilter(std::vector<double> &filter,std::vector<double> newVal, double forgettingFactor){
    for(int i=0;i< filter.size();i++)
        filter.at(i)= filter.at(i) * forgettingFactor + (1-forgettingFactor) * newVal.at(i);
}

void Filter::limitsFilter(std::vector<std::pair<double,double>> &filter,std::vector<std::vector<double>> newDataSet, double forgettingFactorShrink, double forgettingFactorExpand, double percentageWithinLimits){
    int vectorsNumber=newDataSet.size();
    for(int i=0;i< vectorsNumber;i++){
        std::vector<double> vectorReadings=newDataSet.at(i);
        std::sort(vectorReadings.begin(),vectorReadings.end());
        int limitIndex = (int)(percentageWithinLimits *vectorReadings.size());
        double lowerLimit=vectorReadings.at(vectorReadings.size()-limitIndex);
        double upperLimit=vectorReadings.at(limitIndex);
        filter.at(i).first = lowerLimit < filter.at(i).first ? (filter.at(i).first * forgettingFactorExpand + lowerLimit * (1-forgettingFactorExpand)) : (filter.at(i).first * forgettingFactorShrink + lowerLimit * (1-forgettingFactorShrink));
        filter.at(i).second = upperLimit > filter.at(i).second ? (filter.at(i).second * forgettingFactorExpand + upperLimit * (1-forgettingFactorExpand)) : (filter.at(i).second * forgettingFactorShrink + upperLimit * (1-forgettingFactorShrink));
        //           cout << " lowerLimit " << lowerLimit << "   filter.at(i).first " << filter.at(i).first  << " upperLimit" << upperLimit << "  filter.at(i).second " <<  filter.at(i).second << endl;
    }
}

void Filter::filterSmoothingThreadHandler(){
    ros::Rate myRate(FILTER_FREQ);
    int cnt=0;
    std::vector<std::vector<double>> newDataSet;
    newDataSet.resize(6);
    while(true){
        if (filtersRunning){
            sensorMutex.lock();
            auto current = sensorVal;
            sensorMutex.unlock();
            filterSmoothingMutex.lock();
            filterDC1Mutex.lock();
            auto valuesAC=removeBias(current,DCFilter1Memory);

            if (cnt < 100){
                for(auto i=0;i<valuesAC.size();i++)
                    newDataSet.at(i).push_back(valuesAC.at(i));

                cnt++;
            }else{
                limitsFilter(limitsFilterMemory,newDataSet,FORGET_SHRINK,FORGET_EXPAND,ACCEPTANCE_INTERVAL);
                newDataSet.clear();

                newDataSet.resize(6);
                cnt=0;
            }
            lowpassFilter(smoothingFilterMemory,(valuesAC),SMOOTHING);//
            filterDC1Mutex.unlock();
            kukaduMutex.lock();
            transformedValueMutex.lock();
            auto jointState = armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);
            projectedFilteredReadings=projectReadings(smoothingFilterMemory,mvKin->computeFk(jointState));
            transformedValueMutex.unlock();
            kukaduMutex.unlock();
            filterSmoothingMutex.unlock();

        }
        myRate.sleep();
    }

}

void Filter::filterDC1ThreadHandler(){
    ros::Rate myRate(FILTER_FREQ);

    while(true){
        if (filtersRunning){
            sensorMutex.lock();
            auto current = sensorVal;
            sensorMutex.unlock();

            filterDC1Mutex.lock();
            lowpassFilter(DCFilter1Memory,current,DC_FILTER1);
            filterDC1Mutex.unlock();

        }
        myRate.sleep();
    }

}
void Filter::filterDC2ThreadHandler(){
    ros::Rate myRate(FILTER_FREQ);

    while(true){
        if (filtersRunning){
            filterSmoothingMutex.lock();
            filterDC2Mutex.lock();
            lowpassFilter(DCFilter2Memory,smoothingFilterMemory,DC_FILTER2);
            filterDC2Mutex.unlock();
            filterSmoothingMutex.unlock();
        }
        myRate.sleep();
    }

}

std::vector<double> Filter::projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose){ //memory problem?
    tf::Quaternion quat(currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    //Fix senser miss-alignment in respect to end effector and wrong sensor roll yaw conventions!!
    vector<double> temp1 = projectVectors(readings.at(0),readings.at(1) ,readings.at(2),0.0,0.0,SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR);
    vector<double> temp2 = projectVectors(readings.at(3),readings.at(4) ,readings.at(5),0.0,0.0, PI + SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR);
    //Project based on joint states
    vector<double> res1 = projectVectors(temp1.at(0),temp1.at(1) ,temp1.at(2),roll,pitch,yaw);
    vector<double> res2 = projectVectors(temp2.at(0),temp2.at(1) ,temp2.at(2),roll,pitch,yaw);
    res1.insert(res1.end(),res2.begin(),res2.end());

    return res1;
}

vector<double> Filter::projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma){
    vector<double> newVec;
    double i= vecX * (cos(beta)*cos(gamma)) + vecY *(cos(gamma)* sin(alpha)* sin(beta) - cos(alpha)*sin(gamma)) + vecZ *(cos(alpha)*cos(gamma)*sin(beta) +sin(alpha)*sin(gamma));
    double j= vecX * (cos(beta)*sin(gamma)) + vecY *(cos(alpha)* cos(gamma) + sin(alpha)*sin(gamma)*sin(beta)) + vecZ *(-1*cos(gamma)*sin(alpha) +cos(alpha)*sin(beta)*sin(gamma));
    double k= vecX * (-1*sin(beta)) + vecY *(cos(beta)* sin(alpha)) + vecZ *(cos(alpha)*cos(beta)); // ref: roll-x-alpha pitch-y-beta yaw-z-gamma

    //    double i= vecX * (cos(alpha)*cos(beta)) + vecY *(cos(alpha)* sin(beta)* sin(gamma) - sin(alpha)*cos(gamma)) + vecZ *(cos(alpha)*sin(beta)*cos(gamma) +sin(alpha)*sin(gamma));
    //    double j= vecX * (sin(alpha)*cos(beta)) + vecY *(cos(alpha)* cos(gamma) + sin(alpha)*sin(gamma)*sin(beta)) + vecZ *(-1*cos(alpha)*sin(gamma) +sin(alpha)*sin(beta)*cos(gamma));
    //    double k= vecX * (-1*sin(beta)) + vecY *(cos(beta)* sin(gamma)) + vecZ *(cos(gamma)*cos(beta)); // ref: roll-x-gamma pitch-y-beta yaw-z-alpha
    newVec.push_back(i);
    newVec.push_back(j);
    newVec.push_back(k);
    return newVec;

}
std::vector<double>  Filter::getProcessedReading(){
    transformedValueMutex.lock();
    filterDC2Mutex.lock();
    std::vector<double>  res = scaleAndLimitReadings(removeBias(projectedFilteredReadings,DCFilter2Memory));
    filterDC2Mutex.unlock();
    transformedValueMutex.unlock();
    return res;
}


void Filter::printFilteredSensorVal(){

    //    filterSmoothingMutex.lock();
    //    auto readings = limitsFilterMemory;//getProcessedReading();
    //    filterSmoothingMutex.unlock();

    auto readings = getProcessedReading();

    cout << "Force/Torque sensor values are : ";
    for (int i=0;i< readings.size();i++)
        cout << setw(13) << fixed << setprecision(5)  << readings.at(i);
    cout << endl;

}

}
