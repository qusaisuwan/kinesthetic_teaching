
#ifndef Filter_HPP
#define Filter_HPP

namespace UIBK_Teaching
{

class Filter{

    static auto constexpr  PI = 3.1412;
    static auto constexpr  SENSOR_MISS_ALIGNMENT_COMPARED_TO_END_EFFECTOR = -2.27;
    static auto constexpr  FILTER_FREQ = 50.0;

    static auto constexpr  SMOOTHING = 0.9;
    static auto constexpr  DC_FILTER1 = 0.99999;
    static auto constexpr  DC_FILTER2 = 0.9998;

    static auto constexpr  FORGET_SHRINK = 0.99;
    static auto constexpr  FORGET_EXPAND = 0.1;
    static auto constexpr  ACCEPTANCE_INTERVAL = 0.90;
	
    bool filtersRunning;
    bool firstReading;


    std::mutex transformedValueMutex;
    std::mutex filterSmoothingMutex;
    std::mutex filterDC1Mutex;
    std::mutex filterDC2Mutex;

    std::vector<double> projectedFilteredReadings;
    std::vector<double> smoothingFilterMemory;
    std::vector<double> DCFilter1Memory;
    std::vector<double> DCFilter2Memory;
    std::vector<std::pair<double,double>> limitsFilterMemory;


    std::shared_ptr<std::thread> filterSmoothingThread;
    std::shared_ptr<std::thread> filterDC1Thread;
    std::shared_ptr<std::thread> filterDC2Thread;

    void lowpassFilter(std::vector<double> &filter,std::vector<double> newVal, double forgettingFactor);
    void limitsFilter(std::vector<std::pair<double,double>> &filter,std::vector<std::vector<double>> newDataSet, double forgettingFactorShrink, double forgettingFactorExpand, double percentageWithinLimits);
    std::vector<double> removeBias(std::vector<double> values,std::vector<double> bias);
    std::vector<double> scaleAndLimitReadings(std::vector<double> msg);
    std::vector<double> projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma);
    std::vector<double> projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose);
    std::vector<double> getProcessedReading();

    void filterSmoothingThreadHandler();
    void filterDC1ThreadHandler();
    void filterDC2ThreadHandler();

	public:
//	Filter();
    void printFilteredSensorVal();
};

}

#endif
