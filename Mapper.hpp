#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

class Mapper {
private:
    cv::Ptr<cv::ORB> detector;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat map;
    cv::Mat H_global;
    cv::Mat prevFrame, prevDesc;
    std::vector<cv::KeyPoint> prevKP;

public:
    Mapper(); 
    cv::Mat update(cv::Mat frame);
};