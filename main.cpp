#include <iostream>
#include "Mapper.hpp"

int main() {
    cv::VideoCapture cap("C://Users//fehre//OneDrive//Documents//ENSEEIHT//IA_Drone//Drone_Mapper//DroneVideo.mp4");
    if (!cap.isOpened()) {
        std::cout << "Error" << std::endl;
        return -1;
    }

    Mapper myMapper; 
    cv::Mat frame;
    int frameCount = 0;

    while (cap.read(frame)) {
        frameCount++;
        if (frameCount % 50 != 0) continue;

        imshow("Drone Mapper", myMapper.update(frame));

        if (cv::waitKey(1) == 27) break;
    }
    return 0;
}