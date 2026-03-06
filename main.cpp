#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    VideoCapture cap("C://Users//fehre//OneDrive//Documents//ENSEEIHT//IA_Drone//Drone_Mapper//DroneVideo.mp4");
    if (!cap.isOpened()) {
        cout << "Error" << endl;
        return -1;
    }

    // OBR initialisation
    Ptr<ORB> detector = ORB::create(100);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    Mat frame, prevFrame, prevDesc;
    vector<KeyPoint> prevKP;

	// Initialisation of the map and the homography matrix
    Mat map = Mat::zeros(1000, 1000, CV_8UC3);
    Mat H_global = Mat::eye(3, 3, CV_64F);

	// We start in the middle of the map
    H_global.at<double>(0, 2) = 500;
    H_global.at<double>(1, 2) = 500;
    int frameCount = 0;

    while (cap.read(frame)) {
		// Prossess one frame out of 10
        frameCount++;
        if (frameCount % 60 != 0) continue;
        resize(frame, frame, Size(), 0.2, 0.2, INTER_LINEAR);

        vector<KeyPoint> kp;
        Mat desc;
        detector->detectAndCompute(frame, noArray(), kp, desc);

        if (!prevFrame.empty() && !desc.empty() && !prevDesc.empty()) {
            vector<DMatch> matches;
            matcher->match(desc, prevDesc, matches);

            // keep the 20 best points
            sort(matches.begin(), matches.end());
            if (matches.size() > 20) matches.erase(matches.begin() + 20, matches.end());

            // extract the matched keypoints
            vector<Point2f> srcPts, dstPts;
            for (auto& m : matches) {
                srcPts.push_back(kp[m.queryIdx].pt);
                dstPts.push_back(prevKP[m.trainIdx].pt);
            }

            if (srcPts.size() >= 4) {
                // estimation of the mouvement
                Mat affine = estimateAffinePartial2D(srcPts, dstPts, noArray(), RANSAC);

                if (!affine.empty()) {
                    // Update the global homography
                    Mat H_local = Mat::eye(3, 3, CV_64F);
                    affine.copyTo(H_local.rowRange(0, 2));
                    H_global = H_global * H_local;

                    // Update the map

                // Image correction
                Mat warped_frame, mask;
                warpPerspective(frame, warped_frame, H_global, map.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
                cvtColor(warped_frame, mask, COLOR_BGR2GRAY);
                threshold(mask, mask, 1, 255, THRESH_BINARY);
                Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
                erode(mask, mask, kernel);

		        // update the map
                warped_frame.copyTo(map, mask);
                }
            }
        }

        // Show the result
        Mat display;
        resize(map, display, Size(), 1, 1); // Zoom back to see the map
        cv::imshow ("Drone Mapper", display);

        prevFrame = frame.clone();
        prevDesc = desc.clone();
        prevKP = kp;
        
        if (waitKey(1) == 27) break;
    }
    return 0;
}