#include "Mapper.hpp"

using namespace cv;
using namespace std;

Mapper::Mapper() {
    // OBR initialisation 
    detector = ORB::create(100);
    matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // Initialisation of the map and the homography matrix
    map = Mat::zeros(1000, 1000, CV_8UC3);
    H_global = Mat::eye(3, 3, CV_64F);

    // starting point
    H_global.at<double>(0, 2) = 500;
    H_global.at<double>(1, 2) = 500;
}

cv::Mat Mapper::update(Mat frame) {
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

				// warp the current frame to the map coordinate system
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

	// update the previous frame, descriptors and keypoints
    prevFrame = frame.clone();
    prevDesc = desc.clone();
    prevKP = kp;

	// print the map
    return map;
}
