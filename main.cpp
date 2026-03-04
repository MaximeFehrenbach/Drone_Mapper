#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() {
    // 1. Chargement de la vidéo
    VideoCapture cap("C://Users//fehre//OneDrive//Documents//ENSEEIHT//IA_Drone//Drone_Mapper//Test_Mapper.mp4");
    if (!cap.isOpened()) {
        cout << "Erreur : Impossible d'ouvrir la vidéo." << endl;
        return -1;
    }

    // 2. Initialisation ORB (Plus de points pour plus de précision)
    Ptr<ORB> detector = ORB::create(2000);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    Mat frame, prevFrame, prevDesc;
    vector<KeyPoint> prevKP;

    // 3. Canvas Géant
    Mat canvas = Mat::zeros(4000, 4000, CV_8UC3);
    Mat H_global = Mat::eye(3, 3, CV_64F);

    // Position de départ au centre du canvas
    H_global.at<double>(0, 2) = 1500;
    H_global.at<double>(1, 2) = 1500;

    while (cap.read(frame)) {
        // Optionnel : resize pour fluidité si vidéo 4K
        // resize(frame, frame, Size(), 0.5, 0.5);

        vector<KeyPoint> kp;
        Mat desc;
        detector->detectAndCompute(frame, noArray(), kp, desc);

        if (!prevFrame.empty() && !desc.empty() && !prevDesc.empty()) {
            vector<DMatch> matches;
            matcher->match(desc, prevDesc, matches);

            // Filtre de qualité : on ne garde que les 100 meilleurs
            sort(matches.begin(), matches.end());
            if (matches.size() > 100) matches.erase(matches.begin() + 100, matches.end());

            vector<Point2f> srcPts, dstPts;
            for (auto& m : matches) {
                srcPts.push_back(kp[m.queryIdx].pt);
                dstPts.push_back(prevKP[m.trainIdx].pt);
            }

            if (srcPts.size() >= 4) {
                // --- ASTUCE STABILITE : ESTIMATE AFFINE AU LIEU DE HOMOGRAPHY ---
                // Calcule translation + rotation + scale uniquement
                Mat affine = estimateAffinePartial2D(srcPts, dstPts, noArray(), RANSAC);

                if (!affine.empty()) {
                    // Convertir la matrice 2x3 en 3x3 pour la multiplication
                    Mat H_local = Mat::eye(3, 3, CV_64F);
                    affine.copyTo(H_local.rowRange(0, 2));

                    // Accumulation
                    H_global = H_global * H_local;

                    // Dessin sur le canvas
                    // 1. Créer une image temporaire pour la frame déformée
                    Mat warped_frame;
                    warpPerspective(frame, warped_frame, H_global, canvas.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));

                    // 2. Créer un masque : là où la nouvelle image n'est pas noire
                    Mat mask;
                    cvtColor(warped_frame, mask, COLOR_BGR2GRAY);
                    threshold(mask, mask, 1, 255, THRESH_BINARY);

                    // 3. Copier uniquement la nouvelle image sur le canvas en utilisant le masque
                    warped_frame.copyTo(canvas, mask);
                }
            }
        }

        // Affichage du résultat réduit
        Mat display;
        resize(canvas, display, Size(), 0.2, 0.2); // Zoom arrière pour voir la carte
        imshow("Drone Mapper - Press ESC to quit", display);

        prevFrame = frame.clone();
        prevDesc = desc.clone();
        prevKP = kp;

        if (waitKey(1) == 27) break;
    }

    // Sauvegarde finale pour ton CV
    imwrite("ma_carte_drone.jpg", canvas);
    return 0;
}