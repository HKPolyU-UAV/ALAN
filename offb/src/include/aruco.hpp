#include "essential.h"
#include <opencv2/aruco.hpp>

class aruco
{
    std::vector<int> markerIds;
    std::vector<Eigen::Matrix<int, 8, 1>> markerConerABCDs;
    Eigen::Matrix<int, 2, 1> markerCenter,last_markerCenter;
    Eigen::Matrix<int, 8, 1> markerConerABCD;
    Eigen::Matrix<int, 8, 1> last_markerConerABCD;

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Point2f> markerCorner;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3d rvec, tvec;
    bool Aruco_init = false;
    bool Aruco_found = false;

    cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
    cv::Mat depthcameraMatrix = cv::Mat::eye(3,3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

public:
    aruco(/* args */);
    ~aruco();
    void Aruco_process(cv::Mat frame);   
     
};

aruco::aruco(/* args */)
{
}

aruco::~aruco()
{
}

void aruco::Aruco_process(cv::Mat frame)
{
    cv::Mat ArucoOutput = frame.clone();

    rvecs.clear();
    tvecs.clear();

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    
    
    if (markerIds.size() > 0)
    {
        markerConerABCDs.clear();
        Aruco_init = true;
        Aruco_found = true;
        cv::aruco::drawDetectedMarkers(ArucoOutput, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.045, cameraMatrix, distCoeffs, rvecs, tvecs);


        for(unsigned int i=0; i<markerIds.size(); i++){
            cv::aruco::drawAxis(ArucoOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            markerCorner = markerCorners[i];
            for (unsigned int j=0; j<markerCorner.size();j++)
            {
                cv::Point2f MC = markerCorner[j];
                markerConerABCD[j*2] = MC.x;
                markerConerABCD[j*2+1] = MC.y;
            }
            markerConerABCDs.push_back(markerConerABCD);
        }
    }
    else
    {
        Aruco_found = false; 
        ArucoLostcounter++;
    }

    if (Aruco_init)
    {
        if(Aruco_found)
        {
            rvec = rvecs.front();
            tvec = tvecs.front();
            Eigen::Matrix<int, 2, 1> Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
            Aruco_PosePub(Camera2World(Aruco_translation_camera,Camera_lp));
        }
    }
    cv::imshow("uav", ArucoOutput);
    cv::waitKey(1);
}