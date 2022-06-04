#include <sstream>
#include <cmath>
#include <Eigen/Dense>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"


#include "include/run_yolo.h"
#include <string>
#include "offb/obj.h"

using namespace std;
static cv::Mat frame, res, gt;

static cv::String weightpath ="/home/patty/Downloads/yolov4-tiny.weights";
static cv::String cfgpath ="/home/patty/Downloads/yolov4-tiny.cfg";
static cv::String classnamepath = "/home/patty/Downloads/coco.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.1));

static int counter = 0;



void callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
{

    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat image_dep = depth_ptr->image;
    

    Yolonet.getdepthdata(image_dep);

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
        res   = cv::imdecode(cv::Mat(rgbimage->data),1);
        gt    = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cout<<frame.size<<endl;
}


int main(int argc, char** argv)
{

    cv::VideoWriter video("/home/patrick/track_ws/src/offb/src/include/yolo/tracking.avi", cv::VideoWriter::fourcc('M','J','P','G'), 8, cv::Size(640,480));
    cv::VideoWriter videoyolo("/home/patrick/track_ws/src/offb/src/include/yolo/yolo.avi", cv::VideoWriter::fourcc('M','J','P','G'), 8, cv::Size(640,480));
    cv::VideoWriter videogt("/home/patrick/track_ws/src/offb/src/include/yolo/gt.avi", cv::VideoWriter::fourcc('M','J','P','G'), 8, cv::Size(640,480));
    


    int stateSize = 8;
    int measSize = 5;
    int contrSize = 0;
    

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z,w,h] need z as well
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    


    cv::setIdentity(kf.transitionMatrix);

    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(9) = 1.0f;
    kf.measurementMatrix.at<float>(18) = 1.0f;

    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(9) = 1e-2;
    kf.processNoiseCov.at<float>(18) = 1e-2;
    kf.processNoiseCov.at<float>(27) = 5.0f;
    kf.processNoiseCov.at<float>(36) = 5.0f;
    kf.processNoiseCov.at<float>(45) = 5.0f;
    kf.processNoiseCov.at<float>(54) = 1e-2;
    kf.processNoiseCov.at<float>(63) = 1e-2;

    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

    int idx = 0;

    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "yolotiny");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Publisher publish_obj_w = nh.advertise<geometry_msgs::Point>("/pose_w",1);
    ros::Publisher publish_obj_c = nh.advertise<geometry_msgs::PointStamped>("/pose_camera",1);
    ros::Publisher publish_found = nh.advertise<std_msgs::Bool>("/obj_found",1);
    ros::Publisher publish_obj_vel = nh.advertise<offb::obj>("/obj_v", 1);



    bool found = false;
    bool measured =false;
    int notFoundCount = 0;
    double dT;
    double tpf=0;
    int w = 200,h = 200;
    double ticks = 0;
    int i=0;

    cv::Point center_true;
    cv::Point center_pred;
    double depth = 0, depth_ = 0;
    double camera_z;
    double fps;
    double fps_average = 0;
    vector<double> fpss;
    double time_start, time_end;
    cv::Rect temprect;

    double confidence;
    cv::Rect predRect;


    while(ros::ok())
    {
        time_start = ros::Time::now().toSec();
        if(!frame.empty())
        {

            double precTick = ticks;
            ticks = (double) cv::getTickCount();

            dT = (ticks - precTick) / cv::getTickFrequency(); //seconds
            if (found)
            {
                // >>>> Matrix A
                kf.transitionMatrix.at<float>(3) = dT;
                kf.transitionMatrix.at<float>(12) = dT;
                kf.transitionMatrix.at<float>(21) = dT;

                // <<<< Matrix A
                //            cout << "dT:" << endl << dT << endl;
                //            cout << "State post:" << endl << state << endl;

                state = kf.predict();

                cv::Point center;
                center.x = state.at<float>(0);
                center.y = state.at<float>(1);
                double z_c_temp = state.at<float>(2);

                predRect.width = temprect.width;
                predRect.height = temprect.height;
                predRect.x = state.at<float>(0) - predRect.width / 2;
                predRect.y = state.at<float>(1) - predRect.height / 2;


                ofstream save("/home/patrick/track_ws/src/offb/src/include/yolo/gt.txt", ios::app);
                save<<counter<<endl;
                save<<predRect.x <<endl;
                save<<predRect.y <<endl;
                save<<predRect.x + predRect.width <<endl;
                save<<predRect.y + predRect.height<<endl<<endl;;
                save.close();


                //                cv::Scalar color(rand()&255, rand()&255, rand()&255);
                //cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
                center_pred = cv::Point(predRect.x+predRect.width/2, predRect.y+predRect.height/2);

                char temp_depth[40];
                sprintf(temp_depth, "%.2f", z_c_temp);
                string d = "Predicted z_C: ";
                string textoutputonframe =d+temp_depth + " m";
                cv::Point placetext = cv::Point((predRect.x-10),(predRect.y+predRect.height+24));
                //cv::putText(res, textoutputonframe, placetext,cv::FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(255,0,0));
                depth_ = z_c_temp;
            }

            double starttime = ros::Time::now().toSec();
            Yolonet.rundarknet(frame);
            double endtime = ros::Time::now().toSec();
            double deltatime = endtime - starttime;
            //cout<<deltatime<<endl;
            fps = 1/deltatime;
            cout<<"fps: "<<fps<<endl;
            if(fpss.size()>2000)
                fpss.clear();
            fpss.push_back(fps);
            fps_average = accumulate(fpss.begin(), fpss.end(),0.0)/fpss.size();
            cout<<"fps_avg: "<<fps_average<<endl;

            vector<objectinfo> temp = Yolonet.obj_vector;
            cout<<"temp size: "<<temp.size()<<endl;

            cv::Rect interested;
            vector<objectinfo> potential;
            vector<float> potential_c;
            vector<float> potential_c_area;
            bool got=false;

            cv::Mat tempp;
            if(temp.size()!=0)
            {
                for(auto stuff:temp)
                {
                    cout<<stuff.classnameofdetection<<endl;
                    if(stuff.classnameofdetection=="Pooh")
                    {
                        potential.push_back(stuff);
                        potential_c.push_back(stuff.confidence);
                        potential_c_area.push_back(stuff.boundingbox.area());
                    }
                }
                cout<<"potential size: "<<potential.size()<<endl;

                if(potential.size()!=0)
                {
                    //                int maxElementIndex = max_element(potential_c.begin(),potential_c.end()) - potential_c.begin();
                    int maxElementIndex = max_element(potential_c_area.begin(),potential_c_area.end()) - potential_c_area.begin();
                    interested = potential[maxElementIndex].boundingbox;
                    temprect = potential[maxElementIndex].boundingbox;

                    got = true;


                    tpf = Yolonet.appro_fps;
                    w=interested.width;
                    h=interested.height;
                    tempp = potential[maxElementIndex].frame;
                    depth = camera_z = potential[maxElementIndex].depth;
                    confidence = potential[maxElementIndex].confidence;

                    char temp_depth[40];
                    sprintf(temp_depth, "%.2f", depth);
                    string d = "z_C: ";
                    string textoutputonframe =d+temp_depth + " m";
                    cv::Point placetext = cv::Point((interested.x-10),(interested.y-24));


                    if(confidence > 75)
                    {
                        center_true=cv::Point(interested.x+interested.width/2, interested.y+interested.height/2);
                        cv::rectangle(res, interested, CV_RGB(255,255,0), 1);
                        //cv::putText(res, textoutputonframe, placetext,cv::FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(255,255,0));
                    }
                }
            }
            time_end=ros::Time::now().toSec();
            cout<<"ms: "<<time_end-time_start<<endl;
            if(!got)
            {
                notFoundCount++;
                measured = false;
                cout << "notFoundCount:" << notFoundCount << endl;
                if(notFoundCount>100)
                {
                    found = false;
                }
            }
            else
            {
                //            cout<<"hey"<<endl;
                measured = true;
                notFoundCount = 0;
                meas.at<float>(0) = interested.x + interested.width /  2;
                meas.at<float>(1) = interested.y + interested.height / 2;
                meas.at<float>(2) = camera_z;
                meas.at<float>(3) = interested.width;
                meas.at<float>(4) = interested.height;
                if (!found) // First detection!
                {
                    // >>>> Initialization
                    kf.errorCovPre.at<float>(0) = 1; // px
                    kf.errorCovPre.at<float>(9) = 1; // px
                    kf.errorCovPre.at<float>(18) = 1;
                    kf.errorCovPre.at<float>(27) = 1;
                    kf.errorCovPre.at<float>(36) = 1; // px
                    kf.errorCovPre.at<float>(45) = 1; // px
                    kf.errorCovPre.at<float>(54) = 1; // px
                    kf.errorCovPre.at<float>(63) = 1; // px

                    state.at<float>(0) = meas.at<float>(0);
                    state.at<float>(1) = meas.at<float>(1);
                    state.at<float>(2) = meas.at<float>(2);
                    state.at<float>(3) = 0;
                    state.at<float>(4) = 0;
                    state.at<float>(5) = 0;
                    state.at<float>(6) = meas.at<float>(3);
                    state.at<float>(7) = meas.at<float>(4);
                    // <<<< Initialization

                    kf.statePost = state;
                    found = true;
                }
                else
                {
                    kf.correct(meas); // Kalman Correction                    

                    cv::Point center;
                    center.x = state.at<float>(0);
                    center.y = state.at<float>(1);
                    double z_c_temp = state.at<float>(2);

                    cv::Rect Rect;
                    Rect.width = temprect.width;
                    Rect.height = temprect.height;
                    Rect.x = state.at<float>(0) - Rect.width / 2;
                    Rect.y = state.at<float>(1) - Rect.height / 2;
                    center_true=cv::Point(Rect.x+Rect.width/2, Rect.y+Rect.height/2);
                    if(confidence <= 75)
                        cv::rectangle(res, Rect, CV_RGB(0,255,0), 1);
                }
            }
            cv::Mat display;

            geometry_msgs::PointStamped send;
            if(measured)
            {
                cout<<"show measure: "<<endl;
                send.point.x = center_true.x;
                send.point.y = center_true.y;
                send.point.z = depth;
            }
            else
            {
                cout<<"show predict"<<endl;
                cv::rectangle(res, predRect, CV_RGB(255,0,0), 1);
                send.point.x = center_pred.x;
                send.point.y = center_pred.y;
                send.point.z = depth_;
            }
            publish_obj_c.publish(send);

            std_msgs::Bool obj_found;
            if(found)
                obj_found.data = true;
            else
                obj_found.data = false;
            publish_found.publish(obj_found);

            offb::obj obj_depth_v;
            obj_depth_v.Z_c = state.at<float>(5);
            cout<<endl<<obj_depth_v<<endl<<endl;;
            publish_obj_vel.publish(obj_depth_v);

//            cv::hconcat(tempp, res, display);
            //cv::line( res, cv::Point(320,240), center_pred, CV_RGB(100,0,255), 1, cv::LINE_AA);
            cv::Point text = cv::Point((320+center_pred.x)/2,(240+center_pred.y)/2);
            char temp_depth[40];
            sprintf(temp_depth, "%.2f", depth);
            string d = "distance: ";
            string textoutputonframe =d+temp_depth + " m";
            //cv::putText(res, textoutputonframe, text,cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(180,140,120));
            cv::imshow("Yolo", frame);
            cv::imshow("Tracking...", res);
            cv::waitKey(20);
            video.write(res);
            videoyolo.write(frame);


            cv::putText(gt, to_string(counter),cv::Point(20,20),cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(0,0,0));
            videogt.write(gt);
            counter++;
        }
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}

void testtrack()
{

}



