/*
 * main.cpp
 *
 *@author: Bryan Wodi
 *@date:  Oct 13, 2016
 *      ADAPTED from robotis tutorial {ball_following}
 *@purpose: sprint_two_colors FIRA2016. Using OPENCV 2.4.13
 */
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <math.h>
#include <libgen.h>
#include <signal.h>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"
#include "VisionMode.h"

#include "CompilerDefinitions.hpp"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "config.ini"
#define SCRIPT_FILE_PATH    "script.asc"


#define U2D_DEV_NAME        "/dev/ttyUSB0"


//variable definitions

//this contains the values used in thresholding the 
//three images/colors: lowH1,lowH2,lowH3,lowS1,lowS2,lowS3,lowV1,lowV2,lowV3...etc
// where 1, 2 and 3 represent each of the three colors
static int hsv_values[12];

//for image processing...
static const int canny_threshold = 100;
static const int canny_ratio = 3;
static const int canny_kernel_size = 3;

//flag for indicating direction & yellow patch tracing
// static bool going_backwards = false;
// static bool tracing_yellow = false;


int m_NoTargetMaxCount = 10;
int m_NoTargetCount = m_NoTargetMaxCount;
int m_KickTargetMaxCount = 10;
int m_KickTargetCount = 0;

double m_MaxFBStep = 12;
double m_MaxRLStep;
double m_MaxDirAngle;

double m_KickTopAngle = -5.0;
double m_KickRightAngle = -30.0;
double m_KickLeftAngle = 30.0;

double m_FollowMaxFBStep = 12;//30.0;
double m_FollowMinFBStep = 5.0;
double m_FollowMaxRLTurn = 35.0;
double m_FitFBStep = 3.0;
double m_FitMaxRLTurn = 35.0;
double m_UnitFBStep = 0.0;//0.3;
double m_UnitRLTurn = 1.0;

double m_BPace = 800;
double m_UnitBPace = -0.3;
double m_MaxBPace = 600;

double m_GoalFBStep = 0;
double m_GoalRLTurn= 0;
double m_FBStep= 0;
double m_RLTurn= 0;


enum SPRINT_STATE
{
    FORWARD_PINK,
    FORWARD_GREEN,
    BACKWARD_GREEN,
    BACKWARD_PINK
};


//for walking after target's been seen. x_amp will be the X_MOVE_AMP
static void m_motion(Point2D target, double x_amp)
{
    if(target.X == -1.0 || target.Y == -1.0) // no target
    {
        if(m_NoTargetCount > m_NoTargetMaxCount)
        {
            // can not find a target
            m_GoalFBStep = 0;
            m_GoalRLTurn = 0;
            Head::GetInstance()->MoveToHome();
    
            VERBOSE("cannot find target");
        }
        else
        {
            m_NoTargetCount++;
        }
    }
    else
    {
        m_NoTargetCount = 0;

        double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
        double pan_range = Head::GetInstance()->GetLeftLimitAngle();
        double pan_percent = pan / pan_range;

        double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
        double tilt_min = Head::GetInstance()->GetBottomLimitAngle();       
        double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
        double tilt_percent = (tilt - tilt_min) / tilt_range;

        if(tilt_percent < 0)
            tilt_percent = -tilt_percent;

        if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
        {
            if(tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE))
            {
                if(target.Y < m_KickTopAngle)
                {
                    m_GoalFBStep = 0;
                    m_GoalRLTurn = 0;

                    if(m_KickTargetCount >= m_KickTargetMaxCount)
                    {
                        m_FBStep = 0;
                        m_RLTurn = 0;      
                    }
                }
                else
                {
                    m_KickTargetCount = 0;
                    m_GoalFBStep = m_FitFBStep;
                    m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
                }
            }
            else
            {
                m_KickTargetCount = 0;
                m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;
                if(m_GoalFBStep < m_FollowMinFBStep)
                    m_GoalFBStep = m_FollowMinFBStep;
                if(m_GoalFBStep > m_FollowMaxFBStep)
                    m_GoalFBStep = m_FollowMaxFBStep;
                m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
            }
        }
        else
        {
            m_KickTargetCount = 0;
            m_GoalFBStep = 0;
            m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
        }       
    }

    if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0)
    {
        if(Walking::GetInstance()->IsRunning() == true)
            Walking::GetInstance()->Stop();
        else
        {
            if(m_KickTargetCount < m_KickTargetMaxCount)
                m_KickTargetCount++;
        }
    }
    else
    {
        if(Walking::GetInstance()->IsRunning() == false)
        {//changed to test ini
            m_FBStep = 0;
            m_RLTurn = 0;
            m_KickTargetCount = 0;
            Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
            Walking::GetInstance()->Start();            
        }
        else
        {
            Walking::GetInstance()->X_MOVE_AMPLITUDE = x_amp; 

            //TODO: do walk state check to adjust RL turn
            if(m_RLTurn < m_GoalRLTurn)
                m_RLTurn += m_UnitRLTurn;
            else if(m_RLTurn > m_GoalRLTurn)
                m_RLTurn -= m_UnitRLTurn;

            Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
            fprintf(stderr, "(Pace: %f m_RLTurn: %f)\n", Walking::GetInstance()->PERIOD_TIME,Walking::GetInstance()->A_MOVE_AMPLITUDE);
        }
    }   
}

static double getDist(cv::Point p0, cv::Point p1)
{
    return sqrt( (p1.x- p0.x)*(p1.x - p0.x) + (p1.y - p0.y)*(p1.y - p0.y));
}


static void adjust_gait()
{
    Walking::GetInstance()->PERIOD_TIME = 700;
    Walking::GetInstance()->X_MOVE_AMPLITUDE = -1;
    Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;

}

// //return 2D distance between two points
// static float get_2D_distance( cv::Point& pt1,  cv::Point& pt2)
// {//based on the Euclidean plane
//     float diffX = pt1.x - pt2.x;
//     float diffY = pt1.y - pt2.y;
//     return sqrt( (diffX * diffX) + (diffY * diffY) );
// }

// This is used later by std::sort()
struct sortComparator {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y);}
} sortbyx;
/// input: main camera frame for marking, Thresholded HSV image, default value: minSize =200, maxSize = 10000 
/// return marker: (x,y,size)
cv::Point3f findMarker(cv::Mat &camFrame,  cv::Mat &hsvThreshold, int minSize, int maxSize)
{ 
    
    cv::Point3f marker;
    cv::vector<cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;

    cv::Mat cannyFrame;

    /// closing operation to fill the black pixels
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 20, 20));
    cv::morphologyEx(hsvThreshold, hsvThreshold, 3,element ); /// Opening: MORPH_OPEN : 2 Closing: MORPH_CLOSE: 3
        
    cv::Canny( hsvThreshold, cannyFrame, 100,300/*value_Canny_Low, value_Canny_High*/, 3 );
    
    ///Find all contours from canny edges
    cv::findContours( cannyFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    // cv::Mat result = cv::Mat::zeros(mat_frame.size(), CV_8UC3 ); 
    
    /// Get the moments
    cv::vector<cv::Moments> mu(contours.size() );
  
    for( int i = 0; i < contours.size(); i++ )
     { mu[i] = cv::moments( contours[i], false ); }

    /// Get the mass centers:
    cv::vector<cv::Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
     { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
    
    cv::vector<cv::Point> approxArrow;
    
    ///Use contours to match polygons
    for(int i = 0; i < contours.size(); i++){
        int patchSize = mu[i].m00;

        bool checkSign = false; // rectangle of checks
        if (patchSize > minSize && patchSize < maxSize) // moment area: if relatively large contour (frame) is detected
        {
            
            //drawContours(result, contours, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
            //approxPolyDP(contours[i], approxArrow, arcLength(Mat(contours[i]), true)*0.02, true);
            //convexHull( contours[i], approxArrow, false );
            //drawContours(result, approxArrow, i, CV_RGB(255,255,255 ), 1, 8, hierarchy, 0, cv::Point() );
            cv::approxPolyDP(contours[i], approxArrow, arcLength(cv::Mat(contours[i]), true)*0.05, true);
            //for (int j = 0; j < approxArrow.size(); ++j)
                //cout << "Convex" << approxArrow.size() << endl;
            
            /// arrow sign with 4 points
            if(approxArrow.size() == 4) 
            {
                //cout << "Detect Rect:" << mu[i].m00 << endl;
                // int angle[4];       
                /// sort vector using myobject as comparator
                sort(approxArrow.begin(), approxArrow.end(), sortbyx);
                std::vector<cv::Point> patchCorners;
                patchCorners = approxArrow;
               
                /// arrange the corner position if the drawing pad is not align properly, only if skew to the left
                /// clockwise order from top-left
                /// useful it for ROI extraction and analysis
                if (approxArrow[0].x> approxArrow[1].x)
                {
                    patchCorners[0] = approxArrow[1];
                    patchCorners[1] = approxArrow[0];
                    patchCorners[2] = approxArrow[2];
                    patchCorners[3] = approxArrow[3];
                }
                else
                {
                    patchCorners[0] = approxArrow[0];
                    patchCorners[1] = approxArrow[1];
                    patchCorners[2] = approxArrow[3];
                    patchCorners[3] = approxArrow[2];
                }
                    
                double pointDist[4];
                pointDist[0] = getDist(patchCorners[0], patchCorners[1]);
                pointDist[1] = getDist(patchCorners[1], patchCorners[2]);
                pointDist[2] = getDist(patchCorners[2], patchCorners[3]);
                pointDist[3] = getDist(patchCorners[3], patchCorners[0]);
                
                /// threshold for error between edges of square
                int distError = 60;  
                if ( (abs(pointDist[0] - pointDist[1]) + abs(pointDist[1] - pointDist[2]) + abs(pointDist[2] - pointDist[3])  + abs(pointDist[3] - pointDist[0])) < distError)
                {
                    checkSign = true;
                }
                
            
                if ( checkSign == true) /// confirmation of square marker
                {                               
                    cv::drawContours( camFrame, contours, i, cv::Scalar(255, 0, 0), 2, 8, hierarchy, 0, cv::Point() );
                    
                    for(int p = 0; p < 4; ++p)
                    {
                        if (p == 0) cv::circle(camFrame, patchCorners[p], 3, cv::Scalar(0, 0, 255), 1); // red
                        else if (p == 1) cv::circle(camFrame, patchCorners[p], 3, cv::Scalar(0, 255, 0), 1); // green
                        else if (p == 2) cv::circle(camFrame, patchCorners[p], 3, cv::Scalar(255, 0, 0), 1); // blue
                    }
                    float x = mu[i].m10/mu[i].m00;
                    float y = mu[i].m01/mu[i].m00;
                    marker = cv::Point3f(x,y,patchSize);
                                    
                } // IF confirmed rectangle
                else
                {
                    marker = cv::Point3f (-1,-1,-1);
                }
            } // IF four corner polygon 
        } // IF large moments
    } // FOR contour loop
    return marker;
} /// findArrowPatch

//used by main to find target
//params: two diff binary frames
//return: 3-point float containing (x,y,dist. btw x&y).
//returns (-1,-1,-1) if target not found
static cv::Point3f find_target(cv::Mat& cam_frame,  cv::Mat& frame_a, int color)
{
    cv::Point3f target_centre(-1,-1,-1);

    cv::Point center_A, center_B;
    cv::Point3f p0;

    float size_a;

    if (color == 0)
    {//pink
        p0 = findMarker(cam_frame, frame_a, 150,15000);
    }
    else if(color == 1)
    {//green
        p0 = findMarker(cam_frame, frame_a, 150,700);
    }
    

    size_a = p0.z;
    VERBOSE("marker.x: "<< p0.x<<" marker.y: "<<p0.y<<" marker.z: "<<size_a);

    if ( size_a != -1 )
    {
        center_A.x = p0.x;
        center_A.y = p0.y;

        cv::circle(cam_frame,cv::Point(center_A.x,center_A.y),3,cv::Scalar(255,0,0));
        
        // float curr_distance = get_2D_distance(center_A,center_B);
        target_centre = cv::Point3f(center_A.x,center_A.y, size_a);
    }
    
    return target_centre;
}


//sets the range values for the HSV
//based on openCV documentation
static void initialize_hsv_array()
{
    for(int i = 0; i<12;i++)
    {
        if(i <= 5)
            hsv_values[i] = 0;
        else if(i >5 && i <8)
            hsv_values[i] = 179;
        else
            hsv_values[i] = 255;
    }

    //default vals for thresholding
    //pink
    hsv_values[0] = 128;
    hsv_values[2] = 100;
    //green
    hsv_values[3] = 141;
    hsv_values[7] = 60;
}

//before the race starts, you want to calibrate the vals for 
//all 2colors. this receives d_x which is an offset (0 or 1)
//that sets the thresh vals for the corresponding color
//0 -- means, set HSV vals for 1st color (pink)
//1 -- set HSV vals for the 2nd color (green)

//in the main, just call this method 3ice with 0, 1 
//to set the thresh vals for all 2 colors
static void set_range_params(int d_x)
{
    VERBOSE("Setting the range for thresholding");
    VERBOSE("Press the ENTER key when finished!\n");


    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    
    //values for inRange thresholding of red colored objects
    cv::namedWindow("Colour Control", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Threshold Image", CV_WINDOW_AUTOSIZE);


    cvCreateTrackbar("LowH", "Colour Control", &hsv_values[d_x], 179);
    cvCreateTrackbar("HighH", "Colour Control", &hsv_values[d_x+6], 179);

    cvCreateTrackbar("LowS", "Colour Control", &hsv_values[d_x+2], 255);
    cvCreateTrackbar("HighS", "Colour Control", &hsv_values[d_x+8], 255);

    cvCreateTrackbar("LowV", "Colour Control", &hsv_values[d_x+4], 255);
    cvCreateTrackbar("HighV", "Colour Control", &hsv_values[d_x+10], 255);

    while( true )
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        cv::Mat curr_frame=cv::Mat(rgb_output->m_Height,rgb_output->m_Width,CV_8UC3,rgb_output->m_ImageData);
        cv::Mat curr_hsv,curr_canny, curr_thresholded;
        
        if( curr_frame.data )
        {        
            //first convert cam image to bgr before hsv
            cv::cvtColor(curr_frame,curr_frame,cv::COLOR_RGB2BGR);
            cv::cvtColor(curr_frame,curr_hsv,cv::COLOR_BGR2HSV);

            curr_thresholded = cv::Mat(curr_frame.size(),CV_8UC1);
            
            cv::inRange(curr_hsv,cv::Scalar(hsv_values[d_x],hsv_values[d_x+2],hsv_values[d_x+4]),cv::Scalar(hsv_values[d_x+6],hsv_values[d_x+8],hsv_values[d_x+10]),curr_thresholded);
            // cv::GaussianBlur(curr_thresholded,curr_thresholded,cv::Size(9,9),2,2);
            cv::erode(curr_thresholded,curr_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
            cv::Canny(curr_thresholded,curr_canny,canny_threshold,canny_threshold*canny_ratio,canny_kernel_size);
            // cv::dilate(curr_thresholded,curr_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
        }
        
        cv::Moments muA = cv::moments(curr_thresholded, true);
        VERBOSE(muA.m00);
        
       
        // cv::imshow("Canny Image",curr_canny);
        cv::imshow("Threshold Image",curr_thresholded);
        cv::imshow("Colour Control",curr_frame);
        
        if(cv::waitKey(30) == 10) 
        {
            cv::destroyAllWindows();
            break;
        }
    }
}

static void flush_left_right(cv::Mat& frame, double ratio)
{
    cv::rectangle(frame, cv::Point(0,0), cv::Point(frame.size().width*ratio, frame.size().height), cv::Scalar(0,0,0),-1,1);
    cv::rectangle(frame, cv::Point(frame.size().width*(1.0 - ratio),0), cv::Point(frame.size().width, frame.size().height), cv::Scalar(0,0,0),-1,1);
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    exit(0);
}

int main(void)
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
    follower.DEBUG_PRINT = true;

    httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        #ifdef DEBUG
        VERBOSE("Failed to initialize Motion Manager!");
        #else
        printf("Failed to initialize Motion Manager!\n");
        #endif
        return 0;
    }
    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    MotionManager::GetInstance()->LoadINISettings(ini);

    
    int n = 0;
    int param[JointData::NUMBER_OF_JOINTS * 5];
    int wGoalPosition, wStartPosition, wDistance;

    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
        wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
        if( wStartPosition > wGoalPosition )
            wDistance = wStartPosition - wGoalPosition;
        else
            wDistance = wGoalPosition - wStartPosition;

        wDistance >>= 2;
        if( wDistance < 8 )
            wDistance = 8;

        param[n++] = id;
        param[n++] = CM730::GetLowByte(wGoalPosition);
        param[n++] = CM730::GetHighByte(wGoalPosition);
        param[n++] = CM730::GetLowByte(wDistance);
        param[n++] = CM730::GetHighByte(wDistance);
    }
    cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //stand up and stage for thresholding and running 
    // LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    usleep( ((Walking::GetInstance()->PERIOD_TIME * 7) ) * 1000);

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    MotionManager::GetInstance()->SetEnable(true);
    Walking::GetInstance()->STEP_FB_RATIO = 1.0;
    
    initialize_hsv_array(); //set range for thresholding 


    Head::GetInstance()->MoveByAngle(0,40);
    //now calibrate the 2 colors. see method def
    set_range_params(0);//pink
    set_range_params(1);//green


    // #ifdef DEBUG
    //this shows you the vals as explained
    // VERBOSETP("iLowH1: ",hsv_values[0]);
    // VERBOSETP("iHighH1: ",hsv_values[6]);
    // VERBOSETP("iLowS1: ",hsv_values[2]);
    // VERBOSETP("iHighS1: ",hsv_values[8]);
    // VERBOSETP("iLowV1: ",hsv_values[4]);
    // VERBOSETP("iHighV1: ",hsv_values[10]);
    // printf("\n");
    // VERBOSETP("iLowH2: ",hsv_values[1]);
    // VERBOSETP("iHighH2: ",hsv_values[7]);
    // VERBOSETP("iLowS2: ",hsv_values[3]);
    // VERBOSETP("iHighS2: ",hsv_values[9]);
    // VERBOSETP("iLowV2: ",hsv_values[5]);
    // VERBOSETP("iHighV2: ",hsv_values[11]);
    // #endif
    
    printf("All set!\n");
    // printf("Switch to Vision Mode to begin!\n");
    printf("\nhit ENTER\n");
    getchar();

    //starting pace is 600
    Walking::GetInstance()->PERIOD_TIME = 600;

    //these are the x and y values for the target.
    //default vals = -1
    float iLastX = -1; 
    float iLastY = -1;

    //hsv image and the two threshold images for for both colors
    //note: pink_threshold and green_threshold will be binary images
    cv::Mat img_hsv, pink_threshold, green_threshold;

    StatusCheck::Check(cm730);
    StatusCheck::m_cur_mode = SPRINT;

    SPRINT_STATE curr_state = FORWARD_PINK;

    while( true )
    {
        StatusCheck::Check(cm730);

        if(StatusCheck::m_cur_mode == SPRINT)
        {
            while( true )
            {
                LinuxCamera::GetInstance()->CaptureFrame();
                memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
                cv::Mat mat_frame=cv::Mat(rgb_output->m_Height,rgb_output->m_Width,CV_8UC3,rgb_output->m_ImageData);
                
                if (mat_frame.data)
                {
                    //flush
                    flush_left_right(mat_frame, 0.2);
                    //convert to BGR before converting to HSV
                    cv::cvtColor(mat_frame,mat_frame,cv::COLOR_RGB2BGR);
                    cv::cvtColor(mat_frame,img_hsv,cv::COLOR_BGR2HSV);
                    
                    //initializing threshold mat images as single channel images
                    pink_threshold = cv::Mat(mat_frame.size(),CV_8UC1);
                    green_threshold = cv::Mat(mat_frame.size(),CV_8UC1);

                    cv::Point3f img_target; //target contains x, y and some other param
                    float image_area = 0.0;//area of yellow patch

                    switch(curr_state)
                    {
                        case FORWARD_PINK:
                            VERBOSE("FORWARD_PINK");
                            cv::inRange(img_hsv,cv::Scalar(hsv_values[0], hsv_values[2], hsv_values[4]),cv::Scalar(hsv_values[6], hsv_values[8], hsv_values[10]),pink_threshold);
                            //removing false positives
                            // cv::erode(pink_threshold,pink_threshold,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

                            //find target based on two colours. pass two binary images to find_target. returns (x,y,distance btw x&y)
                            img_target = find_target(mat_frame, pink_threshold,0);
                            image_area = img_target.z;
                            if ( image_area != -1 && image_area != 0 )
                            {
                                VERBOSE("FORWARD_PINK area: "<<image_area);

                                if (image_area > 3400)
                                {
                                    //slow down...
                                    Walking::GetInstance()->PERIOD_TIME = 850;
                                    curr_state = FORWARD_GREEN;
                                }
                                else
                                {
                                    //pace
                                    Walking::GetInstance()->PERIOD_TIME = 600;

                                    //update the x and y co-ordinatimg_targetes of the target
                                    iLastX = img_target.x;
                                    iLastY = img_target.y;

                                    Point2D marker(iLastX,iLastY);
                                    tracker.Process(marker);
                                    m_motion(tracker.ball_position,10);
                                }
                            }
                            else
                            {
                                adjust_gait();
                            }
                        break;
                        case FORWARD_GREEN:
                            VERBOSE("FORWARD_GREEN");
                            //applying respective thresholding based on calibrated vals from set_range_params()
                            cv::inRange(img_hsv,cv::Scalar(hsv_values[1], hsv_values[3], hsv_values[5]),cv::Scalar(hsv_values[7], hsv_values[9], hsv_values[11]),green_threshold);
                            //removing false positives
                            // cv::erode(green_threshold,green_threshold,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

                            //find target based on two colours. pass two binary images to find_target. returns (x,y,distance btw x&y)
                            img_target = find_target(mat_frame, green_threshold,1); 
                            image_area = img_target.z;
                            if ( image_area != -1 && image_area != 0 )
                            {
                                VERBOSE("FORWARD_GREEN area: "<<image_area);
                                
                                if (image_area > 550)
                                {
                                    Walking::GetInstance()->PERIOD_TIME = 850;
                                    curr_state = BACKWARD_GREEN;
                                }
                                else
                                {
                                    Walking::GetInstance()->PERIOD_TIME = 850;

                                    //update the x and y co-ordinatimg_targetes of the target
                                    iLastX = img_target.x;
                                    iLastY = img_target.y;

                                    Point2D marker(iLastX,iLastY);
                                    tracker.Process(marker);
                                    m_motion(tracker.ball_position,0);
                                }
                            }
                            else
                            {
                                adjust_gait();
                            }
                        break;
                        case BACKWARD_GREEN:
                            VERBOSE("BACKWARD_GREEN");

                            //set hip pitch
                            Walking::GetInstance()->HIP_PITCH_OFFSET = 10.6;
                            //applying respective thresholding based on calibrated vals from set_range_params()
                            cv::inRange(img_hsv,cv::Scalar(hsv_values[1], hsv_values[3], hsv_values[5]),cv::Scalar(hsv_values[7], hsv_values[9], hsv_values[11]),green_threshold);
                            //removing false positives
                            // cv::erode(green_threshold,green_threshold,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

                            //find target based on two colours. pass two binary images to find_target. returns (x,y,distance btw x&y)
                            img_target = find_target(mat_frame, green_threshold,1); 
                            image_area = img_target.z;
                            if ( image_area != -1 && image_area != 0 )
                            {
                                VERBOSE("BACKWARD_GREEN area: "<<image_area);
                                
                                if (image_area < 450)
                                {
                                    Walking::GetInstance()->PERIOD_TIME = 600;
                                    curr_state = BACKWARD_PINK;
                                }
                                else
                                {
                                    Walking::GetInstance()->PERIOD_TIME = 800;
                                    //update the x and y co-ordinatimg_targetes of the target
                                    iLastX = img_target.x;
                                    iLastY = img_target.y;

                                    Point2D marker(iLastX,iLastY);
                                    tracker.Process(marker);
                                    m_motion(tracker.ball_position,-9.0);
                                }
                            }
                            else
                            {
                                adjust_gait();
                            }
                        break;
                        case BACKWARD_PINK:
                            VERBOSE("BACKWARD_PINK");

                            Walking::GetInstance()->HIP_PITCH_OFFSET = 10.6;
                            cv::inRange(img_hsv,cv::Scalar(hsv_values[0], hsv_values[2], hsv_values[4]),cv::Scalar(hsv_values[6], hsv_values[8], hsv_values[10]),pink_threshold);
                            //removing false positives
                            // cv::erode(pink_threshold,pink_threshold,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

                            //find target based on two colours. pass two binary images to find_target. returns (x,y,distance btw x&y)
                            img_target = find_target(mat_frame, pink_threshold,0);
                            image_area = img_target.z;
                            if ( image_area != -1 && image_area != 0 )
                            {
                                VERBOSE("BACKWARD_PINK area: "<<image_area);

                                Walking::GetInstance()->PERIOD_TIME = 600;
                                //update the x and y co-ordinatimg_targetes of the target
                                iLastX = img_target.x;
                                iLastY = img_target.y;

                                Point2D marker(iLastX,iLastY);
                                tracker.Process(marker);
                                m_motion(tracker.ball_position,-16.0);
                            }
                            else
                            {
                                adjust_gait();
                            }
                        break;
                    }
                    //live stream results
                    // cv::circle(mat_frame,cv::Point(iLastX,iLastY),3,cv::Scalar(255,0,0));
                    // cv::line(mat_frame,POINT_A,POINT_B,cv::Scalar(0,255,0),1,8);
                    cv::imshow("Live feed", mat_frame);
                    // cv::imshow("Binary Image1",pink_threshold);
                    // cv::imshow("Binary Image2",green_threshold);
                    if( cv::waitKey(30) == 27 ) 
                    {
                        VERBOSE("");
                        break;
                    }
                    // break;
                }//mat_frame.data
            }//while vison
            // VERBOSE("");
            break;
        }//StatusCheck::m_cur_mode == SPRINT
    }//while status check
    return 0;
}
