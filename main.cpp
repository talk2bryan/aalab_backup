/*
 * main.cpp
 *
 *@author: Bryan Wodi <talk2kamp@gmail.com>
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
static int hsv_values[18]; 
static cv::Point POINT_A;
static cv::Point POINT_B;

//for image processing...
static const int canny_threshold = 100;
static const int canny_ratio = 3;
static const int canny_kernel_size = 3;

//flag for indicating direction & yellow patch tracing
static bool going_backwards = false;
static bool tracing_yellow = false;



//returns the angle (in radians) between two vectors
//for degrees: result * 180 / PI
static float calculate_angle_between_points(const cv::Point& pt1, const cv::Point& pt2)
{
    return atan2( pt1.y - pt2.y, pt1.x - pt2.x );
}

static void move_backward() //happens once
{
    if (! going_backwards)
    {
        going_backwards = true;
        tracing_yellow = false;
        VERBOSE("GOING BACK SET");
        Walking::GetInstance()->PERIOD_TIME = 800;
        Walking::GetInstance()->X_MOVE_AMPLITUDE = -9;
        usleep( ((Walking::GetInstance()->PERIOD_TIME * 7) ) * 1000);
    }
}

//param: binary of yellow thrshold
//return: 3-point float containing (x,y,area of patch)
//returns (-1,-1,-1) if target not found
static cv::Point3f trace_yellow_patch(const cv::Mat& frame)
{
    
    float x, y;

    cv::Moments muA = cv::moments(frame, true);
    
    x = muA.m10 / muA.m00;
    y = muA.m01 / muA.m00;

    cv::Point3f result(x,y,muA.m00);
    return result;
}


//return 2D distance between two points
static float get_2D_distance(const cv::Point& pt1, const cv::Point& pt2)
{//based on the Euclidean plane
    float diffX = pt1.x - pt2.x;
    float diffY = pt1.y - pt2.y;
    return sqrt( (diffX * diffX) + (diffY * diffY) );
}

//used by main to find target
//params: two diff binary frames
//return: 3-point float containing (x,y,dist. btw x&y).
//returns (-1,-1,-1) if target not found
static cv::Point3f find_target(const cv::Mat& frame_a,const cv::Mat& frame_b)
{
    cv::Point3f target_centre(-1,-1,-1);
    float x, y;


    cv::Moments muA = cv::moments(frame_a, true);
    cv::Moments muB = cv::moments(frame_b, true);

    cv::Point center_A, center_B;
    
    center_A.x = muA.m10 / muA.m00;
    center_A.y = muA.m01 / muA.m00;

    center_B.x = muB.m10 / muB.m00;
    center_B.y = muB.m01 / muB.m00;

    POINT_A = center_A;
    POINT_B = center_B;
    float angle = calculate_angle_between_points(center_A,center_B);


    VERBOSE("angle: " << angle );

    x = (center_A.x+center_B.x)/2;
    y = (center_A.y+center_B.y)/2;

    float curr_distance = get_2D_distance(center_A,center_B);

    //filter to ensure the right objects are being picked up based on caliberated distances
    //between the images, looking at target from start line and at finish line.
    if ( (curr_distance >= 15 && curr_distance <= 150)
        &&  (-0.6 <= angle && angle <= 0.06)/*&& (abs(shortest_path - curr_distance) <=20)*/ )//calibrated value
    {
        target_centre = cv::Point3f(x,y, curr_distance);
    }
    
    return target_centre;
}


//sets the range values for the HSV
//based on openCV documentation
static void initialize_hsv_array()
{
    for(int i = 0; i<18;i++)
    {
        if(i <= 8)
            hsv_values[i] = 0;
        else if(i >8 && i <12)
            hsv_values[i] = 179;
        else
            hsv_values[i] = 255;
    }

    //default vals for thresholding
    //assuming you set the thresh vals in the order: yellow, pink
    //before green... 

    //yellow patch
    hsv_values[3] = 116;//lowS
    hsv_values[12] = 227; //highS
    hsv_values[9] = 34; //highH
    hsv_values[6] = 155; //lowV

    //pink paper
    hsv_values[1] = 132; //lowH
    // hsv_values[2] = 100;
    // //green paper
    hsv_values[2] = 26; //lowH
    hsv_values[5] = 200; //lowS
    hsv_values[11] = 57; //highH
}

//before the race starts, you want to calibrate the vals for 
//all 3 colors. this receives d_x which is an offset (0, 1 oo 2)
//that sets the thresh vals for the corresponding color
//0 -- means, set HSV vals for 1st color
//1 -- set HSV vals for the 2nd color
//2 -- set HSV vals for the 3rd color

//i assume yellow is set 1st as the head is tilted only for the first time
//in the main, just call this method 3ice with 0, 1 and 2
//to set the thresh vals for all 3 colors
static void set_range_params(int d_x)
{
    VERBOSE("Setting the range for thresholding");
    VERBOSE("Press the ENTER key when finished!\n");


    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    
    //values for inRange thresholding of red colored objects
    cv::namedWindow("Colour Control", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Threshold Image", CV_WINDOW_AUTOSIZE);


    cvCreateTrackbar("LowH", "Colour Control", &hsv_values[d_x], 179);
    cvCreateTrackbar("HighH", "Colour Control", &hsv_values[d_x+9], 179);

    cvCreateTrackbar("LowS", "Colour Control", &hsv_values[d_x+3], 255);
    cvCreateTrackbar("HighS", "Colour Control", &hsv_values[d_x+12], 255);

    cvCreateTrackbar("LowV", "Colour Control", &hsv_values[d_x+6], 255);
    cvCreateTrackbar("HighV", "Colour Control", &hsv_values[d_x+15], 255);

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
            
            cv::inRange(curr_hsv,cv::Scalar(hsv_values[d_x],hsv_values[d_x+3],hsv_values[d_x+6]),cv::Scalar(hsv_values[d_x+9],hsv_values[d_x+12],hsv_values[d_x+15]),curr_thresholded);
            cv::erode(curr_thresholded,curr_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
            cv::Canny(curr_thresholded,curr_canny,canny_threshold,canny_threshold*canny_ratio,canny_kernel_size);
        }
        //report area just for calibrating yellow
        if ( d_x == 0)
        {
            cv::Moments muA = cv::moments(curr_thresholded, true);
            VERBOSE(muA.m00);
        }
        
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
    LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    VERBOSE("hit ENTER to begin: ")
    getchar();

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    MotionManager::GetInstance()->SetEnable(true);
    Walking::GetInstance()->STEP_FB_RATIO = 1.0;
    
    

    initialize_hsv_array(); //set range for thresholding 
    
    //now calibrate the 3 colors. see method def
    Head::GetInstance()->MoveByAngle(0,0); //tilt head to thresh yellow
    set_range_params(0);

    Head::GetInstance()->MoveByAngle(0,50);//look back up
    set_range_params(1);//pink
    set_range_params(2);//green  -- the order of these two donot matter


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
    //note: img_thresholded1 and 2 will be binary images
    cv::Mat img_hsv, img_thresholded1, img_thresholded2;
    cv::Mat yellow_threshold;

    StatusCheck::Check(cm730);
    StatusCheck::m_cur_mode = SPRINT;

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
                {//convert to0 BGR before converting to HSV
                    cv::cvtColor(mat_frame,mat_frame,cv::COLOR_RGB2BGR);
                    cv::cvtColor(mat_frame,img_hsv,cv::COLOR_BGR2HSV);
                    
                    //initializing threshold mat images as single channel images
                    img_thresholded1 = cv::Mat(mat_frame.size(),CV_8UC1);
                    img_thresholded2 = cv::Mat(mat_frame.size(),CV_8UC1);
                    yellow_threshold = cv::Mat(mat_frame.size(),CV_8UC1);

                    cv::Point3f img_target; //target contains x, y and some other param
                    int curr_dist = 0; //distance between two images
                    float yellow_area = 0.0;//area of yellow patch


                    if ( !tracing_yellow ) //still far from target 
                    {
                        yellow_area = 0.0;
                        //applying respective thresholding based on calibrated vals from set_range_params()
                        cv::inRange(img_hsv,cv::Scalar(hsv_values[1], hsv_values[4], hsv_values[7]),cv::Scalar(hsv_values[10], hsv_values[13], hsv_values[16]),img_thresholded1);
                        cv::inRange(img_hsv,cv::Scalar(hsv_values[2], hsv_values[5], hsv_values[8]),cv::Scalar(hsv_values[11], hsv_values[14], hsv_values[17]),img_thresholded2);
                        //removing false positives
                        cv::erode(img_thresholded1,img_thresholded1,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
                        cv::erode(img_thresholded2,img_thresholded2,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

                        //find target based on two colours. pass two binary images to find_target. returns (x,y,distance btw x&y)
                        img_target = find_target(img_thresholded1, img_thresholded2); 
                        curr_dist = img_target.z;
                        VERBOSE("rel dist: "<<curr_dist);
                    }
                    else
                    {//close to target and tracing yellow patch
                        curr_dist = 0;
                        //slow down...
                        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
                        Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
                        Walking::GetInstance()->PERIOD_TIME = 850;

                        cv::inRange(img_hsv,cv::Scalar(hsv_values[0], hsv_values[3], hsv_values[6]),cv::Scalar(hsv_values[9], hsv_values[12], hsv_values[15]),yellow_threshold);
                        cv::erode(yellow_threshold,yellow_threshold,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
                        
                        //target is yellow patch. contains (x,y,patch area)
                        img_target = trace_yellow_patch(yellow_threshold);
                        yellow_area = img_target.z;
                        VERBOSE("yellow_area: "<<yellow_area);
                    }
                    
                    if ( img_target.z != -1 )
                    {//if we find our target...
                        //update the x and y co-ordinates of the target
                        iLastX = img_target.x;
                        iLastY = img_target.y;

                        Point2D marker(iLastX,iLastY);
                        tracker.Process(marker);
                        if(!going_backwards)
                            follower.Process(tracker.ball_position);


                        if (curr_dist >90 && !going_backwards)
                        {//approaching target
                            tracing_yellow = true;
                        }

                        if (yellow_area > 2170 && !going_backwards)
                        {
                            move_backward();
                            Walking::GetInstance()->PERIOD_TIME = 700;
                            Head::GetInstance()->MoveToHome();
                            Head::GetInstance()->InitTracking();
                        }
                        if (going_backwards)
                        {
                            // Walking::GetInstance()->X_MOVE_AMPLITUDE = -9;
                            Walking::GetInstance()->HIP_PITCH_OFFSET = 24.5;
                        }


                        //display target on livestream -- for debugging
                        #ifdef DEBUG
                        cv::circle(mat_frame,cv::Point(iLastX,iLastY),3,cv::Scalar(255,0,0));
                        cv::line(mat_frame,POINT_A,POINT_B,cv::Scalar(0,255,0),1,8);
                        #endif
                    }
                    else
                    {
                        Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
                        Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
                    }

                    //live stream results
                    cv::imshow("Live feed", mat_frame);
                    // cv::imshow("Binary Image1",img_thresholded1);
                    // cv::imshow("Binary Image2",img_thresholded2);
                    if( cv::waitKey(30) == 27 ) 
                    {
                        VERBOSE("");
                        break;
                    }
                    // break;
                }//mat_frame.data
            }//while vison
            VERBOSE("");
            break;
        }//StatusCheck::m_cur_mode == SPRINT
    }//while status check

    return 0;
}