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

#define INI_FILE_PATH       "../../../Data/config.ini"

#define U2D_DEV_NAME        "/dev/ttyUSB0"



static bool found_target = false;
static bool going_backwards = false;

static int target_not_found_count = 0;
static int m_fb_step;

static double m_head_tilt;
static double m_head_pan;

static double default_period_time;
static double default_a_move_amp;
static double default_x_move_amp;
static double default_y_move_amp;
static double default_pelvis_offset;

static const int canny_threshold = 100;
static const int canny_ratio = 3;
static const int canny_kernel_size = 3;
static const int transformed_height_width = 200;
static const int min_target_area = 100;
static const int max_target_area = 33000; //these values are based on sampling the target area
static const int max_target_not_found_count = 100;
static const int max_target_found_count = 2;
static const int past_finish_line_dist = 110;


static const unsigned int num_vertices_square = 4;
static const double max_fb_step = 30.0;
static const double max_tilt_top = 25;
static const double max_tilt_bottom= -12;
static const double max_pan = 65;

static int hsv_values[12];

static cv::Point POINT_A;
static cv::Point POINT_B;





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


//use step irection from tuner to pan head
static void pan_left()
{
    VERBOSE("pan left...")
    Head::GetInstance()->MoveByAngle(30,30);
    Head::GetInstance()->MoveByAngle(0,30);
}

static void pan_right()
{
    VERBOSE("pan right...")
    Head::GetInstance()->MoveByAngle(30,30);
    Head::GetInstance()->MoveByAngle(60,30);
}
static void start_running()
{//update (x, y, a) move amplitude
    if( !Walking::GetInstance()->IsRunning() )
    {
        VERBOSE("running...")
        Walking::GetInstance()->PERIOD_TIME = 600;
        // Walking::GetInstance()->X_MOVE_AMPLITUDE += 10.0;
        // Walking::GetInstance()->Y_MOVE_AMPLITUDE = 1;
        // Walking::GetInstance()->Y_OFFSET = 0;
        Walking::GetInstance()->Start();
    }
    
}

static void restore_params()
{
    Walking::GetInstance()->A_MOVE_AMPLITUDE = default_a_move_amp;
    Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;
    Walking::GetInstance()->Y_MOVE_AMPLITUDE = default_y_move_amp;
    Walking::GetInstance()->PERIOD_TIME = default_period_time;
    Walking::GetInstance()->PELVIS_OFFSET = default_pelvis_offset;
    VERBOSE("restored params")
}

void increase_pace()
{
    // Walking::GetInstance()->PERIOD_TIME = 300;
    Walking::GetInstance()->X_MOVE_AMPLITUDE += 10.0;
}

void decrease_pace()
{
    Walking::GetInstance()->PERIOD_TIME = default_period_time;
    Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;
}

void stop_running()
{
    if( Walking::GetInstance()->IsRunning() )
    {   
        VERBOSE("stopped running")
        Walking::GetInstance()->PERIOD_TIME = default_period_time;
        Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;
        Walking::GetInstance()->Y_MOVE_AMPLITUDE = default_y_move_amp;
        Walking::GetInstance()->Stop();
    }
}
static float get_2D_distance(const cv::Point& pt1, const cv::Point& pt2)
{//based on the Euclidean plane
    float diffX = pt1.x - pt2.x;
    float diffY = pt1.y - pt2.y;
    return sqrt( (diffX * diffX) + (diffY * diffY) );
}

static void adjust_gait()
{

    // if((found_target == false) && target_not_found_count > 3)
    // {
    //     VERBOSE("adjust gait...");
    //     // if( (target_not_found_count % 2) == 0)
    //     // {
    //     //     Head::GetInstance()->MoveToHome();
    //     //     Head::GetInstance()->MoveByAngle(-30,30);
    //     //     VERBOSE("pan left");
    //     // }
    //     // else
    //     // {
    //     //     Head::GetInstance()->MoveToHome();
    //     //     Head::GetInstance()->MoveByAngle(30,30);
    //     //     VERBOSE("pan right");
    //     // }
    // }
    VERBOSE("adjusting gait...");
    Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0; //direction
    Walking::GetInstance()->X_MOVE_AMPLITUDE = 0.0; //forward/backward
    Walking::GetInstance()->PERIOD_TIME = 600;
    Walking::GetInstance()->Start();
    usleep(200000);
}

static void move_backward()
{
    if (! going_backwards)
    {
        stop_running();
        Walking::GetInstance()->X_MOVE_AMPLITUDE=-10;
        going_backwards = true;
        VERBOSETP("new X_MOVE_AMPLITUDE: ",Walking::GetInstance()->X_MOVE_AMPLITUDE);
        start_running();
        usleep(8*1000);        
    }
     
}

static void scan_area()
{
    if(! found_target)
    {
        
        Walking::GetInstance()->A_MOVE_AMPLITUDE = 0.0; //look straght
        Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;

        if (target_not_found_count < max_target_not_found_count)
        {
            Head::GetInstance()->MoveTracking();
            VERBOSE("Head::GetInstance()->MoveTracking()")
            target_not_found_count++;
        }
        else
        {
            Head::GetInstance()->InitTracking();
        }
    }
    else
    {
        target_not_found_count = 0;
    }
}

static std::vector<cv::Point> get_points_in_clockwise_order(const cv::Mat& frame)
{
    cv::Mat img_canny;
    static std::vector<std::vector<cv::Point> > contours; 
    static std::vector<cv::Point> approx_poly_points;
    static std::vector<cv::Point> ordered_points;
    static std::vector<cv::Vec4i> hierarchy;

    // ordered_points[0] = cv::Point(-1,-1);

    //canny, then check if it's a rectangle and check if a circle's in it
    cv::Canny(frame,img_canny,canny_threshold,canny_threshold*canny_ratio,canny_kernel_size);
    cv::findContours( img_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    if (contours.size() > 0)
    {
        for (size_t i = 0; i < contours.size(); ++i)
        {
            // approximate contour with accuracy proportional
            // to the contour perimeter
            cv::approxPolyDP( cv::Mat(contours[i]), approx_poly_points, cv::arcLength(cv::Mat(contours[i]), true)*0.05 , true);
            if ( (approx_poly_points.size() == num_vertices_square) 
                && (fabs(cv::contourArea(cv::Mat(approx_poly_points))) > min_target_area)
                && (cv::isContourConvex(approx_poly_points)) 
                )
            {   //points are either 
                /*
                a   d           OR     b   a
                b   c //normal         c   d //abnormal
                */
                //if the slope between a and c is -ve, then we have the abnormal case
                ordered_points = approx_poly_points;
                if(approx_poly_points[0].x > approx_poly_points[2].x) //positive slope
                {//shift all
                    ordered_points[0] = approx_poly_points[1];
                    ordered_points[1] = approx_poly_points[2];
                    ordered_points[2] = approx_poly_points[3];
                    ordered_points[3] = approx_poly_points[0];
                }

                #ifdef SHOWROI
                cv::Mat rotated = cv::Mat(transformed_height_width,transformed_height_width,CV_8UC1); //this will contain our roi
                cv::Point2f dst_vertices[4]; 
                //in the order:
                //top left, bottom left, bottom right, top right
                dst_vertices[0] = cv::Point(0,0);
                dst_vertices[1] = cv::Point(0,transformed_height_width-1);
                dst_vertices[2] = cv::Point(transformed_height_width-1,transformed_height_width-1);
                dst_vertices[3] = cv::Point(transformed_height_width-1,0);

                cv::Point2f src_vertices[4];
                src_vertices[0] = ordered_points[0];
                src_vertices[1] = ordered_points[1];
                src_vertices[2] = ordered_points[2];
                src_vertices[3] = ordered_points[3];
                
                cv::Mat warpAffineMatrix = cv::getPerspectiveTransform(src_vertices,dst_vertices);

                cv::Size warp_size(transformed_height_width,transformed_height_width);
                cv::warpPerspective(frame,rotated,warpAffineMatrix,warp_size,cv::INTER_LINEAR,cv::BORDER_CONSTANT);

                if(rotated.data)
                {
                    cv::imshow("rotated",rotated);
                }
                #endif
            }
        }
    }
    return ordered_points;
}

static cv::Point get_centre_of_frame(const cv::Mat& frame)
{
    std::vector<cv::Point> frame_points;
    double sum_x, sum_y; //finding the x and y based on averaged vals
    cv::Point result(-1,-1);

    frame_points = get_points_in_clockwise_order(frame);

    if ( frame_points[0].x != 0 && frame_points[0].y != 0 ) //------------------causes seg fault 30% of the time-----//
    {
        sum_y = 0.0;
        sum_x = 0.0;
        
        for (int i = 0; i < num_vertices_square; ++i)
        {
            sum_x += frame_points.at(i).x;
            sum_y += frame_points.at(i).y;
        }
        return cv::Point( (sum_x/num_vertices_square), (sum_y/num_vertices_square) );
    }
    return result;
}

static cv::Point3f get_relative_distance_between_frame_coordinates(const cv::Mat& frame_a, const cv::Mat& frame_b)
{
    float x, y;
    cv::Point3f result(-1,-1,-1);

    // cv::Point a( get_centre_of_frame(frame_a) );
    // cv::Point b( get_centre_of_frame(frame_b) );


    if ( true )//(a.x != -1 && a.y != -1) && (b.x != -1 && b.y != -1) )
    {
        cv::Moments muA = cv::moments(frame_a, true);
        cv::Moments muB = cv::moments(frame_b, true);

        cv::Point center_A, center_B;
        
        center_A.x = muA.m10 / muA.m00;
        center_A.y = muA.m01 / muA.m00;

        center_B.x = muB.m10 / muB.m00;
        center_B.y = muB.m01 / muB.m00;

        POINT_A = center_A;
        POINT_B = center_B;

        x = (center_A.x+center_B.x)/2;
        y = (center_A.y+center_B.y)/2;

        printf("A: {%d, %d}\t", POINT_A.x,POINT_A.y);
        printf("B: {%d, %d}\n", POINT_B.x,POINT_B.y);

        result = cv::Point3f(x,y, get_2D_distance(POINT_A,POINT_B));
    }

    return result;
}

static cv::Point3f find_target(cv::Mat& frame_a, cv::Mat& frame_b)
{
    cv::Point3f target_centre(-1,-1,-1);
    cv::Mat img_canny,rotated;

    static std::vector<std::vector<cv::Point> > contours; 
    static std::vector<cv::Point> approx_poly_points;
    static std::vector<cv::Point> ordered_points;
    float shortest_path = 100000.0;

  
    cv::Point3f point_and_dist = get_relative_distance_between_frame_coordinates(frame_a,frame_b);

    if (point_and_dist.x != -1 && point_and_dist.y != -1 && point_and_dist.z != -1 )
    {
        float running_distance = point_and_dist.z;
        printf("running_distance: %f\n", running_distance);

        if ( running_distance >= 15 && running_distance <= 150) //calibrated value
        {
            target_centre = cv::Point3f(point_and_dist.x,point_and_dist.y, point_and_dist.z);//dummy - remove //(global_frame_a_array[i].size() * global_frame_b_array[i].size()));
            // printf("TARGET: {%d, %d}\n", point_and_dist.x,point_and_dist.y);
        }
    }
    return target_centre;
}

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
}

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


/*find  cosine of angle between two vectors from pt0->pt1 and from pt0->pt2 */
// static double find_cosine( const cv::Point pt1, const cv::Point pt2, const cv::Point pt0 )
// {
//     double dx1 = pt1.x - pt0.x;
//     double dy1 = pt1.y - pt0.y;
//     double dx2 = pt2.x - pt0.x;
//     double dy2 = pt2.y - pt0.y;
//     return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
// }

// static double determine_angle(const cv::Point& point)
// {
//     return ( -atan2(point.x,-point.y) );
// }

// static bool compare_points(const cv::Point& a, const cv::Point& b)
// {
//     return ( (a.x) < (b.x) );
//     // return ( determine_angle(a) < determine_angle(b) );
// }

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

    // mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;

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
    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    
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

    //values for inRange thresholding    
    initialize_hsv_array();
    set_range_params(0);
    set_range_params(1);

    #ifdef DEBUG
    VERBOSETP("iLowH1: ",hsv_values[0]);
    VERBOSETP("iHighH1: ",hsv_values[6]);
    VERBOSETP("iLowS1: ",hsv_values[2]);
    VERBOSETP("iHighS1: ",hsv_values[8]);
    VERBOSETP("iLowV1: ",hsv_values[4]);
    VERBOSETP("iHighV1: ",hsv_values[10]);
    printf("\n");
    VERBOSETP("iLowH2: ",hsv_values[1]);
    VERBOSETP("iHighH2: ",hsv_values[7]);
    VERBOSETP("iLowS2: ",hsv_values[3]);
    VERBOSETP("iHighS2: ",hsv_values[9]);
    VERBOSETP("iLowV2: ",hsv_values[5]);
    VERBOSETP("iHighV2: ",hsv_values[11]);
    #endif
    
    printf("All set!\n");
    printf("Press the ENTER key to begin!\n");
    getchar();
   
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    MotionManager::GetInstance()->SetEnable(true);
    Walking::GetInstance()->STEP_FB_RATIO = 1.0;

    default_period_time = Walking::GetInstance()->PERIOD_TIME;
    default_a_move_amp =  Walking::GetInstance()->A_MOVE_AMPLITUDE;
    default_x_move_amp =  Walking::GetInstance()->X_MOVE_AMPLITUDE;
    default_y_move_amp =  Walking::GetInstance()->Y_MOVE_AMPLITUDE;
    default_pelvis_offset = Walking::GetInstance()->PELVIS_OFFSET;

    Head::GetInstance()->MoveByAngle(0,40); //keep head focused on target
    
    //values for reporting the X and Y and area vals for found target
    float iLastX = -1; 
    float iLastY = -1;
    int curr_dist;

    cv::Mat img_hsv, img_thresholded1, img_thresholded2;
    
    //new logic here, run for 60 steps and run back -- may be cheaper

    while( true )
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        cv::Mat mat_frame=cv::Mat(rgb_output->m_Height,rgb_output->m_Width,CV_8UC3,rgb_output->m_ImageData);

        if( mat_frame.data )
        {
            cv::cvtColor(mat_frame,mat_frame,cv::COLOR_RGB2BGR);
            cv::cvtColor(mat_frame,img_hsv,cv::COLOR_BGR2HSV);
            
            img_thresholded1 = cv::Mat(mat_frame.size(),CV_8UC1);
            img_thresholded2 = cv::Mat(mat_frame.size(),CV_8UC1);
            
            cv::inRange(img_hsv,cv::Scalar(hsv_values[0], hsv_values[2], hsv_values[4]),cv::Scalar(hsv_values[6], hsv_values[8], hsv_values[10]),img_thresholded1);
            cv::inRange(img_hsv,cv::Scalar(hsv_values[1], hsv_values[3], hsv_values[5]),cv::Scalar(hsv_values[7], hsv_values[9], hsv_values[11]),img_thresholded2);

            // cv::GaussianBlur(img_thresholded,img_thresholded,cv::Size(9,9),2,2);

            cv::erode(img_thresholded1,img_thresholded1,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
            cv::erode(img_thresholded2,img_thresholded2,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

            // cv::dilate(img_thresholded,img_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

            cv::Point3f img_target= find_target(img_thresholded1, img_thresholded2);

            curr_dist = img_target.z;

            if (curr_dist != -1)
            {
                VERBOSETP("Targets' rel. distance: ",curr_dist);
                iLastX = img_target.x;
                iLastY = img_target.y;
                Point2D new_ball_pos(iLastX,iLastY);

                #ifdef BLIND_SPRINT
                tracker.Process(new_ball_pos);
                start_running();
                usleep((Robot::Walking::GetInstance()->PERIOD_TIME * 10)*1000); //walk 10 steps
                #else
                // increase_pace();
                tracker.Process(new_ball_pos);
                follower.Process(tracker.ball_position);
                // start_running();
                // usleep(240000);
                // usleep((Walking::GetInstance()->PERIOD_TIME * 5)*1000);

                if (curr_dist >= past_finish_line_dist)
                {
                    stop_running();
                    // move_backward();
                }


                // if (curr_dist < 7000)
                // {
                    
                //     tracker.Process(new_ball_pos);
                    // follower.Process(tracker.ball_position);
                //     //start_running();
                //     // usleep(2400000);
                // }
                // else
                // {
                //     stop_running();
                // }
            
                // if (curr_dist > 7200) //~50cm away, 30cm away is about 10,500 in pixel area
                // {//closing in on object so tilt head downwards to focus
                //     Point2D new_ball_pos(iLastX,iLastY);
                //     Head::GetInstance()->MoveByAngleOffset(0,-1);
                //     // Walking::GetInstance()->X_MOVE_AMPLITUDE = 1.0;
                //     tracker.Process(new_ball_pos);
                //     follower.Process(tracker.ball_position);
                //     // start_running();
                //     usleep(240);
                //     // stop_running();
                //     // usleep((Robot::Walking::GetInstance()->PERIOD_TIME * 2)*1000);
                //     // follower.Process(tracker.ball_position);
                //     Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;
                //     Walking::GetInstance()->PERIOD_TIME = default_period_time;
                //     // usleep(250); 
                // }
                // else
                // {
                //     //TODO - tweak period time to increase speed -----
                //     // get centre of the target, x marks the spot
                //     Point2D new_ball_pos(iLastX,iLastY);
                //     //walk straight because target is far away
                //     // Head::GetInstance()->MoveByAngle(0,30); //look straight
                //     // Walking::GetInstance()->X_MOVE_AMPLITUDE = 1.0;// Walking::GetInstance()->STEP_FB_RATIO = 1.0
                //     tracker.Process(new_ball_pos);
                //     // follower.Process(tracker.ball_position);
                //     // Walking::GetInstance()->PERIOD_TIME = 300;
                //     // Walking::GetInstance()->X_MOVE_AMPLITUDE = 10.0;
                //     follower.Process(tracker.ball_position);
                //     // start_running();
                //     usleep(240);
                //     Walking::GetInstance()->PERIOD_TIME = default_period_time;
                //     // stop_running();
                //     // usleep(250); 
                // }
                #endif
            
            }
            else
            {
                //target not found
                VERBOSE("not finding target...");
                stop_running();
                // scan_area();
                //adjust_gait();
                // usleep(200);
                // Walking::GetInstance()->PERIOD_TIME = default_period_time;
                // Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;
            }
            cv::imshow("Binary Image1",img_thresholded1);
            cv::imshow("Binary Image2",img_thresholded2);

            #ifdef DEBUG
            cv::circle(mat_frame,cv::Point(iLastX,iLastY),3,cv::Scalar(255,0,0));
            cv::line(mat_frame,POINT_A,POINT_B,cv::Scalar(0,255,0),1,8);
            #endif
            cv::imshow("Live feed", mat_frame);
            if(cv::waitKey(30) == 27) 
            {
                stop_running();
                restore_params();
                break;
            }
        } // if mat_frame.data
    } //outermost while
    return 0;
}
