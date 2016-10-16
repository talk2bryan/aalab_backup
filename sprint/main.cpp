/*
 * main.cpp
 *
 *@author: Bryan Wodi <talk2kamp@gmail.com>
 *@date:  Aug 13, 2016
 *      ADAPTED from robotis tutorial {ball_following}
 *@purpose: sprint FIRA2016. Using OPENCV 2.4.13
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

static int iLowH = 0;
static int iHighH = 179;

static int iLowS = 0; 
static int iHighS = 255;

static int iLowV = 0;
static int iHighV = 255;

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
static const unsigned int num_vertices_square = 4;
static const double max_fb_step = 30.0;
static const double max_tilt_top = 25;
static const double max_tilt_bottom= -12;
static const double max_pan = 65;




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

static void start_running()
{//update (x, y, a) move amplitude
    if( !Walking::GetInstance()->IsRunning() )
    {
        VERBOSE("running...")
        Walking::GetInstance()->PERIOD_TIME = 500;
        // Walking::GetInstance()->X_MOVE_AMPLITUDE += 10.0;
        // Walking::GetInstance()->Y_MOVE_AMPLITUDE = 1;
        // Walking::GetInstance()->Y_OFFSET = 0;
        Walking::GetInstance()->Start();
        usleep(6000);
    }
    
}

static void stop_running()
{
    if( Walking::GetInstance()->IsRunning() )
    {   
        Walking::GetInstance()->Stop();
        VERBOSE("stopped running")
        Walking::GetInstance()->PERIOD_TIME = default_period_time;
        Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;
        Walking::GetInstance()->Y_MOVE_AMPLITUDE = default_y_move_amp;
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

static void move_backwards()
{
    if (! going_backwards)
    {
        stop_running();
        Walking::GetInstance()->X_MOVE_AMPLITUDE=-10;
        going_backwards = true;
        VERBOSETP("new X_MOVE_AMPLITUDE: ",Walking::GetInstance()->X_MOVE_AMPLITUDE);
        start_running();       
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
            // VERBOSE("Head::GetInstance()->InitTracking()")
        }

        // if( (target_not_found_count % 2) == 0)
        // {
        //     VERBOSE("panning left...");
        //     // Walking::GetInstance()->A_MOVE_AMPLITUDE += 50.0;
        // }
        // else
        // {
        //     VERBOSE("panning right...");
        //     // Walking::GetInstance()->A_MOVE_AMPLITUDE -= 50.0;
        // }
    }
    else
    {
        target_not_found_count = 0;
    }
}


static cv::Point3f find_target(cv::Mat& frame)
{
    cv::Point3f target_centre;
    cv::Mat img_canny,rotated;
    static std::vector<std::vector<cv::Point> > contours; 
    static std::vector<cv::Point> approx_poly_points;
    static std::vector<cv::Point> ordered_points;
    static double target_x, target_y, target_area, sum_x, sum_y; //finding the x and y based on averaged vals
    static std::vector<cv::Vec4i> hierarchy;
    
    //canny, then check if it's a rectangle and check if a circle's in it
    cv::Canny(frame,img_canny,canny_threshold,canny_threshold*canny_ratio,canny_kernel_size);
    cv::findContours( img_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);//CV_RETR_EXTERNAL

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

                //now the order of points is: top left, bottom left, bottom right, top right.
                rotated = cv::Mat(transformed_height_width,transformed_height_width,CV_8UC1); //this will contain our roi
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

                //get area, (check if in range), get x and y
                target_area = cv::contourArea(approx_poly_points);

                if (min_target_area < target_area && target_area < max_target_area)
                {//need to add an added layer of verification -- TODO black patch detection
                    sum_y = 0.0;
                    sum_x = 0.0;
                    
                    VERBOSE("Found target");
                    found_target = true;
                    for (int i = 0; i < approx_poly_points.size(); ++i)
                    {
                        sum_x += approx_poly_points.at(i).x;
                        sum_y += approx_poly_points.at(i).y;
                    }
                    target_x = sum_x/approx_poly_points.size();
                    target_y = sum_y/approx_poly_points.size();

                    if(rotated.data)
                    {
                        cv::line(rotated, cv::Point(110,110), cv::Point(120,110), cv::Scalar(255,0,0),1,1);
                        cv::imshow("rotated",rotated);
                    }
                    target_centre = cv::Point3f(target_x, target_y,target_area);

                }//if target
                else
                {
                    target_centre = cv::Point3f(0.0,0.0,0.0);
                }
            }//if rect
        }//for contours
    }//if contours > 0
    else
    {
        target_centre = cv::Point3f(0.0,0.0,0.0);
    }
    return target_centre;
}

static void set_range_params()
{
    std::cout <<"Setting the range for thresholding" <<std::endl;
    std::cout <<"Press the ENTER key when finished!\n"<<std::endl;

    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    //values for inRange thresholding of red colored objects
    cv::namedWindow("Colour Control", CV_WINDOW_AUTOSIZE);
    // cv::namedWindow("Canny Image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Threshold Image", CV_WINDOW_AUTOSIZE);

    //Create trackbars in "Colour Control" window
    cvCreateTrackbar("LowH", "Colour Control", &iLowH, 179);
    cvCreateTrackbar("HighH", "Colour Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Colour Control", &iLowS, 255);
    cvCreateTrackbar("HighS", "Colour Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Colour Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Colour Control", &iHighV, 255);
    
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
            
            cv::inRange(curr_hsv,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),curr_thresholded);
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

    //values for inRange thresholding of red colored objects    
    set_range_params();
    VERBOSETP("iLowH: ",iLowH);
    VERBOSETP("iHighH: ",iHighH);
    VERBOSETP("iLowS: ",iLowS);
    VERBOSETP("iHighS: ",iHighS);
    VERBOSETP("iLowV: ",iLowV);
    VERBOSETP("iHighV: ",iHighV);
    VERBOSE("All set!\n");

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
    Walking::GetInstance()->X_MOVE_AMPLITUDE=10;


    
    //values for reporting the X and Y and area vals for found target
    float iLastX = -1; 
    float iLastY = -1;
    int curr_area;

    cv::namedWindow("Binary Image");
    cv::Mat img_hsv, img_thresholded;// img_canny, rotated;
    
    

    while( true )
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        cv::Mat mat_frame=cv::Mat(rgb_output->m_Height,rgb_output->m_Width,CV_8UC3,rgb_output->m_ImageData);

        if( mat_frame.data )
        {
            cv::cvtColor(mat_frame,mat_frame,cv::COLOR_RGB2BGR);
            cv::cvtColor(mat_frame,img_hsv,cv::COLOR_BGR2HSV);
            
            img_thresholded = cv::Mat(mat_frame.size(),CV_8UC1);
            cv::inRange(img_hsv,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),img_thresholded);
            // cv::GaussianBlur(img_thresholded,img_thresholded,cv::Size(9,9),2,2);

            cv::erode(img_thresholded,img_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
            // cv::dilate(img_thresholded,img_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

            cv::Point3f img_target= find_target(img_thresholded); //rtn x, y and area as z
            curr_area = img_target.z;

            if (curr_area != 0)
            {
                VERBOSETP("Target area: ",curr_area);
                iLastX = img_target.x;
                iLastY = img_target.y;
                Point2D new_ball_pos(iLastX,iLastY);
                // increase_pace();
                tracker.Process(new_ball_pos);
                follower.Process(tracker.ball_position);
                // start_running();
                // usleep(240000);
                // usleep((Walking::GetInstance()->PERIOD_TIME * 5)*1000);

                if (curr_area >= 13000)
                {
                    stop_running();
                    move_backwards();
                }


                // if (curr_area < 7000)
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
            
                // if (curr_area > 7200) //~50cm away, 30cm away is about 10,500 in pixel area
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
            
            }
            else
            {
                //target not found
                VERBOSE("not finding target...");
                stop_running();
                scan_area();
                //adjust_gait();
                // usleep(200);
                // Walking::GetInstance()->PERIOD_TIME = default_period_time;
                // Walking::GetInstance()->X_MOVE_AMPLITUDE = default_x_move_amp;
            }
            cv::imshow("Binary Image",img_thresholded);
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