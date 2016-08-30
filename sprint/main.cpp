/*
 * main.cpp
 *
 *@author: Bryan Wodi <talk2kamp@gmail.com>
 *@date:  Aug 13, 2016
 *      ADAPTED from robotis tutorial {ball_following}
 *@purpose: sprint towards an object
 */
#include "CompilerDefinitions.hpp"

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


static const int canny_threshold = 100;
static const int canny_ratio = 3;
static const int canny_kernel_size = 3;
static const int num_vertices_square = 4;
static const int transformed_height_width = 200;
static const int min_poly_contour_area = 1000; //this is based on sampling the area of the proposed target


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

static void set_range_params()
{
    std::cout <<"Setting the range for thresholding" <<std::endl;
    std::cout <<"Press the ENTER key when finished!\n"<<std::endl;

    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    //values for inRange thresholding of red colored objects
    cv::namedWindow("Colour Control", CV_WINDOW_AUTOSIZE);

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
        cv::Mat curr_hsv, curr_thresholded;
        
        if( curr_frame.data )
        {        
            //first convert cam image to bgr before hsv
            cv::cvtColor(curr_frame,curr_frame,cv::COLOR_RGB2BGR);
            cv::cvtColor(curr_frame,curr_hsv,cv::COLOR_BGR2HSV);
            
            cv::inRange(curr_hsv,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),curr_thresholded);
            cv::GaussianBlur(curr_thresholded,curr_thresholded,cv::Size(5,5),0);
        }
        
        cv::imshow("Thresholded Image",curr_thresholded);
        cv::imshow("Original Image",curr_frame);
        
        if(cv::waitKey(30) == 10) 
        {
            cv::destroyAllWindows();
            break;
        }
    }
    
}

/*find  cosine of angle between two vectors from pt0->pt1 and from pt0->pt2 */
static double find_cosine( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static double determine_angle(const cv::Point& point)
{
    return ( -atan2(point.x,-point.y) );
}

static bool compare_points(const cv::Point& a, const cv::Point& b)
{
    return ( (a.x) < (b.x) );
    // return ( determine_angle(a) < determine_angle(b) );
}

/* check image to ensure it contains rectangles contains a circle -> target*/
// static bool isImageRectangle(cv::Mat& image)
// {
//     squares.clear();
//     contours.clear();

//     std::vector<cv::Point> approx_polys;
//     bool result = false;

//     cv::findContours( image, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) ); //maybe take out Point() and leave default?
//     if(contours.size() > 0)
//     {
//         for (size_t i = 0; i < contours.size(); ++i)
//         {
//             // approximate contour with accuracy proportional
//             // to the contour perimeter
//             cv::approxPolyDP( cv::Mat(contours[i]), approx_polys, cv::arcLength(cv::Mat(contours[i]), true)*0.02 , true);

//             if ( (approx_polys.size() == num_vertices_square) 
//                 && (fabs(cv::contourArea(cv::Mat(approx_polys))) > min_poly_contour_area)
//                 && (cv::isContourConvex(cv::Mat(approx_polys))) 
//                 )
//             { 
//                 double maxCosine = 0;

//                 for( int j = 2; j < num_vertices_square+1; j++ )
//                 {
//                     // find the maximum cosine of the angle between joint edges
//                     double cosine = fabs(find_cosine(approx_polys[j%num_vertices_square], approx_polys[j-2], approx_polys[j-1]));
//                     maxCosine = MAX(maxCosine, cosine);
//                 }

//                 if( maxCosine < 0.3 )
//                 {
//                     squares.push_back(approx_polys);
//                     VERBOSETP("Number of rectanges: ",squares.size());
//                     result = true;
//                 }
//             }
//         }
//     }
//     return result;
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

    Head::GetInstance()->MoveByAngle(0,30); //keep head focused on target

    //values for reporting the X and Y vals for found circle
    int iLastX = -1; 
    int iLastY = -1;
    cv::Mat img_hsv, img_thresholded, img_canny, img_ROI, new_frame, rotated,blurred_frame;
    

    while( true )
    {
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
        cv::Mat mat_frame=cv::Mat(rgb_output->m_Height,rgb_output->m_Width,CV_8UC3,rgb_output->m_ImageData);

        if( mat_frame.data )
        {
        	new_frame = cv::Mat::zeros(mat_frame.size(),CV_8UC3);
            img_thresholded = cv::Mat(mat_frame.size(),CV_8U);
            static std::vector<std::vector<cv::Point> > contours; 
            // static std::vector<std::vector<cv::Point> > squares;
            static std::vector<cv::Point> approx_polys;
            static std::vector<cv::Point> ordered_polys;
            //first convert cam image to BGR to properly correctly convert to HSV
            cv::cvtColor(mat_frame,mat_frame,cv::COLOR_RGB2BGR);
            cv::cvtColor(mat_frame,img_hsv,cv::COLOR_BGR2HSV);
            
            cv::inRange(img_hsv,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),img_thresholded);
            cv::GaussianBlur(img_thresholded,img_thresholded,cv::Size(5,5),0);

            // cv::erode(img_thresholded,img_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));
            // cv::dilate(img_thresholded,img_thresholded,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,4)));

            //canny, then check if it's a rectangle and check if a circle's in it
            cv::Canny(img_thresholded,img_canny,canny_threshold,canny_threshold*canny_ratio,canny_kernel_size);

            cv::findContours( img_canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            
            // img_ROI = mat_frame(cv::Rect(100,100,mat_frame.cols,mat_frame.rows));
            if (contours.size() > 0)
            {
            	
                for (size_t i = 0; i < contours.size(); ++i)
                {                    
                    // approximate contour with accuracy proportional
                    // to the contour perimeter
                    cv::approxPolyDP( cv::Mat(contours[i]), approx_polys, cv::arcLength(cv::Mat(contours[i]), true)*0.02 , true);
                    //points of the polygon are saved in approx_polys in 
                    //the order: top right, bottom right, bottom left,top left
                    if ( (approx_polys.size() == num_vertices_square) 
                        //&& (fabs(cv::contourArea(cv::Mat(approx_polys))) > min_poly_contour_area)
                        // && (cv::isContourConvex(approx_polys)) 
                        )
                    {
                        VERBOSETP("Area: ", cv::contourArea(approx_polys));
                        //std::sort(approx_polys.begin(), approx_polys.end(),compare_points);
                        // ordered_polys = approx_polys;
                        //points are either 
                        /*
                        a   d   
                        b   c
                          OR
                        b   a
                        c   d
                        */
                        if (approx_polys[1].y < approx_polys[3].y && approx_polys[0].y < approx_polys[2].y)
                        {//else we have TR, TL, BL, BR (bacd)
                            ordered_polys[0] = approx_polys[1];
                            ordered_polys[1] = approx_polys[2];
                            ordered_polys[2] = approx_polys[3];
                            ordered_polys[3] = approx_polys[0];
                        }
                        else
                        {//normal - TL, BL,BR, TR (abcd)
                            ordered_polys = approx_polys;
                        }

                        //now the order of points is: top left, top right, bottom right, bottom left.
                        rotated = cv::Mat(transformed_height_width,transformed_height_width,CV_8U); //this will contain our roi
                    	cv::Point2f dst_vertices[4]; 
                    	//in the order:
                        //top left, bottom left, bottom right, top right
                    	dst_vertices[0] = cv::Point(0,0);
                    	dst_vertices[1] = cv::Point(0,transformed_height_width-1);
                    	dst_vertices[2] = cv::Point(transformed_height_width-1,transformed_height_width-1);
                    	dst_vertices[3] = cv::Point(transformed_height_width-1,0);

                        cv::Point2f src_vertices[4];
                        src_vertices[0] = ordered_polys[0];
                        src_vertices[1] = ordered_polys[1];
                        src_vertices[2] = ordered_polys[2];
                        src_vertices[3] = ordered_polys[3];
                    	
                    	cv::Mat warpAffineMatrix = cv::getPerspectiveTransform(src_vertices,dst_vertices);

                    	cv::Size warp_size(transformed_height_width,transformed_height_width);
                    	cv::warpPerspective(img_thresholded,rotated,warpAffineMatrix,warp_size,cv::INTER_LINEAR,cv::BORDER_CONSTANT);

                        //get area, (check if in range), get x and y
                        // /// Get the moments
                        // cv::vector<cv::Moments> mu(contours.size() );
                        // for( int i = 0; i < contours.size(); i++ )
                        // {
                        //     mu[i] = moments( contours[i], false );
                        // }

                        // ///  Get the mass centers:
                        // cv::vector<cv::Point2f> mc( contours.size() );
                        // for( int i = 0; i < contours.size(); i++ )
                        // {
                        //     mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
                        // }
                        //VERBOSETP("Area of ROI:",mu[i].m00);


                    }
                }
            }
            // if( isImageRectangle(img_canny) )
            // {
            //     VERBOSE("seen a rectangle");	
            //     //Calculate the moments of the thresholded image
            //     cv::Moments oMoments = cv::moments(img_canny);

            //     double dM01 = oMoments.m01;
            //     double dM10 = oMoments.m10;
            //     double dArea = oMoments.m00;

            //     VERBOSETP("Area: ", dArea);

            //     //calculate the position of the target
            //     int posX = (int)(dM10 / dArea);
            //     int posY = (int)(dM01 / dArea);
            //     iLastX = posX;
            //     iLastY = posY;

            //     Point2D new_ball_pos(iLastX,iLastY);
            //     tracker.Process(new_ball_pos);
            //     //follower.Process(tracker.ball_position);
            	// cv::Mat img(new_frame.rows,new_frame.cols,CV_8UC1);
            	// cv::polylines(img,approx_polys,true,cv::Scalar(255));
                cv::imshow("Thresholded Image",img_thresholded);
                cv::imshow("Canny Image",img_canny);
                cv::imshow("Rotated",rotated);
                // cv::imshow("ROI",img_ROI);
                if(cv::waitKey(30) == 27) break;

            // }
        } //outer if
       // usleep(1000);
    } //while
    return 0;
}
