/** @file client.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  All the exampls for ROS are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

//Include Necessary Libs
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <iostream>
#include <cstring>
#include <cmath>
#include <future>

//Define namespaces
using namespace std;
using namespace cv;
using namespace DJI::onboardSDK;

//Define Structs
struct processedImage {
    Mat imgOrig;
    Mat imgThresh;
    vector<Vec3f> circles;
};

//Define vision system functions
processedImage processImage(Mat);
bool processData(int, int, int, int);
Mat processSection(Mat m);
void goTo(DJIDrone *drone, float posX, float posY);
void landOnTarget(DJIDrone *drone);
void flySideways(DJIDrone *drone,int distance, bool direction);
void flyCircle(DJIDrone *drone, int circleRadius);
void flyRectangle(DJIDrone *drone, int length, int width);
//! Function Prototypes for Mobile command callbacks - Core Functions
void ObtainControlMobileCallback(DJIDrone *drone);
void ReleaseControlMobileCallback(DJIDrone *drone);
void TakeOffMobileCallback(DJIDrone *drone);
void LandingMobileCallback(DJIDrone *drone);
void GetSDKVersionMobileCallback(DJIDrone *drone);
void ArmMobileCallback(DJIDrone *drone);
void DisarmMobileCallback(DJIDrone *drone);
void GoHomeMobileCallback(DJIDrone *drone);
void TakePhotoMobileCallback(DJIDrone *drone);
void StartVideoMobileCallback(DJIDrone *drone);
void StopVideoMobileCallback(DJIDrone *drone);
//! Function Prototypes for Mobile command callbacks - Custom Missions
void DrawCircleDemoMobileCallback(DJIDrone *drone);
void DrawSquareDemoMobileCallback(DJIDrone *drone);
void GimbalControlDemoMobileCallback(DJIDrone *drone);
void AttitudeControlDemoMobileCallback(DJIDrone *drone);
void LocalNavigationTestMobileCallback(DJIDrone *drone);
void GlobalNavigationTestMobileCallback(DJIDrone *drone);
void WaypointNavigationTestMobileCallback(DJIDrone *drone);
void VirtuaRCTestMobileCallback(DJIDrone *drone);
//! For LAS logging
void StartMapLASLoggingMobileCallback(DJIDrone *drone);
void StopMapLASLoggingMobileCallback(DJIDrone *drone);
void StartCollisionAvoidanceCallback(DJIDrone *drone);
void StopCollisionAvoidanceCallback(DJIDrone *drone);

//Global Variables
bool targetfound = false;
bool finishedFly = false;
float yaw = 0;

static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > -------------------------+\n");
    printf("| [1]  SDK Version Query        | [7]  Concentric Circle Search    |\n");
    printf("| [2]  Request Control          | [8]  Rectangle Sideways Search   |\n");
    printf("| [3]  Release Control          | [9]  Straight Ahead Search       |\n");
    printf("| [4]  Takeoff                  | [10] Test Rectangle Path         |\n");
    printf("| [5]  Landing                  | [11] Test Circle Path            |\n");
    printf("| [6]  Go Home                  | [12] Test Picamera               |\n");
    printf("| [13]                          | [99] Exit Program                | \n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}


int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;
    int circleRadius;
    int circleHeight;
    float Phi, circleRadiusIncrements;
    int x_center, y_center, yaw_local;
    bool valid_flag = false;
    bool err_flag = false;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

    //enable opencv optimizations
    setUseOptimized(true);

    //virtual RC test data
    uint32_t virtual_rc_data[16];

    //set frequency test data
    uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
    //waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

    //groundstation test data
    dji_sdk::MissionWaypointTask waypoint_task;
    dji_sdk::MissionWaypoint 	 waypoint;
    dji_sdk::MissionHotpointTask hotpoint_task;
    dji_sdk::MissionFollowmeTask followme_task;
    dji_sdk::MissionFollowmeTarget followme_target;
    uint8_t userData = 0;
    ros::spinOnce();

    //! Setting functions to be called for Mobile App Commands mode
    drone->setObtainControlMobileCallback(ObtainControlMobileCallback, &userData);
    drone->setReleaseControlMobileCallback(ReleaseControlMobileCallback, &userData);
    drone->setTakeOffMobileCallback(TakeOffMobileCallback, &userData);
    drone->setLandingMobileCallback(LandingMobileCallback, &userData);
    drone->setGetSDKVersionMobileCallback(GetSDKVersionMobileCallback, &userData);
    drone->setArmMobileCallback(ArmMobileCallback, &userData);
    drone->setDisarmMobileCallback(DisarmMobileCallback, &userData);
    drone->setGoHomeMobileCallback(GoHomeMobileCallback, &userData);
    drone->setTakePhotoMobileCallback(TakePhotoMobileCallback, &userData);
    drone->setStartVideoMobileCallback(StartVideoMobileCallback,&userData);
    drone->setStopVideoMobileCallback(StopVideoMobileCallback,&userData);
    drone->setDrawCircleDemoMobileCallback(DrawCircleDemoMobileCallback, &userData);
    drone->setDrawSquareDemoMobileCallback(DrawSquareDemoMobileCallback, &userData);
    drone->setGimbalControlDemoMobileCallback(GimbalControlDemoMobileCallback, &userData);
    drone->setAttitudeControlDemoMobileCallback(AttitudeControlDemoMobileCallback, &userData);
    drone->setLocalNavigationTestMobileCallback(LocalNavigationTestMobileCallback, &userData);
    drone->setGlobalNavigationTestMobileCallback(GlobalNavigationTestMobileCallback, &userData);
    drone->setWaypointNavigationTestMobileCallback(WaypointNavigationTestMobileCallback, &userData);
    drone->setVirtuaRCTestMobileCallback(VirtuaRCTestMobileCallback, &userData);

    drone->setStartMapLASLoggingMobileCallback(StartMapLASLoggingMobileCallback, &userData);
    drone->setStopMapLASLoggingMobileCallback(StopMapLASLoggingMobileCallback, &userData);
    drone->setStartCollisionAvoidanceCallback(StartCollisionAvoidanceCallback, &userData);
    drone->setStopCollisionAvoidanceCallback(StopCollisionAvoidanceCallback, &userData);


    Display_Main_Menu();
    while(1)
    {
        ros::spinOnce();
        std::cout << "Enter Input Val: ";
        while(!(std::cin >> temp32)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input.  Try again: ";
        }

        if(temp32>0 && temp32<100)
        {
            main_operate_code = temp32;
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }
        switch(main_operate_code)
        {
            case 1:
                /* SDK version query*/
                drone->check_version();
                break;
            case 2:
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 3:
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            case 4:
                /* take off */
                drone->takeoff();
                break;
            case 5:
                /* landing*/
                drone->landing();
                break;
            case 6:
                /* go home*/
                drone->gohome();
                break;
            case 7: {
                /*Concentric Circle Search*/
                drone->request_sdk_permission_control();
                sleep(1);
                drone->takeoff();
                sleep(8);
                for (int r = 3; r < 100; r += 3) {
                    finishedFly = false;
                    std::async(flyCircle, drone, r * M_PI);
                    while (!finishedFly && !targetfound) {
                        Mat img = captureImage();
                        processImage(img);
                    }
                    if (targetfound) {
                        break;
                    }
                }
                if (targetfound) {
                    landOnTarget(drone);
                } else {
                    drone->landing();
                }
            }
                break;
            case 8: {
                /* Rectangle End to End Search*/
                drone->request_sdk_permission_control();
                sleep(1);
                drone->takeoff();
                sleep(8);
                float x = drone->local_position.x;
                float y = drone->local_position.y;
                //25sin45
                x = x - 5.384;
                y = y + 7.62;
                yaw = 0;
                goTo(drone, x, y);
                finishedFly = false;
                std::async(flySideways, drone, 11, true);
                while (!finishedFly && !targetfound) {
                    Mat img = captureImage();
                    processImage(img);
                }
                if (targetfound) {
                    landOnTarget(drone);
                } else {
                    drone->landing();
                }
            }
                break;
            case 9: {
                /*Straight ahead search*/
                drone->request_sdk_permission_control();
                sleep(1);
                drone->takeoff();
                sleep(8);
                yaw = 0;
                float x = drone->local_position.x;
                float y = drone->local_position.y;
                //~19ft forward
                x = x;
                y = y + 6.7;
                yaw = 0;
                goTo(drone, x, y);
                while(!targetfound && y<9.4){
                    Mat img = captureImage();
                    processImage(img);
                    goTo(drone,x,y+0.05);
                }
                if (targetfound) {
                    landOnTarget(drone);
                }
                else {
                    for (int w = 6; w < 18; w += 6) {
                        std::async(flyRectangle, drone, w,10);
                        while (!finishedFly && !targetfound) {
                            Mat img = captureImage();
                            processImage(img);
                        }
                        if (targetfound) {
                            break;
                        }
                        float x = drone->local_position.x;
                        float y = drone->local_position.y;
                        goTo(drone,x,y+10);
                    }
                    if (targetfound) {
                        landOnTarget(drone);
                    } else {
                        drone->landing();
                    }
                }
            }
                break;

            case 10:
                /*draw square sample*/

                break;
            case 11: {
                /*draw circle sample*/
                std::cout << "Enter the radius of the circle in meteres (10m > x > 4m)\n";
                std::cin >> circleRad;
                flyCircle(drone, circleRad);
            }
                break;
            case 12: {
                //test picamera
                VideoCapture cap(0);
                //cap.set(CAP_PROP_FRAME_WIDTH, 1640);
                //cap.set(CAP_PROP_FRAME_HEIGHT, 922);
                Mat camImg;
                if(!cap.isOpened()){
                    cout << "cap fail" << endl;
                }
                cout << "make img" << endl;
                bool b = cap.read(camImg);
                if(!b){
                    cout << "img error" <<endl;
                }
                cout << "read img" << endl;
                imwrite("/home/pi/Desktop/image.jpg", camImg);
                cout << "end test" << endl;
            }
                break;
            case 13:
                /*stop video*/
                drone->stop_video();
                break;

            case 99:
                return 1;

            default:
                break;
        }
        main_operate_code = -1;
        Display_Main_Menu();
    }
    return 0;
}

//! Callback functions for Mobile Commands
void ObtainControlMobileCallback(DJIDrone *drone)
{
    drone->request_sdk_permission_control();
}

void ReleaseControlMobileCallback(DJIDrone *drone)
{
    drone->release_sdk_permission_control();
}

void TakeOffMobileCallback(DJIDrone *drone)
{
    drone->takeoff();
}

void LandingMobileCallback(DJIDrone *drone)
{
    drone->landing();
}

void GetSDKVersionMobileCallback(DJIDrone *drone)
{
    drone->check_version();
}

void ArmMobileCallback(DJIDrone *drone)
{
    drone->drone_arm();
}

void DisarmMobileCallback(DJIDrone *drone)
{
    drone->drone_disarm();
}

void GoHomeMobileCallback(DJIDrone *drone)
{
    drone->gohome();
}

void TakePhotoMobileCallback(DJIDrone *drone)
{
    drone->take_picture();
}

void StartVideoMobileCallback(DJIDrone *drone)
{
    drone->start_video();
}

void StopVideoMobileCallback(DJIDrone *drone)
{
    drone->stop_video();
}

void DrawCircleDemoMobileCallback(DJIDrone *drone)
{
    static float R = 2;
    static float V = 2;
    static float x;
    static float y;
    int circleRadius;
    int circleHeight;
    float Phi =0, circleRadiusIncrements;
    int x_center, y_center, yaw_local;

    circleHeight = 7;
    circleRadius = 7;

    x_center = drone->local_position.x;
    y_center = drone->local_position.y;
    circleRadiusIncrements = 0.01;

    for(int j = 0; j < 1000; j ++)
    {
        if (circleRadiusIncrements < circleRadius)
        {
            x =  x_center + circleRadiusIncrements;
            y =  y_center;
            circleRadiusIncrements = circleRadiusIncrements + 0.01;
            drone->local_position_control(x ,y ,circleHeight, 0);
            usleep(20000);
        }
        else
        {
            break;
        }
    }


    /* start to draw circle */
    for(int i = 0; i < 1890; i ++)
    {
        x =  x_center + circleRadius*cos((Phi/300));
        y =  y_center + circleRadius*sin((Phi/300));
        Phi = Phi+1;
        drone->local_position_control(x ,y ,circleHeight, 0);
        usleep(20000);
    }

}
void DrawSquareDemoMobileCallback(DJIDrone *drone)
{
    /*draw square sample*/
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 3, 3, 0, 0 );
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 -3, 3, 0, 0);
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 -3, -3, 0, 0);
        usleep(20000);
    }
    for(int i = 0;i < 60;i++)
    {
        drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                                 Flight::VerticalLogic::VERTICAL_VELOCITY |
                                 Flight::YawLogic::YAW_ANGLE |
                                 Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                                 Flight::SmoothMode::SMOOTH_ENABLE,
                                 3, -3, 0, 0);
        usleep(20000);
    }
}

void GimbalControlDemoMobileCallback(DJIDrone *drone)
{
    drone->gimbal_angle_control(0, 0, 1800, 20);
    sleep(2);
    drone->gimbal_angle_control(0, 0, -1800, 20);
    sleep(2);
    drone->gimbal_angle_control(300, 0, 0, 20);
    sleep(2);
    drone->gimbal_angle_control(-300, 0, 0, 20);
    sleep(2);
    drone->gimbal_angle_control(0, 300, 0, 20);
    sleep(2);
    drone->gimbal_angle_control(0, -300, 0, 20);
    sleep(2);
    drone->gimbal_speed_control(100, 0, 0);
    sleep(2);
    drone->gimbal_speed_control(-100, 0, 0);
    sleep(2);
    drone->gimbal_speed_control(0, 0, 200);
    sleep(2);
    drone->gimbal_speed_control(0, 0, -200);
    sleep(2);
    drone->gimbal_speed_control(0, 200, 0);
    sleep(2);
    drone->gimbal_speed_control(0, -200, 0);
    sleep(2);
    drone->gimbal_angle_control(0, 0, 0, 20);
}

void AttitudeControlDemoMobileCallback(DJIDrone *drone)
{
    /* attitude control sample*/
    drone->takeoff();
    sleep(8);


    for(int i = 0; i < 100; i ++)
    {
        if(i < 90)
            drone->attitude_control(0x40, 0, 2, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 2, 0, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, -2, 0, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, 2, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, -2, 0, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, 0, 0.5, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0x40, 0, 0, -0.5, 0);
        else
            drone->attitude_control(0x40, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0xA, 0, 0, 0, 90);
        else
            drone->attitude_control(0xA, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    for(int i = 0; i < 200; i ++)
    {
        if(i < 180)
            drone->attitude_control(0xA, 0, 0, 0, -90);
        else
            drone->attitude_control(0xA, 0, 0, 0, 0);
        usleep(20000);
    }
    sleep(1);

    drone->landing();

}
void LocalNavigationTestMobileCallback(DJIDrone *drone)
{

}
void GlobalNavigationTestMobileCallback(DJIDrone *drone)
{

}
void WaypointNavigationTestMobileCallback(DJIDrone *drone)
{

}
void VirtuaRCTestMobileCallback(DJIDrone *drone)
{
    //virtual RC test data
    uint32_t virtual_rc_data[16];
    //virtual rc test 1: arm & disarm
    drone->virtual_rc_enable();
    usleep(20000);

    virtual_rc_data[0] = 1024;  //0-> roll      [1024-660,1024+660]
    virtual_rc_data[1] = 1024;  //1-> pitch     [1024-660,1024+660]
    virtual_rc_data[2] = 1024+660;  //2-> throttle  [1024-660,1024+660]
    virtual_rc_data[3] = 1024;  //3-> yaw       [1024-660,1024+660]
    virtual_rc_data[4] = 1684;      //4-> gear      {1684(UP), 1324(DOWN)}
    virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

    for (int i = 0; i < 100; i++){
        drone->virtual_rc_control(virtual_rc_data);
        usleep(20000);
    }

    //virtual rc test 2: yaw
    drone->virtual_rc_enable();
    virtual_rc_data[0] = 1024;      //0-> roll      [1024-660,1024+660]
    virtual_rc_data[1] = 1024;      //1-> pitch     [1024-660,1024+660]
    virtual_rc_data[2] = 1024-200;  //2-> throttle  [1024-660,1024+660]
    virtual_rc_data[3] = 1024;      //3-> yaw       [1024-660,1024+660]
    virtual_rc_data[4] = 1324;      //4-> gear      {1684(UP), 1324(DOWN)}
    virtual_rc_data[6] = 1552;      //6-> mode      {1552(P), 1024(A), 496(F)}

    for(int i = 0; i < 100; i++) {
        drone->virtual_rc_control(virtual_rc_data);
        usleep(20000);
    }
    drone->virtual_rc_disable();
}

void StartMapLASLoggingMobileCallback(DJIDrone *drone)
{
    system("roslaunch point_cloud_las start_velodyne_and_loam.launch &");
    system("rosrun point_cloud_las write _topic:=/laser_cloud_surround _folder_path:=. &");
}

void StopMapLASLoggingMobileCallback(DJIDrone *drone)
{
    system("rosnode kill /write_LAS /scanRegistration /laserMapping /transformMaintenance /laserOdometry  &");
}

void StartCollisionAvoidanceCallback(DJIDrone *drone)
{
    uint8_t freq[16];
    freq[0] = 1;    // 0 - Timestamp
    freq[1] = 4;    // 1 - Attitude Quaterniouns
    freq[2] = 1;    // 2 - Acceleration
    freq[3] = 4;    // 3 - Velocity (Ground Frame)
    freq[4] = 4;    // 4 - Angular Velocity (Body Frame)
    freq[5] = 3;    // 5 - Position
    freq[6] = 0;    // 6 - Magnetometer
    freq[7] = 3;    // 7 - M100:RC Channels Data, A3:RTK Detailed Information
    freq[8] = 0;    // 8 - M100:Gimbal Data, A3: Magnetometer
    freq[9] = 3;    // 9 - M100:Flight Status, A3: RC Channels
    freq[10] = 0;   // 10 - M100:Battery Level, A3: Gimble Data
    freq[11] = 2;   // 11 - M100:Control Information, A3: Flight Status

    drone->set_message_frequency(freq);
    usleep(1e4);
    system("roslaunch dji_collision_avoidance from_DJI_ros_demo.launch &");
}

void StopCollisionAvoidanceCallback(DJIDrone *drone)
{
    drone->release_sdk_permission_control();
    system("rosnode kill /drone_tf_builder /dji_occupancy_grid_node /dji_collision_detection_node /collision_velodyne_nodelet_manager /manual_fly");
    usleep(1e4);
    drone->request_sdk_permission_control();
}

//Reads in next from from camera and returns it
Mat captureImage(){
    Mat img;
    //Define video stream
    VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH, maxX);
    cap.set(CAP_PROP_FRAME_HEIGHT, maxY);
    cap.set(CAP_PROP_FPS, 40);
    bool bSuccess = cap.read(img); // read a new frame from video
    return img;
}//captureImage

//Processes image passed for the landing pad using hsv range passed
processedImage processImage(Mat imgOriginal) {
    Mat imgHSV;
    //predefine HSV color range
    int iLowH = 7;
    int iHighH = 25;
    int iLowS = 120;
    int iHighS = 240;
    int iLowV = 120;
    int iHighV = 225;
    vector<int> hsvVals = { iLowH, iLowS, iLowV, iHighH, iHighS, iHighV };
    imgHSV = imgOriginal.clone();
    cvtColor(imgHSV, imgHSV, COLOR_BGR2HSV);
    inRange(imgHSV, Scalar(hsvVals[0], hsvVals[1], hsvVals[2]), Scalar(hsvVals[3], hsvVals[4], hsvVals[5]), imgHSV);

    Mat top_left = (imgHSV(Rect(0, 0, 320, 240)));
    Mat top_right = (imgHSV(Rect(320, 0, 320, 240)));
    Mat bottom_left = (imgHSV(Rect(0, 240, 320, 240)));
    Mat bottom_right = (imgHSV(Rect(320, 240, 320, 240)));

    //Convert the captured frame from BGR to HSV
    auto f1 = std::async(processSection, top_left);
    auto f2 = std::async(processSection, top_right);
    auto f3 = std::async(processSection, bottom_left);
    auto f4 = std::async(processSection, bottom_right);
    top_left = f1.get();
    top_right = f2.get();
    bottom_left = f3.get();
    bottom_right = f4.get();

    vector<Vec3f> circles;

    //Find target
    HoughCircles(imgHSV, circles, CV_HOUGH_GRADIENT, 1, 10000, 200, 35, 5, 500);

    //Draw circles on top of original image
    for (int i = 0; i < circles.size(); i++)
    {
        targetfound = true;
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        //Draw an indicator on the center of mass
        circle(imgOriginal, center, 5, Scalar(0, 255, 0), -1, 8, 0);//center point
        //circle(imgOriginal, center, radius, Scalar(0, 0, 255), 2, 8, 0);//outline
    }

    //Return data to main function
    struct processedImage output = { imgOriginal,imgHSV,circles };
    return output;
}//processImage

//Used for multithreading image processing
Mat processSection(Mat m) {
    medianBlur(m, m, 5);
    return m;
}//processSection

//Fly a rectangular path
void flyRectangle(DJIDrone *drone, int length, int width){
    yaw = 0;
    float x = drone->local_position.x;
    float y = drone->local_position.y;
    float posX;
    float posY = y;
    float increment = 0.05;
    if(length <= 0){
        posX = x-length;
    }
    else{
        posX = x+length;
    }
    while((cvRound(x) != cvRound(posX)) && !targetfound)
    {
        if(cvRound(x) < cvRound(posX)) {
            x = x + 0.05;
        }
        else{
            x = x-0.05;
        }
        drone->local_position_control(x,y ,6.4, yaw);
        usleep(20000);
    }
    float x = drone->local_position.x;
    float y = drone->local_position.y;
    posY = y-width;
    while(cvRound(y)!= cvRound(posY) && !targetfound)
    {
        if(cvRound(y) < cvRound(posY)) {
            y = y + 0.05;
        }
        else{
            y = y-0.05;
        }
        drone->local_position_control(x,y ,6.4, yaw);
        usleep(20000);
    }
    float x = drone->local_position.x;
    float y = drone->local_position.y;
    if(length <= 0){
        posX = x+length;
    }
    else{
        posX = x-length;
    }
    while((cvRound(x) != cvRound(posX)) && !targetfound)
    {
        if(cvRound(x) < cvRound(posX)) {
            x = x + 0.05;
        }
        else{
            x = x-0.05;
        }
        drone->local_position_control(x,y ,6.4, yaw);
        usleep(20000);
    }
    float x = drone->local_position.x;
    float y = drone->local_position.y;
    posY = y+width;
    while(cvRound(y)!= cvRound(posY) && !targetfound)
    {
        if(cvRound(y) < cvRound(posY)) {
            y = y + 0.05;
        }
        else{
            y = y-0.05;
        }
        drone->local_position_control(x,y ,6.4, yaw);
        usleep(20000);
    }
    finishedFly = true;
}

//Fly horizontally if true, right, false is left
void flySideways(DJIDrone *drone,int distance, bool direction){
    yaw = 0;
    float x = drone->local_position.x;
    float y = drone->local_position.y;
    float posX;
    float posY = y;
    float increment = 0.05;
    if(direction){
        posX = x+distance;
    }
    else{
        posX = x-distance;
    }
    while((cvRound(x) != cvRound(posX) || cvRound(y)!= cvRound(posY)) && !targetfound)
    {
        if(cvRound(x) < cvRound(posX)) {
            x = x + 0.05;
        }
        else if(cvRound(x) > cvRound(posX)){
            x = x-0.05;
        }
        if(cvRound(y) < cvRound(posY)) {
            y = y + 0.05;
        }
        else if(cvRound(y) > cvRound(posY)){
            y = y-0.05;
        }
        drone->local_position_control(x,y ,6.4, yaw);
        usleep(20000);
    }
    finishedFly = true;
}

//Fly a circle of a given radius
void flyCircle(DJIDrone *drone, int circleRadius){
    static float R = 2;
    static float V = 2;
    static float x;
    static float y;
    Phi = 0.000001;

    static int circleHeight = 6.4;

    x_center = drone->local_position.x;
    y_center = drone->local_position.y;
    circleRadiusIncrements = 0.0;

    while(circleRadiusIncrements <= circleRadius && !targetfound)
    {
        x =  x_center + circleRadiusIncrements;
        y =  y_center;
        circleRadiusIncrements = circleRadiusIncrements + 0.05;
        drone->local_position_control(x ,y ,circleHeight, 0);
        usleep(20000);
    }
    usleep(500000);
    yaw = 0;
    static float x_start = drone->local_position.x;
    static float y_start = drone->local_position.y;
    cout << "x_center: " << x_center <<endl;
    cout << "y_center: " << y_center <<endl;
    static int scale = floor(circleRadius/15)+1;
    static int loops = 1890*scale;
    /* start to draw circle */
    for(int i = 0; i < loops; i ++)
    {
        if(targetfound){
           break;
        }
        x =  x_center + circleRadius*cos((Phi/300));
        y =  y_center + circleRadius*sin((Phi/300));
        Phi = Phi+(1/scale);
        yaw = Phi/300;
        yaw = (yaw*180)/(M_PI);
        if(yaw > 180){
            yaw = yaw-360;
        }
        drone->local_position_control(x ,y ,circleHeight, yaw);
        usleep(20000);
    }
    finishedFly = true;
}
void goTo(DJIDrone *drone, float posX, float posY){
    yaw = 0;
    float x = drone->local_position.x;
    float y = drone->local_position.y;
    float increment = 0.05;
    while(cvRound(x) != cvRound(posX) || cvRound(y)!=cvRound(posY))
    {
        if(cvRound(x) < cvRound(posX)) {
            x = x + 0.05;
        }
        else if(cvRound(x) > cvRound(posX)){
            x = x-0.05;
        }
        if(cvRound(y) < cvRound(posY)) {
            y = y + 0.05;
        }
        else if(cvRound(y) > cvRound(posY)){
            y = y-0.05;
        }
        drone->local_position_control(x,y ,6.4, yaw);
        usleep(20000);
    }
}
void landOnTarget(DJIDrone *drone){
    processedImage output;
    Mat img = captureImage();
    output = processImage(img);
    vector <Vec3f> circles = output.circles;
    float posX = circles[0][0];
    float posY = circles[0][1];
    goTo(drone,posX,posY);
    drone->landing();

}
