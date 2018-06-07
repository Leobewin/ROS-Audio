#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 

#include <stdio.h>
extern "C"
{
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
#include "libavdevice/avdevice.h"
}



#include <iostream>
using namespace std;

#define eMicPhone_Closed 0
#define eMicPhone_Activate 1
#define eMicPhone_Communicate 2


/*UART COMMAND*/
const char VERSION[]= "VER\n";
const char TALKER[] = "CALL_MODE 1\n";
const char LOCALIZATION[] = "CALL_MODE 0\n";
const char WAKE_UP_YES[] = "WAKEUP 1\n";
const char WAKE_UP_NO[] = "WAKEUP 0\n";
const char BEAM_0[] = "BEAM 0\n";
const char BEAM_1[] = "BEAM 1\n";
const char BEAM_2[] = "BEAM 2\n";
const char BEAM_3[] = "BEAM 3\n";
const char BEAM_4[] = "BEAM 4\n";
const char BEAM_5[] = "BEAM 5\n";


serial::Serial ser;





int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial_communication");
    ros::NodeHandle nh;



    /*set Serial Port values*/
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(time_out);
        ser.setStopbits(serial::stopbits_one);
        ser.setFlowcontrol(serial::flowcontrol_none);
        ser.setParity(serial::parity_none);
        ser.open();
    }
    catch (serial::IOException& e)
    {   
        ROS_ERROR_STREAM("Unable to open Serial Port");
        return -1;
    }

    /*make sure the Serial Port is opened*/ 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    }


    

    ros::Rate loop_rate(1);
    int current_mode;
    while (ros::ok())   
    {



        /*
        ser.write(VERSION);
        if(ser.available())
        {
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read " << result.data << endl);
            loop_rate.sleep();
        }
        else
        {
            ROS_INFO_STREAM("Not available");
            loop_rate.sleep();
        }
        */
        
    }

    


   
    return 0;
}

