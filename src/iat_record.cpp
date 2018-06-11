/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

/*
	@author  Alex Wang
	@date    2018.06.08

*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <iconv.h>

/* for ROS and C++*/
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h> 
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib>
using namespace std;



#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

/* microphone mode */
#define eMicPhone_Closed 				0
#define eMicPhone_Activate 				1
#define eMicPhone_Communicate 			2
#define eMicPhone_Communicate_Quit 	   -2


/* UART COMMAND */
#define VERSION 		"VER\n"
#define RESET			"RESET\n"
#define TALK  			"CALL_MODE 1\n"
#define LOCALIZATION 	"CALL_MODE 0\n"
#define WAKE_UP_YES  	"WAKEUP 1\n"
#define WAKE_UP_NO  	"WAKEUP 0\n"
#define BEAM_0  		"BEAM 0\n"
#define BEAM_1  		"BEAM 1\n"
#define BEAM_2  		"BEAM 2\n"
#define BEAM_3  		"BEAM 3\n"
#define BEAM_4  		"BEAM 4\n"
#define BEAM_5  		"BEAM 5\n"


static void show_result(char *string, char is_over)
{
	
    printf("\rResult:[ %s ]",  string);

    if(is_over)
		putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}

void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}

void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
	{	
		printf("\nSpeaking done \n");
	}
	else
		printf("\nRecognizer error %d\n", reason);
}


/* demo recognize the audio from microphone */
static void demo_mic( char* session_begin_params) /* const char* session_begin_params */
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}

	/* demo 10 seconds recording */
	while(i++ < 10)
	{
		sleep(1);
	}

	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}
	sr_uninit(&iat);
}



/* the mode of the microphone */
volatile int microphone_mode;
volatile int microphone_mode_cmd;
void microphone_cmd_cb(const std_msgs::Int8ConstPtr &msg)
{

	if(msg->data == 0)
	{
		microphone_mode_cmd = eMicPhone_Closed; 
		//cout << "microphone mode: Closed" << endl;
	}

	if(msg->data == 2)
	{
		microphone_mode_cmd = eMicPhone_Communicate;
		//cout << "microphone mode: Communicate" << endl;
	}

	if(msg->data == -2)
	{
		microphone_mode_cmd = eMicPhone_Communicate_Quit;
		//cout << "microphone mode: Communicate_Quit" << endl;
	}
}


/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
int main(int argc, char* argv[])
{
	/* ros */
	ros::init(argc, argv, "ros_audio");
	ros::NodeHandle nh;

	/* publish the microphone state information */
	ros::Publisher mode_info_pub = nh.advertise<std_msgs::Int8>("microphone_mode_info", 1000);
	std_msgs::Int8 mode_info;

	/* publish the wake up angle */
	ros::Publisher angle_pub = nh.advertise<std_msgs::Int8>("wake_up_angle", 1000);
	std_msgs::Int8 angle_info;

	/* publish communication cmd */
	ros::Publisher communicate_pub = nh.advertise<std_msgs::Bool>("communicate",1000);
	std_msgs::Bool communicate_mode;

	/* subscribe microphone command to change the state of 6MIC ARRAY */
	ros::Subscriber sub = nh.subscribe("microphone_mode_cmd",1000,microphone_cmd_cb);

	ros::AsyncSpinner spinner(4);
	spinner.start();

init:	
	/*set Serial Port values*/
	serial::Serial ser;
    try
    {
		/* Port need to be changed accordin to system settings */
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
		/* bash command */
		//system("echo 'passwd' | sudo -S chmod 777 /dev/ttyUSB* ");
        //goto init;
    }

    /* make sure the Serial Port is opened */ 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1;
    }

	/* the angle when waked up */
	int angle;

	/* SDK from IFlytek*/
	int ret = MSP_SUCCESS;
	int upload_on =	1; /* whether upload the user word */
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 5b0f9ad2, work_dir = .";
	int aud_src = 0; /* from mic or file */
	int next = 0;
	bool activated = false;

	/*
	* See "iFlytek MSC Reference Manual"
	*/
	char* session_begin_params =
	"sub = iat, domain = iat, language = zh_cn, "
	"accent = mandarin, sample_rate = 16000, "
	"result_type = plain, result_encoding = utf8";

	/* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes 
	 * */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}

	cout << "If you want to use the microphone, please activate it first or please use the communicate mode ..!" << endl;
	while(ros::ok())
	{	
		aud_src = 1;
		if(aud_src != 0) 
		{
			
			if(microphone_mode_cmd == eMicPhone_Communicate)
			{
				microphone_mode = eMicPhone_Communicate;
				cout << "change to communicating mode ..." << endl;
				ser.write(TALK);
				mode_info.data = eMicPhone_Communicate;
				mode_info_pub.publish(mode_info);
				communicate_mode.data = true;
				communicate_pub.publish(communicate_mode);
				ros::Duration(5).sleep();
				microphone_mode_cmd = -99;
			}
			if(microphone_mode_cmd == eMicPhone_Communicate_Quit)
			{
				microphone_mode = eMicPhone_Communicate_Quit;
				cout << "quit communicating mode ..." << endl;
				mode_info.data = eMicPhone_Communicate_Quit;
				mode_info_pub.publish(mode_info);
				communicate_mode.data = false;
				communicate_pub.publish(communicate_mode);
				ros::Duration(5).sleep();
				microphone_mode_cmd = -99;
			}
			

activate:
			if(microphone_mode != eMicPhone_Communicate)
			{
				/* wake up and localization mode */
				ser.write(LOCALIZATION);
				ser.write(WAKE_UP_YES);
				/* read Serial data */
				if(ser.available())
				{
					std_msgs::String result;
					result.data = ser.read(ser.available());
					string s = result.data;
					/* get the angle */
					int pos = s.find("angle:");
					if(pos != -1 )
					{	
						cout << "    activated!    " << endl;
						/* activation mode */
						microphone_mode = eMicPhone_Activate;
						mode_info.data = eMicPhone_Activate;
						mode_info_pub.publish(mode_info);
						activated = true;		
						/* get the angle */
						string ang;
						ang  = s.substr(pos+6,3);
						stringstream ss;
						ss << ang;
						ss >> angle;
						cout << "angle=" << angle << endl; 
						angle_info.data = angle;
						angle_pub.publish(angle_info);
					}
					else
					{
						if(microphone_mode != eMicPhone_Activate)
						{
							ROS_INFO_STREAM("Please activate the microphone !" << endl);	
							goto activate;
						}
					}
				}

				if(microphone_mode == eMicPhone_Activate)
				{
					printf(" Recognizing the speech from microphone\n");
					printf("Speak in 10 seconds\n");
					demo_mic(session_begin_params);
					printf("10 sec passed\n");
					string audio = g_result;
					if(!audio.empty())
					{
						/*remove punctuation*/
						audio.erase(audio.size()-3,3);
						cout << "received " << audio << endl;
						/* do Semantic Analysis */
					}
					else
					{
close:
						if(microphone_mode != eMicPhone_Closed)
						{
							cout << "No audio data received, closing soon..." << endl;
							cout << "If you want to use the microphone, please reactivate it !" << endl;						
							/* reset and will receive no audio result */
							ser.write(RESET); 
							/* closed mode */
							microphone_mode = eMicPhone_Closed;
							mode_info.data = eMicPhone_Closed;
							mode_info_pub.publish(mode_info);
							activated = false;
						}
					}

				}
			
			}
		}
	}
exit:
	MSPLogout(); // Logout...

	return 0;
}
