# ROS driver for IFlyTek 6MIC-ARRAY
## Project Environment
* Software: Ubuntu 16.04 LTS, ROS Kinetic
* Hardware: IFlyTek XFM-EVB-01, XFM10621, 6MIC-ARRAY

# Publisher

## 1. Microphone Mode
* ### name: __mode_info_pub__    
* ### type: **std_msg::Int8**
* ### description: **publish the microphone mode**
* ### data range: **{-2, 0, 1, 2}**
* ###  __Data__ |     __Description__ 
       0        |    eMicPhone_Closed
       1        |    eMicPhone_Activate
       2        |    eMicPhone_Communicate
       -2       |    eMicPhone_Communicate_Quit
---
       
       
       
       
## 2. Wake-up Angle
* ### name: __angle_pub__    
* ### type: **std_msg::Int8**
* ### description: **publish the wake-up angle**
* ### data rangle: **0~360**
---

## 3. Communicate Mode
* ### name: __communicate_pub__    
* ### type: **std_msg::Bool**
* ### description: **publish the command to start Python node to record audio**
* ### data rangle: **{False, True}**
* ###  __Data__ |     __Description__ 
       False        |    stop communication
       True         |    start communication
---
 
# Subscriber
 
## 1. Microphone Command
* ### name: __microphone_cmd_sub__    
* ### type: **std_msg::Int8**
* ### description: **receive commands from user to change the microphone mode**
* ### data rangle: **{-2, 0, 2}**
* ###  __Data__ |     __Description__ 
       -2        |    stop communication
       2         |    start communication
       0         |    stop 
---
 

    
