# ROS driver for IFlyTek 6MIC-ARRAY
## Project Environment
* Software: Ubuntu 16.04 LTS, ROS Kinetic
* Hardware: IFlyTek XFM-EVB-01, XFM10621, 6MIC-ARRAY


## 1. Microphone Mode
---
* ### name: __mode_info_pub__    
* ### type: **std_msg::Int8**
* ### description: **publish the microphone mode**
* ### data range:**{-2, 0, 1, 2}
* ###  __Data__ |     __Description__ 
       0        |    eMicPhone_Closed
       1        |    eMicPhone_Activate
       2        |    eMicPhone_Communicate
       -2       |    eMicPhone_Communicate_Quit
       
       
       
       
## 2. Wake-up Angle
---
* ### name: __angle_pub__    
* ### type: **std_msg::Int8**
* ### description: **publish the wake-up angle**
* ### data rangle: **0-360**
       
    
