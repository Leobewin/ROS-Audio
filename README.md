# ROS driver for IFlyTek 6MIC-ARRAY
## Project Environment
* Software: Ubuntu 16.04 LTS, ROS Kinetic
* Hardware: IFlyTek XFM-EVB-01, XFM10621, 6MIC-ARRAY


## Publisher 
---
* ### name: __mode_info_pub__    
* ### type: **std_msg::Int8**
* ###  __Data__ |     __Description__
       0        |    eMicPhone_Closed
       1        |    eMicPhone_Activate
       2        |    eMicPhone_Communicate
       -2       |    eMicPhone_Communicate_Quit
