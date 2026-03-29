# Power Safety Board

## Overview
The purpose of the power safety board is to control the power electrically distributed. In the event of emergency, this board can shut down part or all of the systems electronics. 

Power distribution is managed electrically, but whether is is disabled or not is controlled by software. The center of this board is managed by an STM32 which is capable of communicating the current power statistics. However, the signals for toggling power safety functions are sent through dedicated GPiO. 
