# GNSSJY
GNSS functions and modules designed by myself, keeping updating.
# Description
I realized simple point position function, as well as basic framework initializations recently. 
# Latest Update
###### 9-6  
EKF-based Solver is applied, waiting for testing. 
###### 9-6  
Ouput interfaces is done, wait for testing, done debug by myself. 
###### 9-7  
Range-smoothing Solver done, no bugs currently found. 
###### 9-7  
An image printing function has been added to the Output modules. See Output.h to know more. 
###### 9-8  
Now I'm trying to make the program have the ability to deal with ion & tro.  
     You can find an IONEX reader in RINEX2.h. 
###### 9-10 
I thought I can do ION corroection with ionex, but it turned out to be a little difficult. \
Accomplished the interface of RTCM3 Differ, wait for testing.
# PLANS
I'll try DGPS interface first and then try linear-combined observation method this week.
