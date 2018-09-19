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
I thought I can do ION correction with ionex, but it turned out to be a little difficult. \
Accomplished the interface of RTCM3 Differ, wait for testing.
###### 9-11
I took a class today and haven't done very much, but I managed to finish the sp3 file reading which is the most easy one
# PLANS FOR THIS WEEK
I'll try DGPS interface first and then try some linear-combined observation methods this week.
I planed to use a wide lane which I never tried before.
###### 9-12
I set a new class file DIYSolver, which is to test the behavior of some PPP method. \
Currently it doesn't work very well, DO NOT compile it with other code and DO NOT laugh at me :)\
Or try to hold that.
###### 9-14
I tried to use batch least square method to solve float ambiguity of L1 and it worked.  \
The result looks like splited lines, due to the fact that I used nothing for cycle-slips. \
I will try sequence LS and cycle-slip detector to get better results. \
Later afternoon today I finished the test of batch least square and it is still suffering. \
Sequence LS is applied for SPP and tomorrow I will work on PPP edition of Sequence LS. \
Also a new set of data file has been uploaded which is the one I'm currently using.
###### 9-18 Simple (undebuggable) PPP applied & TJ 了
I applied IF combine & sequence LS filter for this PPP function but still had troubles. \
Recently I am gonna work on my lecture jobs, so maybe updating slowly. \
And a troposphere model class has been created, using hopefield and NMF model. \
###### 9-19 Merging function of RINEX applied
A function has been created to merge RINEX files(nav or obs).\
It helps to solve merging highrate observation files from igs: \
<a href="ftp://cddis.gsfc.nasa.gov/pub/gps/data/highrate" target="_blank">ftp://cddis.gsfc.nasa.gov/pub/gps/data/highrate</a>

