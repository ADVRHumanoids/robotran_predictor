
from numpy import loadtxt
import numpy as np
import matplotlib.pyplot as plt

#####################
#robot state
#####################

data_array = loadtxt("robot_state/data.log", delimiter=" ", unpack=False)

time_1 = data_array[:,1]

 # joint id
 # 0->5   // [LHipLat, LHipYaw, LHipSag, LKneeSag, LAnkSag, LAnkLat,
 # 6->11  // RHipLat, RHipYaw, RHipSag, RKneeSag, RAnkSag, RAnkLat,
 # 12->14 // WaistLat, WaistSag, WaistYaw,
 # 15->21 // LShSag, LShLat, LShYaw, LElbj, LForearmPlate, LWrj1, LWrj2,
 # 22->23 // NeckYawj, NeckPitchj,
 # 24->30 // RShSag, RShLat, RShYaw, RElbj, RForearmPlate, RWrj1, RWrj2]

LHipLat_id = 0;
LHipYaw_id = 1;
LKneeSag_id = 3;
WaistSag_id = 13;
LElbj_id = 18;


LHipLat_pos = data_array[:,3+LHipLat_id*11]
LHipYaw_pos = data_array[:,3+LHipYaw_id*11]
LKneeSag_pos = data_array[:,3+LKneeSag_id*11]
WaistSag_pos = data_array[:,3+WaistSag_id*11]
LElbj_pos = data_array[:,3+LElbj_id*11]

#plt.plot(time_1, LHipLat_pos)
#plt.plot(time_1, LHipYaw_pos)
plt.plot(time_1, LKneeSag_pos)
#plt.plot(time_1, WaistSag_pos)
#plt.plot(time_1, LElbj_pos)

#####################
# prediction request
#####################

data_array_2 = loadtxt("prediction_request/data.log",
                       usecols = (0,1,3,4,5),
                       delimiter=" ", unpack=False)

time_2 = data_array_2[:,1]
pred_req = data_array_2[:,3]*0.0015

plt.plot(time_2, pred_req,'o')

for i in range(0,len(time_2)):
    plt.axvline(time_2[i])

#####################
# prediction
#####################

data_array_3 = loadtxt("prediction/data.log",
                       usecols = (0,1,3,4,5),
                       delimiter=" ", unpack=False)

time_3 = data_array_3[:,1]
pred = data_array_3[:,2]
plt.plot(time_3, pred,'o')

plt.show()