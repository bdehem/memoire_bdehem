import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os

def repairfile( file_in, is_result = False, file_out = 'output' ):
    "replace -0.04 by 0.04"
    infile  = '/home/bor/bagfiles/benchmark_point_threshold/' + file_in
    print(infile)
    topicofinterest = '/pose_estimation' if is_result else '/manual_pose_estimation'
    print(topicofinterest)

    with rosbag.Bag(file_out, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(infile).read_messages():
            if (topic == topicofinterest and msg.y < -0.03 and msg.y >-0.05):
                print(msg)
                msg.y = 0.04
                print(msg)
            outbag.write(topic, msg, t)
    return


for fn in os.listdir('/home/bor/bagfiles/benchmark_point_threshold/'):
    repairfile( fn, is_result = True, file_out = "__" + fn )

if(len(sys.argv)==1):
    print('please enter a file name')
elif(len(sys.argv)==2):
    repairfile(sys.argv[1])
elif(len(sys.argv)==3):
    repairfile(sys.argv[1],sys.argv[2])
else:
    repairfile(sys.argv[1],sys.argv[2],sys.argv[3])
