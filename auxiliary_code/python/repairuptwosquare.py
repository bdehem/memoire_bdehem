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
    infile  = '/home/bor/bagfiles/' + file_in
    print(infile)
    topicofinterest = '/manual_pose_estimation'
    print(topicofinterest)
    count = 0
    niter = 0
    with rosbag.Bag(file_out, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(infile).read_messages():
            niter = niter+1
            if niter % 100 ==0:
                print(niter)
            if (topic == topicofinterest):
                count = count+1
                print(msg)
                print(count)
                if count < 4 :
                    outbag.write(topic, msg, t)
                else:
                    print("not writing")
            else:
                outbag.write(topic, msg, t)
    return


#for fn in os.listdir('/home/bor/bagfiles/benchmark_point_threshold/'):
fn = "uptwostoolsquare.bag"
repairfile(fn , is_result = True, file_out = "__" + fn )

#if(len(sys.argv)==1):
    #print('please enter a file name')
#elif(len(sys.argv)==2):
    #repairfile(sys.argv[1])
#elif(len(sys.argv)==3):
    #repairfile(sys.argv[1],sys.argv[2])
#else:
    #repairfile(sys.argv[1],sys.argv[2],sys.argv[3])
