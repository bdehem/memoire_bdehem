import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os

def fillnans(file_in):
    "fill with nans between displacements"
    print(file_in)
    infile  = '/home/bor/bagfiles/benchmark_point_threshold/' + file_in
    outfile = '/home/bor/bagfiles/benchmark_point_threshold/' + 'nn' + file_in
    topicofinterest = '/pose_estimation'
    with rosbag.Bag(outfile, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(infile).read_messages():
            if (topic == topicofinterest and msg.x == 0):
                msg.x    = float('NaN')
                msg.y    = float('NaN')
                msg.z    = float('NaN')
                msg.rotX = float('NaN')
                msg.rotY = float('NaN')
                msg.rotZ = float('NaN')
            outbag.write(topic, msg, t)
    return

fillnans("__result_benchmark_precision_thresh_250_maxmatches_-1_noBA_false_naivetriang_false_robust_false_bundleadjustmenttol_0.00001.bag")
