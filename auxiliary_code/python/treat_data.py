import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import math
import subprocess, yaml
import rosbag_pandas
import sys
import os

def treat_data(file_in,do_plot):
    vis = rosbag_pandas.bag_to_dataframe(file_in,include="/pose_visual")
    man = rosbag_pandas.bag_to_dataframe(file_in,include="/pose_estimation")
    timediff = (vis.index[0] - man.index[0]).total_seconds()
    ref = pd.read_pickle('reference_poses.pkl')
    vis = vis.set_index((vis.index - vis.index[0]).total_seconds() + timediff)
    k = 0
    count = 0
    errDist = 0
    errAng = 0
    for row in vis.itertuples():
        while row.Index > ref.iloc[[k]].index:
            k+=1
        if not (ref['pose_estimation__x'].iloc[[k]].values == 0 or math.isnan(ref['pose_estimation__x'].iloc[[k]])):
            errX    = row.pose_visual__x    - ref['pose_estimation__x'].iloc[[k]].values
            errY    = row.pose_visual__y    - ref['pose_estimation__y'].iloc[[k]].values
            errrotZ = row.pose_visual__rotZ - ref['pose_estimation__rotZ'].iloc[[k]].values
            errDist += (errX**2 + errY**2)**(0.5)
            errAng  += abs(errrotZ)
            count   += 1
    avgerrD = (errDist/count)
    avgerrR = (errAng/count)*180/math.pi


    pts_map = np.zeros((3))
    k = 0
    batime = 0
    for topic, msg, t in rosbag.Bag(file_in).read_messages():
        if topic == "/benchmark":
            batime += msg.BA_times_pass1
            batime += msg.BA_times_pass2
            pts_map[k] = msg.pts_map
            k+=1

    if do_plot:
        f, (axx, axy, axr) = plt.subplots(3, sharex=True)
        axx.plot(ref.index, ref['pose_estimation__x'])
        axx.plot(vis.index, vis['pose_visual__x'])
        axy.plot(ref.index, ref['pose_estimation__y'])
        axy.plot(vis.index, vis['pose_visual__y'])
        axr.plot(ref.index, ref['pose_estimation__rotZ'])
        axr.plot(vis.index, vis['pose_visual__rotZ'])

        axx.set_title('X')
        axy.set_title('Y')
        axr.set_title('rotZ')
    return (avgerrD,avgerrR,batime)

def main():
    fn = "vanillaBA_robust.bag"
    dirname = '/home/bor/bagfiles/otherpcresults/'
    dirname = '/home/bor/bagfiles/aout/bag/'
    avgerrD,avgerrR,batime = treat_data(dirname + fn,True)
    print(fn)
    print(avgerrD)
    print(avgerrR)
    print(batime)
    plt.show()

if __name__ == "__main__":
    main()
