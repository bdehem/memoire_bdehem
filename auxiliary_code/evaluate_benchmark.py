import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import math
import subprocess, yaml
import rosbag_pandas
import sys
import os

# This file contains functions to evaluate the output of a benchmark test. The rosbag file should be used as file_in argument
# Set do_plot to true to plot the result.
# This file is hardcoded for the benchmark that was created during the 2017 thesis.

def evaluate_benchmark(file_in, do_plot):
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
        refplot = ref.ix[145:325]
        visplot = vis.ix[145:325]
        f, (axx, axy, axr) = plt.subplots(3, sharex=True)
        refpose = axx.plot(refplot.index, refplot['pose_estimation__x'],label='True Posesk')
        vispose = axx.plot(visplot.index, visplot['pose_visual__x'],label='Visual Estimationsk')
        axy.plot(refplot.index, refplot['pose_estimation__y'])
        axy.plot(visplot.index, visplot['pose_visual__y'])
        axr.plot(refplot.index, refplot['pose_estimation__rotZ']*180/math.pi)
        axr.plot(visplot.index, visplot['pose_visual__rotZ']*180/math.pi)

        axx.set_title('X')
        axx.set_ylabel('Position [m]')
        axy.set_title('Y')
        axy.set_ylabel('Position [m]')
        axr.set_title('rotZ')
        axr.set_ylabel('Orientation [$\degree$]')
        axr.set_xlabel("Time [s]")
        axr.legend(('True pose', 'Visual estimation'), 'bottom left')

        fig2 = plt.figure()
        axxyy = fig2.add_subplot(111)
        axxyy.scatter(ref['pose_estimation__y'], ref['pose_estimation__x'])
        axxyy.plot(vis['pose_visual__y'], vis['pose_visual__x'])
        axxyy.set_xlabel("Y position [m]")
        axxyy.set_ylabel('X position [m]')
        axxyy.legend(('Estimated trajectory','Reference positions'), 'bottom left')
        axxyy.invert_xaxis()

    return (avgerrD,avgerrR,batime)

def main():
    fn = "vanillaBA_robust.bag"
    dirname = '/home/bor/bagfiles/otherpcresults/'
    dirname = '/home/bor/bagfiles/aout/bag/'
    avgerrD,avgerrR,batime = evaluate_benchmark(dirname + fn,True)
    print(fn)
    print(avgerrD)
    print(avgerrR)
    print(batime)
    plt.show()

if __name__ == "__main__":
    main()
