import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas


def show_result(file_in):
    infile = '/home/bor/bagfiles/benchmark_point_threshold/' + file_in
    i = 0
    w = 40
    h = 3
    ba_data = [[0 for x in range(w)] for y in range(h)]
    for topic, msg, t in rosbag.Bag(infile).read_messages():
        if topic == "/benchmark":
            ba_data[i][0] = msg.pts_map
            for j in range(0,4) :
                ba_data[i][1+9*j+0] = msg.keyframes_ID[j]          if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+1] = msg.n_pts_keyframe[j]        if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+2] = msg.n_mapped_pts_keyframe[j] if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+3] = msg.keyframes_pose[j].x      if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+4] = msg.keyframes_pose[j].y      if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+5] = msg.keyframes_pose[j].z      if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+6] = msg.keyframes_pose[j].rotX   if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+7] = msg.keyframes_pose[j].rotY   if len(msg.keyframes_pose) > j else float('NaN')
                ba_data[i][1+9*j+8] = msg.keyframes_pose[j].rotZ   if len(msg.keyframes_pose) > j else float('NaN')
            ba_data[i][37] = msg.BA_times_pass1[i]
            ba_data[i][38] = msg.BA_times_pass2[i]
            ba_data[i][39] = msg.BA_times_pass1[i] + msg.BA_times_pass2[i]
            i+=1
    ba = pd.DataFrame(ba_data,columns=['n_pts_map',
    'ID_kf1','n_pts_kf1','n_mapped_kf1','x_kf1','y_kf1','z_kf1','rot_x_kf1','rot_y_kf1','rot_z_kf1',
    'ID_kf2','n_pts_kf2','n_mapped_kf2','x_kf2','y_kf2','z_kf2','rot_x_kf2','rot_y_kf2','rot_z_kf2',
    'ID_kf3','n_pts_kf3','n_mapped_kf3','x_kf3','y_kf3','z_kf3','rot_x_kf3','rot_y_kf3','rot_z_kf3',
    'ID_kf4','n_pts_kf4','n_mapped_kf4','x_kf4','y_kf4','z_kf4','rot_x_kf4','rot_y_kf4','rot_z_kf4','t1','t2','ttot'])
    pd.set_option('display.max_columns', 40)
    print(ba)
    pd.reset_option('display.max_columns')

    vis = rosbag_pandas.bag_to_dataframe(infile,include="/pose_visual")
    man = rosbag_pandas.bag_to_dataframe(infile,include="/pose_estimation")

    f, (axx, axy, axz, axr) = plt.subplots(4, sharex=True)
    axx.plot(man.index, man['pose_estimation__x'])
    axx.plot(vis.index, vis['pose_visual__x'])
    axy.plot(man.index, man['pose_estimation__y'])
    axy.plot(vis.index, vis['pose_visual__y'])
    axz.plot(man.index, man['pose_estimation__z'])
    axz.plot(vis.index, vis['pose_visual__z'])
    axr.plot(man.index, man['pose_estimation__rotZ'])
    axr.plot(vis.index, vis['pose_visual__rotZ'])
    axx.set_title('X')
    axy.set_title('Y')
    axz.set_title('Z')
    axr.set_title('rotZ')
    return

show_result('nn__result_benchmark_precision_thresh_250_maxmatches_-1_noBA_false_naivetriang_false_robust_false_bundleadjustmenttol_0.00001.bag')
show_result('__result_benchmark_precision_thresh_250_maxmatches_-1_noBA_false_naivetriang_false_robust_false_bundleadjustmenttol_0.00001.bag')
plt.show()
