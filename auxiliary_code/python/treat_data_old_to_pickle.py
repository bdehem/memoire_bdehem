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
    infile = file_in
    ref_pose_x    = 0
    ref_pose_y    = 0
    ref_pose_z    = 0
    ref_pose_rotZ = 0
    errors = np.zeros((4,5))
    counts = np.zeros((4,5))
    batimes = np.zeros((3,2))
    pts_map = np.zeros((3))
    repaired = False

    vis = rosbag_pandas.bag_to_dataframe(infile,include="/pose_visual")
    man = rosbag_pandas.bag_to_dataframe(infile,include="/pose_estimation")
    ind = -1
    starttime = -1
    k = 0
    q = 0

    for topic, msg, t in rosbag.Bag(infile).read_messages():
        if starttime == -1:
            starttime = t.secs
        if topic == "/pose_estimation":
            pose_is_manual = msg.x==0 or msg.x==-0.09 or msg.x==1.12 or msg.x==0.80 or msg.x==0.57 or msg.y==0.545
            new_man_pose   = pose_is_manual and (msg.x!=ref_pose_x or msg.y!=ref_pose_y or msg.z!=ref_pose_z or msg.rotZ!=ref_pose_rotZ)
            new_ref_pose   = new_man_pose and msg.x != 0
            #if new_ref_pose:
                #print(t.secs - starttime)
            ind += new_ref_pose
            if new_man_pose:
                ref_pose_x    = msg.x
                ref_pose_y    = msg.y
                ref_pose_z    = msg.z
                ref_pose_rotZ = msg.rotZ
            if not repaired and msg.header.stamp.secs > 1502185200:
                ref_pose_y = 0.545
                repaired = True
            if (ref_pose_rotZ == 0 and ref_pose_x == 0 and ref_pose_y == 0 and ref_pose_z == 0) or (
            ref_pose_y == 0.545 or ref_pose_z ==0.645):
                man.iloc[q,4]  = float('nan')
                man.iloc[q,6]  = float('nan')
                man.iloc[q,8]  = float('nan')
                man.iloc[q,10] = float('nan')
            else :
                man.iloc[q,4]  = ref_pose_rotZ
                man.iloc[q,6]  = ref_pose_x
                man.iloc[q,8]  = ref_pose_y
                man.iloc[q,10] = ref_pose_z
            q+=1
        if (topic == "/pose_visual" and ref_pose_x != 0):
            errX    = ref_pose_x    - msg.x
            errY    = ref_pose_y    - msg.y
            errD    = (errX**2 + errY**2)**(0.5)
            errrotZ = ref_pose_rotZ - msg.rotZ
            errors[0,ind] += abs(errX)
            errors[1,ind] += abs(errY)
            errors[2,ind] += abs(errD)
            errors[3,ind] += abs(errrotZ)
            counts[:,ind] += 1
        if topic == "/benchmark":

            batimes[k,0] = msg.BA_times_pass1
            batimes[k,1] = msg.BA_times_pass2
            pts_map[k] = msg.pts_map
            k+=1
    #print(batimes)
    batime  = sum(sum(batimes))
    avgerrD = (sum(errors[2,])/sum(counts[2,]))
    avgerrR = (sum(errors[3,])/sum(counts[3,]))*180/math.pi
    avgerr = sum(sum(errors))/sum(sum(counts))
    vis = vis.set_index((vis.index-man.index[0]).total_seconds())
    man = man.set_index((man.index-man.index[0]).total_seconds())
    man.to_pickle('reference_poses.pkl')
    if do_plot:
        f, (axx, axy, axr) = plt.subplots(3, sharex=True)
        axx.plot(man.index, man['pose_estimation__x'])
        axx.plot(vis.index, vis['pose_visual__x'])
        axy.plot(man.index, man['pose_estimation__y'])
        axy.plot(vis.index, vis['pose_visual__y'])
        axr.plot(man.index, man['pose_estimation__rotZ'])
        axr.plot(vis.index, vis['pose_visual__rotZ'])

        axx.set_title('X')
        axy.set_title('Y')
        axr.set_title('rotZ')
    #(robust,mm,batol,rpt2,rpt3,huber,rmvcoeff,rmvcst) = get_params_from_fn(file_in)
    #return (avgerrD,avgerrR,batime)
    robust = 0
    mm = 0
    batol = 0
    rpt2 = 0
    rpt3 = 0
    huber = 0
    rmvcoeff = 0
    rmvcst = 0

    #return (robust,mm,batol,rpt2,rpt3,huber,rmvcoeff,rmvcst,avgerrD,avgerrR,batime,pts_map[-1])
    return (avgerrD,avgerrR,batime)


def get_params_from_fn(file_in):
    arglist1 = file_in.split('/')
    file_loc = arglist1[-1]
    arglist = file_loc.replace('.ba','_').split('_')
    robust  = arglist[2]
    mm      = arglist[4]
    batol   = arglist[6]
    rpt2    = arglist[8]
    rpt3    = arglist[10]
    rmvcst   = arglist[14] if len(arglist) > 14 else 0
    rmvcoeff = arglist[16] if len(arglist) > 16 else -1
    huber    = arglist[18] if len(arglist) > 18 else "1.0"
    return (robust,mm,batol,rpt2,rpt3,huber,rmvcoeff,rmvcst)


def main():
    fn = "vanillaBA_norobust.bag"
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
