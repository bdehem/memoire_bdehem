import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td

pd.set_option('display.max_columns', None)
pd.set_option('display.width', 1000)
pd.set_option('display.max_rows', 1000)
directory = '/home/bor/bagfiles/res/'
directory = '/home/bor/bagfiles/results/stable_kfs/rpt/'
directory = '/home/bor/bagfiles/results/stable_kfs/batol/'
directory = '/home/bor/bagfiles/results/triangulation_and_ba/'
filelist  = os.listdir(directory)
filelist.sort()  #list is now alphabetical, so nonrobust are all first
nrow = len(filelist)/2
columns = ['batol','huber','rpt2','rpt3','rmvcoeff','rmvcst','avgerrD_prec','avgerrR_prec','batime_prec','ptsmap_prec','avgerrD_rob','avgerrR_rob','batime_rob','ptsmap_rob']
ncol = len(columns)
index   = range(0,nrow)
dataf = pd.DataFrame(np.random.randn(nrow,ncol),index=index, columns=columns)
for i,loc_fn in enumerate(filelist):
    print(loc_fn)
    fn = directory + loc_fn
    print(i)

    (robust,mm,batol,rpt2,rpt3,huber,rmvcoeff,rmvcst,avgerrD,avgerrR,batime,pts_map) = td.treat_data(fn,False)
    if (robust!="true"):
        #dataf['mm'].iloc[[i]]    = mm
        dataf['batol'].iloc[[i]] = batol
        dataf['rpt2'].iloc[[i]]  = rpt2
        dataf['rpt3'].iloc[[i]]  = rpt3
        dataf['huber'].iloc[[i]]  = huber
        dataf['rmvcoeff'].iloc[[i]]  = rmvcoeff
        dataf['rmvcst'].iloc[[i]]  = rmvcst
        dataf['avgerrD_prec'].iloc[[i]] = avgerrD
        dataf['avgerrR_prec'].iloc[[i]] = avgerrR
        dataf['batime_prec'].iloc[[i]]  = batime
        dataf['ptsmap_prec'].iloc[[i]]  = pts_map
    else :
        #assert(dataf['mm'].iloc[i-nrow]    == mm)
        assert(dataf['batol'].iloc[i-nrow]   == batol)
        assert(dataf['rpt2'].iloc[i-nrow]    == rpt2)
        assert(dataf['rpt3'].iloc[i-nrow]    == rpt3)
        assert(dataf['huber'].iloc[i-nrow] == huber)
        dataf['avgerrD_rob'].iloc[[i-nrow]] = avgerrD
        dataf['avgerrR_rob'].iloc[[i-nrow]] = avgerrR
        dataf['batime_rob'].iloc[[i-nrow]]  = batime
        dataf['ptsmap_rob'].iloc[[i-nrow]]  = pts_map
    print(avgerrD)
    print(avgerrR)
    print(batime)
dataf.to_csv("/home/bor/bagfiles/triangulation_and_ba.csv")
print(dataf)
