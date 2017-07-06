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
directory = '/home/bor/bagfiles/results/rpt2_batol_is_0.5/'
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
    if (robust=="false"):
        #dataf.loc[i,'mm']    = mm
        dataf.loc[i,'batol']    = batol
        dataf.loc[i,'rpt2']     = rpt2
        dataf.loc[i,'rpt3']     = rpt3
        dataf.loc[i,'huber']    = huber
        dataf.loc[i,'rmvcoeff'] = rmvcoeff
        dataf.loc[i,'rmvcst']   = rmvcst
        dataf.loc[i,'avgerrD_prec'] = avgerrD
        dataf.loc[i,'avgerrR_prec'] = avgerrR
        dataf.loc[i,'batime_prec']  = batime
        dataf.loc[i,'ptsmap_prec']  = pts_map
    else :
        #assert(dataf.loc[i-nrow,'mm']    == mm)
        assert(dataf.loc[i-nrow,'batol'] == batol)
        assert(dataf.loc[i-nrow,'rpt2']  == rpt2)
        assert(dataf.loc[i-nrow,'rpt3']  == rpt3)
        assert(dataf.loc[i-nrow,'huber'] == huber)
        dataf.loc[i-nrow,'avgerrD_rob']  = avgerrD
        dataf.loc[i-nrow,'avgerrR_rob']  = avgerrR
        dataf.loc[i-nrow,'batime_rob']   = batime
        dataf.loc[i-nrow,'ptsmap_rob']   = pts_map
    print(avgerrD)
    print(avgerrR)
    print(batime)
dataf.to_csv("/home/bor/bagfiles/rpt2_batol_is_0.5.csv")
print(dataf)
