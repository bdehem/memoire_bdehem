import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td


directory = '/home/bor/bagfiles/results/thresh_rpt/'
filelist  = os.listdir(directory)
filelist.sort()  #list is now alphabetical, so nonrobust are all first
nrow = len(filelist)/2
columns = ['mm','batol','rpt2','rpt3','avgerrD_prec','avgerrR_prec','batime_prec','avgerrD_rob','avgerrR_rob','batime_rob']
index   = 1:nrow
dataf = pd.DataFrame(index=index, columns=columns)

for i,loc_fn in enumerate(filelist):
    print(loc_fn)
    fn = directory + loc_fn

    (robust,mm,batol,rpt2,rpt3,avgerrD,avgerrR,batime) = td.treat_data(fn,False)
    if not robust:
        dataf[i,'mm']    = mm
        dataf[i,'batol'] = batol
        dataf[i,'rpt2']  = rpt2
        dataf[i,'rpt3']  = rpt3
        dataf[i,'avgerrD_prec'] = avgerrD
        dataf[i,'avgerrR_prec'] = avgerrR
        dataf[i,'batime_prec']  = batime
    else :
        dataf[i-nrow,'avgerrD_rob'] = avgerrD
        dataf[i-nrow,'avgerrR_rob'] = avgerrR
        dataf[i-nrow,'batime_rob']  = batime
    print(avgerrD)
    print(avgerrR)
    print(batime)
