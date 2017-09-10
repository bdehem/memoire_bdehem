import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td

# This file is a utility script to vall evaluate_benchmark on all files within a folder

pd.set_option('display.max_columns', None)
pd.set_option('display.width', 1000)
pd.set_option('display.max_rows', 1000)
directory = '/home/bor/bagfiles/morerealistic/'
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
    filetype = loc_fn.split('.')[-1]
    if filetype=='bag':
        (avgerrD,avgerrR,batime) = td.treat_data(fn,False)
        print(avgerrD)
        print(avgerrR)
        print(batime)
