import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td

nbin = 50
pd.set_option('display.max_columns', None)
pd.set_option('display.width', 1000)
directory = '/home/bor/bagfiles/results/cost_per_obs/'
filelist  = os.listdir(directory)
filelist.sort()
for i,loc_fn in enumerate(filelist):
    df = pd.read_csv(directory + loc_fn)
    plt.figure(num=loc_fn)
    df['cost_per_obs'].plot.box()
plt.show()
