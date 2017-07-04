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
cumul = False
for i,loc_fn in enumerate(filelist):
    df = pd.read_csv(directory + loc_fn)
    npt = len(df['cost_per_obs'])
    nkf = loc_fn.split('_')[-1].split('.')[0]
    total_cost = pd.DataFrame(df['cost_per_obs'].values*df['nobs'].values)
    total_nobs = df['nobs'].sum()
    frac_cost  = pd.DataFrame(total_cost.values/total_cost.sum().values[0])
    fc_sort = frac_cost.sort_values(by=0,axis=0,ascending=False)
    cum_fc = fc_sort.cumsum()
    if nkf == "2":
        print(cum_fc)
    cum_fc.index = np.linspace(0,1,npt)
    #plt.figure(num=loc_fn)
    #frac_cost.hist(cumulative=cumul,normed=cumul,bins=100)
    save_name = 'err_repartition_' + nkf + '.eps'
    save_dir = '/home/bor/catkin_ws/src/boris_drone/report/images/'
    cum_fc.plot(legend=False,logx=True)
    plt.axis([0, 1, 0, 1])
    plt.xlabel('Fraction of points')
    plt.ylabel('Fraction of total error captured')
    plt.savefig(save_dir+save_name)
plt.show()
