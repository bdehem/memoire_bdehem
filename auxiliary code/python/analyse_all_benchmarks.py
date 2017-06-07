import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td

fnames = ['result_benchmark_noBA_naivetriang','result_benchmark_noBA_optimaltriang']
fnames = ['result_selection_thresh_250_maxmatches_-1_noBA_true_triang_falsefalse_robust_false_bundleadjustmenttol_0.000001']


#for fn in os.listdir('/home/bor/bagfiles/otherpcresults/'):
for fn in fnames:
    ffnn = '/home/bor/bagfiles/otherpcresults/' + fn + '.bag'
    ffnn = '/home/bor/bagfiles/benchmark_point_threshold/' + fn + '.bag'

    avgerrD,avgerrR,batime = td.treat_data(ffnn)
    print(fn)
    print(avgerrD)
    print(avgerrR)
    print(batime)
