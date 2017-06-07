import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td

fnames = ['result_benchmark_midpointtriang','result_benchmark_dlttriang','result_benchmark_optimaltriang']


for fn in os.listdir('/home/bor/bagfiles/otherpcresults/'):
#for fn in fnames:
    ffnn = '/home/bor/bagfiles/otherpcresults/' + fn

    avgerrD,avgerrR,batime = td.treat_data(ffnn)
    print(fn)
    print(avgerrD)
    print(avgerrR)
    print(batime)
