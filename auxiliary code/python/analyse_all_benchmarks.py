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

for loc_fn in os.listdir(directory):
    print(loc_fn)
    fn = directory + loc_fn

    avgerrD,avgerrR,batime = td.treat_data(fn,False)
    print(avgerrD)
    print(avgerrR)
    print(batime)
