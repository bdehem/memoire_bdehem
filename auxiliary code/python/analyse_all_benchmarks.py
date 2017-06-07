import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td


for fn in os.listdir('/home/bor/bagfiles/otherpcresults/'):
    print(fn)
    ffnn = '/home/bor/bagfiles/otherpcresults/' + fn

    avgerrD,avgerrR,batime = td.treat_data(ffnn,False)
    print(avgerrD)
    print(avgerrR)
    print(batime)
