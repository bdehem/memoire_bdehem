import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import treat_data as td



for fn in os.listdir('/home/bor/bagfiles/benchmark_point_threshold/toanalyze/'):
    avgerr,batime = td.treat_data(fn)
    print(fn)
    print(avgerr)
    print(batime)
