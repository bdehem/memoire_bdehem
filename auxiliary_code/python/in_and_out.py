import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag
import subprocess, yaml
import rosbag_pandas
import sys
import os
import math
import treat_data as td

pd.set_option('display.max_columns', None)
pd.set_option('display.width', 1000)
df = pd.read_csv('/home/bor/catkin_ws/src/boris_drone/auxiliary_code/results/in_and_outliers.csv')
df['ratio'] = df['times_in'] / df['times_out']
df['ratio'] = df['ratio'].replace(np.inf,250).replace(0,0.004)
df['logratio'] = np.log(df['ratio'])

df['logratio'].plot.hist(alpha=0.5)
plt.show()

print(df)
