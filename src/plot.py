from matplotlib import pyplot as plt
from joblib import load
import os
import sys

index = int(sys.argv[1])    

dir_path = os.path.dirname(os.path.realpath(__file__))

data = load(dir_path + '/results.gz')

plt.figure(figsize=(12, 8))

time = data[index][:,0]
pos = data[index][:,1]

plt.plot(time, pos)

plt.show()