from matplotlib import pyplot as plt
from joblib import load
import os
import sys
import numpy as np

index = int(sys.argv[1])    

<<<<<<< Updated upstream
dir_path = os.path.dirname(os.path.realpath(__file__))
=======
def s(a):
    return np.sqrt(a[0] **2 + a[1] ** 2)

if sys.argv[1] == 'diff':

    dir_path = os.path.dirname(os.path.realpath(__file__))

    data = load(dir_path + '/all_results.gz')

    for i in range(0, 12, 3):
        diff1 = data[i][-1] - data[i][0]
        diff2 = data[i+1][-1] - data[i+1][0]
        diff3 = data[i+2][-1] - data[i+2][0]
        print(s(diff1), s(diff2), s(diff3), s((diff1+diff2+diff3))/3)
else:
>>>>>>> Stashed changes

data = load(dir_path + '/results.gz')

print(len(data))

plt.figure(figsize=(12, 8))

time = data[index][:,0]
pos = data[index][:,1]

plt.plot(time, pos)

plt.show()