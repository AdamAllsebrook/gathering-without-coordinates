from matplotlib import pyplot as plt
from joblib import load
import os
import sys

"""
open
    random
    full
closed
    random
    full
"""

if sys.argv == 'diff':
    for i in range(0, 12, 3):
        diff1 = data[i][-1] - diff[i][0]
        diff2 = data[i+1][-1] - diff[i+1][0]
        diff3 = data[i+2][-1] - diff[i+2][0]
        print(diff1, diff2, diff3, (diff1+diff2+diff3)/3)
else:

    index = int(sys.argv[1])    

    dir_path = os.path.dirname(os.path.realpath(__file__))

    data = load(dir_path + '/all_results.gz')

    print(len(data))

    plt.figure(figsize=(12, 8))

    # if data[index].shape[0] > 0:
    #     print(data[index][0])
    #     print(data[index][1])
    #     print(data[index][2])
    #     print(data[index][3])
    #     print(data[index][4])

    time = data[index][:,0]
    pos = data[index][:,1]

    plt.plot(time, pos)
    plt.xlabel('time (s)')
    plt.ylabel('average distance (m)')

    plt.show()
