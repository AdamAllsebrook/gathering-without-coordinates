# plot data about a run
from matplotlib import pyplot as plt
from joblib import load, dump
import os
import sys

index = int(sys.argv[1])    

dir_path = os.path.dirname(os.path.realpath(__file__))

data = load(dir_path + '/final_results.gz')
print(len(data))

plt.figure(figsize=(12, 8))

time = data[index][:,0]
pos = data[index][:,1]

plt.plot(time, pos)
plt.xlabel('time (s)')
plt.ylabel('average distance (m)')

plt.show()

csv = 'start,end,percent_change\n'
for row in data:
    start = row[0][1]
    end = row[-1][1]

    percent_change = (end - start) / start

    csv += '%f,%f,%f\n' % (start, end, percent_change)

dump(csv, dir_path + '/results.csv')
