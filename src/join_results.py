
from joblib import load, dump
import os
import sys

dir_path = os.path.dirname(os.path.realpath(__file__))

d1 = load(dir_path + '/results.gz')
d2 = load(dir_path + '/results.li.gz')

new_data = [
    d1[0],
    d1[1],
    d1[9],
    d1[10],
    d1[11],
    d1[12],
    d1[13],
    d1[14],
    d1[15],
    d1[16],
    d1[17],
    d1[18],
    d1[19],
    d1[20],
    d1[21],
    d2[14],
    d2[15],
    d2[16],
    d2[17],
    d2[18],
]

dump(new_data, dir_path + '/final_results.gz')
