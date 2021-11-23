#!/usr/bin/env python3

# Example computing time diff interval 

import numpy as np
import matplotlib.pyplot as plt

def main():
    # read timestamps from text file
    lines = []
    with open('timediff.txt') as f:
        lines = f.readlines()

    timestmp = []
    for line in lines:
        timestmp.append(line.split("\n")[0])
        
    timestmp_array = np.asarray(timestmp, dtype=np.float64)
    framecount = np.arange(0, timestmp_array.shape[0])

    # jitter from consecutive timestamps
    timediff_array = np.diff(timestmp_array)*1e3
    timediff_array_second_order = timediff_array-np.median(timediff_array)

    begin = 0
    end = 100
    fig = plt.figure(figsize=(9, 3))
    # plt.plot(framecount[begin:end], timediff_array[begin:end])
    plt.plot(framecount[begin:end], timediff_array_second_order[begin:end])
    plt.xlabel('frame count')
    plt.ylabel('timestamp diff in  msec')
    plt.show()
    # fig.savefig('comparison.png', dpi=200) 
    
if __name__ == '__main__':
    # sys.argv[] #pass arguments if given and whatnot
    main()