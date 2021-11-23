#!/usr/bin/env python3

# Example computing time diff interval 

import numpy as np
import matplotlib.pyplot as plt

def main():
    # read timestamps no load from text file
    lines_noload = []
    with open('examples/timediff.out') as f:
        lines_noload = f.readlines()
    timestmp_noload = []
    for line in lines_noload:
        timestmp_noload.append(line.split("\n")[0])


    # read timestamps with load from text file
    lines_load = []
    with open('examples/timediff_all.out') as f:
        lines_load = f.readlines()
    timestmp_load = []
    for line in lines_load:
        timestmp_load.append(line.split("\n")[0])


    timestmp_array_noload = np.asarray(timestmp_noload, dtype=np.float64)
    framecount_noload = np.arange(0, timestmp_array_noload.shape[0])
    timestmp_array_load = np.asarray(timestmp_load, dtype=np.float64)
    framecount_load = np.arange(0, timestmp_array_load.shape[0])

    # jitter from consecutive timestamps
    timediff_array_noload = np.diff(timestmp_array_load)*1e3
    timediff_array_second_order = timediff_array_noload-np.median(timediff_array_noload)

    # print to image
    begin = 0
    end = 1000
    fig = plt.figure(figsize=(12, 9))
    plt.subplot(2, 1, 1)
    plt.plot(framecount_noload[begin:end], timestmp_array_noload[begin:end])
    plt.xlabel('frame count')
    plt.ylabel('timestamp diff in mu sec')
    plt.title('ifm3d data retrieval and ROS 2 publishing duration - no load, single subscriber')
    plt.subplot(2, 1, 2)
    plt.plot(framecount_load[begin:end], timestmp_array_load[begin:end])
    plt.xlabel('frame count')
    plt.ylabel('timestamp diff in mu sec')
    plt.title('ifm3d data retrieval and ROS 2 publishing duration - large load, all topics subscribed')
    # plt.show()
    fig.savefig('examples/publishtime.png', dpi=200) 
    
if __name__ == '__main__':
    # sys.argv[] #pass arguments if given and whatnot
    main()