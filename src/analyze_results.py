#!/usr/bin/env python

import sys
import os
import cPickle
import matplotlib.pyplot as plt

if len(sys.argv) == 1:
    raise ValueError('Type the name of a bag file to inspect')

bag_file = sys.argv[1]
root = os.path.expanduser('~')
path = os.path.join(root, bag_file)
with open(path, 'rb') as fh:
    results = cPickle.load(fh)

# Example of how to analyze - your analysis will be different!

times = []
force_x = []
for wrench in results['wrench']:
    times.append(wrench.header.stamp.to_sec())
    force_x.append(wrench.wrench.force.x)

plt.plot(times, force_x)
plt.show()