#!/usr/bin/python3

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plot
import numpy as np
import sys
from optparse import OptionParser

parser = OptionParser()
parser.add_option("--result", dest="result",
                  default="", help="The filename that contains the SLAM result.")
parser.add_option("--output_file", dest="outputFile",
                  default="", help="The output file.")

(options, args) = parser.parse_args()

# Read the original and optimized poses files.
results = None
if options.result != '':
  results = np.genfromtxt(options.result, dtype=str, usecols = (0, 2, 3))

poses = []
landmarks = []
for line in results:
    if line[0] == 'Pose':
        poses.append(line)
    else:
        landmarks.append(line)

poses = np.vstack(poses)
landmarks = np.vstack(landmarks)


# Plots the results for the specified poses.
figure = plot.figure()

poses = np.asarray(poses[:,1:3]).astype(np.float)
landmarks = np.asarray(landmarks[:,1:3]).astype(np.float)
#  print(b.astype(np.float))
if results is not None:

 plot.plot(poses[:, 0], poses[:, 1],
           '*', alpha=0.5, color="green")
 plot.plot(landmarks[:, 0], landmarks[:, 1],
           '*', alpha=0.5, color="red")
#

plot.axis('equal')

plot.title('Trajectories: GT (Green)')

plot.show()
