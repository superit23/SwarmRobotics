import numpy as np
import math

#stackoverflow.com/questions/8930370/where-can-i-find-mad-mean-absolute-deviation-in-scipy
def mad(arr, b=1.4826):
  arr = np.array(arr)
  med = np.median(arr)

  return b * np.median(np.abs(arr - med))


def remOutliers(arr, mad, threshold=3):
  med = np.median(arr)
  return [x for x in arr if math.fabs((x - med) / mad) < threshold]
