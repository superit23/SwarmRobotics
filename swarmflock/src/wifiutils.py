import math
import numpy as np

# Calculates distance between the source of a signal
# and the receiver.
def calcDistance(signalLevelInDb, freqInMHz):
  K = -27.55
  #SNR = -87
  FSPL = ((-1 * K) - (20 * math.log10(freqInMHz)) + math.fabs(signalLevelInDb)) / 20.0
  meters = math.pow(10, FSPL)

  return meters

# stackoverflow.com/questions/9747227/2d-trilateration
def trilaterate(p1, r1, p2, r2, p3, r3):
  p21Diff = p2 - p1
  p31Diff = p3 - p1
  d = np.linalg.norm(p21Diff)
  ex = p21Diff / d
  i = np.dot(ex, p31Diff)
  temp = p31Diff - i * ex
  ey = temp / np.linalg.norm(temp)

  j = np.dot(ey, p31Diff)

  x = (r1 ** 2 - r2 ** 2 + d ** 2) / (2 * d)
  y = (r1 ** 2 - r3 ** 2 + i ** 2 + j ** 2) / (2 * j) - (i * x / j)

  return [x, y]


def test():
  p1 = np.array([0,0])
  p2 = np.array([2,0])
  p3 = np.array([1,1])

  tx = np.array([1.754, 0.921])

  r1 = np.linalg.norm(p1 - tx)
  r2 = np.linalg.norm(p2 - tx)
  r3 = np.linalg.norm(p3 - tx)

  print tx
  return trilaterate(p1, r1, p2, r2, p3, r3)
