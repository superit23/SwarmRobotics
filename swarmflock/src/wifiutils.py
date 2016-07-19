import math

# Calculates distance between the source of a signal
# and the receiver.
def calcDistance(signalLevelInDb, freqInMHz):
  K = -27.55
  #SNR = -87
  FSPL = ((-1 * K) - (20 * math.log10(freqInMHz)) + math.fabs(signalLevelInDb)) / 20.0
  meters = math.pow(10, FSPL)

  return meters
