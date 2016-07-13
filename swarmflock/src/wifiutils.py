import math

# Calculates distance between the source of a signal
# and the receiver.
def calcDistance(signalLevelInDb, freqInMHz):
  K = -27.55
  #SNR = -87
  FSPL = (20 * math.log10(freqInMHz) + K + math.fabs(signalLevelInDb)) / 20
  meters = math.pow(10, FSPL)

  return meters
