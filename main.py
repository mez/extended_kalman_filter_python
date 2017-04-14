import pandas as pd
from utils import MeasurementPacket
from tracker import Tracker

'''
This is simulating our sensors data stream.

***Note***
    This is not a generator so if your sample log file is huge it'll load it
    all into memory. For now this is acceptable ;)
'''
data_packets = pd.read_csv('./data/sample-laser-radar-measurement-data-1.txt',
                header=None,
                sep='\t',
                names=['x'+str(x) for x in range(9)])

'''
For now we are only tracking one object, hence we only need one Tracker.
If the vehicle seen for example three people and three cars we'd need six trackers.
So you'd ideally have a list of trackers going at once. However, I could be wrong!
'''
tracker = Tracker()

'''
This is the main loop that updates and tracks the objects.
EKF style!
'''
n = 0
for _ , raw_measurement_packet in data_packets.iterrows():
    measurement_packet = MeasurementPacket(raw_measurement_packet)
    print(measurement_packet)
    tracker.process_measurement(measurement_packet)
    n+=1

    if n == 10:
        break
