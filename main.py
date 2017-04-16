import pandas as pd
from utils import MeasurementPacket, calculate_rmse, passing_rmse
from tracker import Tracker

def runTracker(data_path):
    '''
    This is simulating our sensors data stream.

    ***Note***
        This is not a generator so if your sample log file is huge it'll load it
        all into memory. For now this is acceptable ;)
    '''
    data_packets = pd.read_csv(data_path,
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

    estimations = []
    measurements = []

    for _ , raw_measurement_packet in data_packets.iterrows():
        measurement_packet = MeasurementPacket(raw_measurement_packet)
        measurements.append(measurement_packet)
        tracker.process_measurement(measurement_packet)
        estimations.append(tracker.state)

    return estimations, measurements


if __name__ == '__main__':
    print("Started Tracker for sample-laser-radar-measurement-data-1.txt")
    estimations, measurements = runTracker('./data/sample-laser-radar-measurement-data-1.txt')
    print("estimations count: ", len(estimations))
    print("measurements count: ", len(measurements))

    rmse = calculate_rmse(estimations, [m.ground_truth for m in measurements]).flatten()
    print("RMSE: ", rmse)
    #check if we passed metric for data1 log file.
    passing_rmse(rmse, [0.08, 0.08, 0.60, 0.60])


    print("\n\nStarted Tracker for sample-laser-radar-measurement-data-2.txt")
    estimations, measurements = runTracker('./data/sample-laser-radar-measurement-data-2.txt')
    print("estimations count: ", len(estimations))
    print("measurements count: ", len(measurements))

    rmse = calculate_rmse(estimations, [m.ground_truth for m in measurements]).flatten()
    print("RMSE: ", rmse)
    #check if we passed metric for data2 log file.
    passing_rmse(rmse, [0.20, 0.20, .50, .85])
