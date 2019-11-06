#import future
import rosbag
import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
#%matplotlib notebook

#pd.set_option('display.max_columns', 30)

pathList =[r"/home/qmind/Data-2019-10-29/gMitTile_s05_t1.bag",
r"/home/qmind/Data-2019-10-29/gMitTile_s05_t2.bag",
r"/home/qmind/Data-2019-10-29/gMitTile_s10_t1.bag",
r"/home/qmind/Data-2019-10-29/gMitTile_s10_t2.bag",
r"/home/qmind/Data-2019-10-29/gMitTile_s15_t1.bag",
r"/home/qmind/Data-2019-10-29/gMitTile_s15_t2.bag",
r"/home/qmind/Data-2019-10-29/gMitTile_s20_t1.bag",
r"/home/qmind/Data-2019-10-29/gMitTile_s20_t2.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s05_t1.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s05_t2.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s10_t1.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s10_t2.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s15_t1.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s15_t2.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s20_t1.bag",
r"/home/qmind/Data-2019-10-29/gTurf_s20_t2.bag"]

for path in pathList:

    #bagPath = r"/home/qmind/Data-2019-10-29/gMitTile_s10_t1.bag"
    bagPath = path
    csvSavePath = bagPath[:-4]+".csv"

    # create two dataframes, one for IMU data and one for Odometry data


    bag = rosbag.Bag(bagPath)
    odomDf = pd.DataFrame(columns = ['Seq', 'Secs', 'Nanosecs', 
                                    'OdomPosX', 'OdomPosY', 'OdomPosZ', 
                                    'OdomOrientX', 'OdomOrientY', 'OdomOrientZ', 'OdomOrientW', 
                                    'OdomLinX', 'OdomLinY', 'OdomLinZ', 
                                    'OdomAngX', 'OdomAngY', 'OdomAngZ'])

    imuDf = pd.DataFrame(columns = ['Seq', 'Secs', 'Nanosecs', 
                                    'ImuOrientX', 'ImuOrientY', 'ImuOrientZ', 'ImuOrientW', 
                                    'ImuAngVelX', 'ImuAngVelY', 'ImuAngVelZ', 
                                    'ImuAccelX', 'ImuAccelY', 'ImuAccelZ'] )
    count = 0
    imuCount = 0
    odomCount = 0
    for topic, msg, t in bag.read_messages():

        if topic == "/imu":
            imuDf.loc[imuCount] = [msg.header.seq, t.secs, t.nsecs, 
                                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, 
                                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, 
                                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            imuCount+=1
        
            
        if topic == '/odom':
            odomDf.loc[odomCount] = [msg.header.seq, t.secs, t.nsecs, msg.pose.pose.position.x, msg.pose.pose.position.y, 
                                msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w, msg.twist.twist.linear.x, 
                                msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.x, 
                                msg.twist.twist.angular.y, msg.twist.twist.angular.z]
            odomCount+=1
        count+=1
                
    print("Total data points: {}".format(count))
    print("IMU data points: {}".format(imuCount))
    print("Odometry data points: {}".format(odomCount))
    bag.close()

    startTime = min(imuDf.Secs[0], odomDf.Secs[0])
    # start at 0 seconds
    odomDf.Secs = odomDf.Secs - startTime
    imuDf.Secs = imuDf.Secs - startTime
    #make a time column from secs and nanosecs
    odomDf['Time'] = odomDf.Secs + odomDf.Nanosecs/10**(9)
    imuDf['Time'] = imuDf.Secs + imuDf.Nanosecs/10**(9)
    # move time column to front
    odomDf = odomDf[[odomDf.columns.tolist()[-1]] + odomDf.columns.tolist()[:-1]]
    imuDf = imuDf[[imuDf.columns.tolist()[-1]] + imuDf.columns.tolist()[:-1]]
    # sanity check for errors
    print(type(odomDf.Nanosecs[0]))
    print(odomDf.Time[1] - odomDf.Time[0])
    print((odomDf.Nanosecs[1]-odomDf.Nanosecs[0])/10**9)

    # remove un-needed cols
    odomDf = odomDf.drop(columns = ['Secs', 'Nanosecs', 'Seq'])
    imuDf = imuDf.drop(columns = ['Secs', 'Nanosecs', 'Seq'])

    # zero start time again because nanoseconds werent zeroed the first time
    startTime2 = min(imuDf.Time[0], odomDf.Time[0])
    odomDf.Time = odomDf.Time - startTime2
    imuDf.Time = imuDf.Time - startTime2

    odomDf['Sensor'] = 'Odom'
    imuDf['Sensor'] = 'Imu'

    allDataDf = pd.concat([odomDf, imuDf], axis=0, sort=False)
    allDataDf = allDataDf.sort_values('Time').reset_index()
    allDataDf = allDataDf.drop(columns='index')

    print(allDataDf.Time[1])


    allDataDf.to_csv(csvSavePath)
    checkDf = pd.read_csv(csvSavePath)
    print(checkDf.shape)