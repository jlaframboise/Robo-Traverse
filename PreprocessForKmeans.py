import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import os
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

dataFolder = r"C:\Users\jaker\Documents\Experiment3Data-2019-11-21"

dataFiles = [
    r"gMitTile_s15_t8.csv",
    r"gMitTile_s15_t9.csv",
    r"gMitTile_s15_t10.csv",
    r"gTurf_s15_t3.csv",
    r"gTurf_s15_t4.csv",
    r"gTurf_s15_t5.csv",
    r"gTurf_s15_t6.csv",
    r"gTurf_s15_t7.csv",
    r"gTurf_s15_t8.csv",
    r"gTurf_s15_t9.csv",
    r"gTurf_s15_t10.csv",
    r"gArcTile_s15_t3.csv",
    r"gArcTile_s15_t4.csv",
    r"gArcTile_s15_t5.csv",
    r"gArcTile_s15_t6.csv",
    r"gArcTile_s15_t7.csv",
    r"gArcTile_s15_t8.csv",
    r"gArcTile_s15_t9.csv",
    r"gArcTile_s15_t10.csv",
    r"gCarp_s15_t3.csv",
    r"gCarp_s15_t4.csv",
    r"gCarp_s15_t5.csv",
    r"gCarp_s15_t6.csv",
    r"gCarp_s15_t7.csv",
    r"gCarp_s15_t8.csv",
    r"gCarp_s15_t9.csv",
    r"gCarp_s15_t10.csv",
    r"gMitTile_s15_t3.csv",
    r"gMitTile_s15_t4.csv",
    r"gMitTile_s15_t5.csv",
    r"gMitTile_s15_t6.csv",
    r"gMitTile_s15_t7.csv"
]

savePath = "Data-32Series-Delta30-Squared.csv"

for i in range(len(dataFiles)):
    terrain = dataFiles[i].split('_')[0][1:]
    speed = dataFiles[i].split('_s')[1][:2]
    trial = dataFiles[i].split('_t')[1][0]

    df = pd.read_csv(os.path.join(dataFolder, dataFiles[i]))
    df = df.rename(columns={'Unnamed: 0': 'Seq'})
    df = df.set_index('Seq')

    #print(df.isnull().sum().sum())
    df = df.interpolate(method='polynomial', order=1)
    #print(df.isnull().sum().sum())

    df = df.dropna()
    #print(df.isnull().sum().sum())
    df = df.reset_index().drop(columns=['Seq'])

    df = df.drop(columns=['OdomPosZ', 'OdomOrientX', 'OdomOrientY', 'OdomLinY', 'OdomLinZ', 'OdomAngX', 'OdomAngY'])

    dList = range(1, 302, 10)
    # dList = [2,4,8,16,32,64]
    for col in df.columns.tolist():
        if col!='Sensor':
            for d in dList:
                df[col+'Delta{}'.format(d)] = df[col].diff(d)
                df[col+'Delta{}Squared'.format(d)] = df[col+'Delta{}'.format(d)]**2
                #print('Added ' + col +'Delta{}'.format(d))
        else:
            pass

    df = df.iloc[max(dList):].reset_index().drop(columns=['index'])
    df = df.drop(columns=['Sensor', 'Time'])
    df['Speed']=speed
    df['Terrain']=terrain
    df['Trial']=trial

    if i==0:
        mainDf = df.copy(deep=True)
    else:
        mainDf = pd.concat([mainDf, df], axis=0, sort=False)
    print("Added {} of size {} to mainDf. ".format(dataFiles[i], df.shape))
    print("Data series completed: {}/{}".format(i+1,len(dataFiles)))
    print("MainDf is now size {}".format(mainDf.shape))
    print('\n')

fullSavePath = os.path.join(dataFolder, savePath)
mainDf.to_csv(fullSavePath)
print('\n')
print("Saved mainDf to {}".format(fullSavePath))
print("MainDF has size {}".format(mainDf.shape))
