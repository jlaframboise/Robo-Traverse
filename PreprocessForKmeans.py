import pandas as pd
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import os
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

dataFolder = r"C:\Users\jaker\Documents\RoboData-Experiment1"

dataFiles = [
    r"gTurf_s15_t2.csv",
    r"gTurf_s20_t1.csv",
    r"gTurf_s20_t2.csv",
    r"gMitTile_s05_t1.csv",
    r"gMitTile_s05_t2.csv",
    r"gMitTile_s10_t1.csv",
    r"gMitTile_s10_t2.csv",
    r"gMitTile_s15_t1.csv",
    r"gMitTile_s15_t2.csv",
    r"gMitTile_s20_t1.csv",
    r"gMitTile_s20_t2.csv",
    r"gTurf_s05_t1.csv",
    r"gTurf_s05_t2.csv",
    r"gTurf_s10_t1.csv",
    r"gTurf_s10_t2.csv",
    r"gTurf_s15_t1.csv"
]

savePath = "AllDataDF.csv"

for i in range(len(dataFiles)):

    df = pd.read_csv(os.path.join(dataFolder, dataFiles[i]))
    df = df.rename(columns={'Unnamed: 0': 'Seq'})
    df = df.set_index('Seq')

    print(df.isnull().sum().sum())
    df = df.interpolate(method='polynomial', order=1)
    print(df.isnull().sum().sum())

    df = df.dropna()
    print(df.isnull().sum().sum())
    df = df.reset_index().drop(columns=['Seq'])

    df = df.drop(columns=['OdomPosZ', 'OdomOrientX', 'OdomOrientY', 'OdomLinY', 'OdomLinZ', 'OdomAngX', 'OdomAngY'])

    def getDeltaCol(col,delta):
        deltaCol = pd.Series([ col.iloc[i] - col.iloc[i-delta] if i>=delta else 0 for i in range(len(col))])
        return deltaCol

    dList = [2,4,8,16,32,64]
    for col in df.columns.tolist():
        if col!='Sensor':
            for d in dList:
                df[col+'Delta{}'.format(d)] = getDeltaCol(df[col], d)
                print('Added ' + col +'Delta{}'.format(d))
        else:
            print('Skipped sensor')

    df = df.iloc[64:].reset_index().drop(columns=['index'])
    df = df.drop(columns=['Sensor', 'Time'])

    if i==0:
        mainDf = df.copy(deep=True)
    else:
        mainDf = pd.concat([mainDf, df], axis=0, sort=False)
    print("Added {} of size {} to mainDf. ".format(dataFiles[i], df.shape))
    print("MainDf is now size {}".format(mainDf.shape))
    print('\n')

fullSavePath = os.path.join(dataFolder, savePath)
mainDf.to_csv(fullSavePath)
print('\n')
print("Saved mainDf to {}".format(fullSavePath))
print("MainDF has size {}".format(mainDf.shape))
