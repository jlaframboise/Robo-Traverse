# Robo-Traverse

### Jacob Laframboise, Jack Demeter, Jesse MacCormac, Daniella Ruisendaal, Anne Broughton



## Intro ##
The QMIND DAIR Robotic Traversal Team works towards understanding and improving robotic traversal over terrain. We take a two pronged approach to traversability, by (I) modelling IMU sensor data as a Turtlebot3 robot traverses different terrains, and (II) by developing an autonomous pothole avoidance system on a Turtlebot3 robot.

## I. Sensor data exploration

Using data recorded of a Turtlebot3 Waffle Pi driving over several different surfaces, we do exploratory data analysis to validate assumptions and learn the distribution of the sensor data. We then attempt to cluster and classify the data by which surface the robot was driving on. We use classical machine learning methods such as logistic regression, as well as deep learning methods such as LSTM networks to map the sensor data to which surface the robot is driving on. 

## II. Autonomous Pothole Avoidance

We focus on the discrete event of driving over a pothole; more specifically how to automatically avoid doing so. We developed a set of linked components for the Turtlebot3 Waffle Pi on the Robot Operating System to:
1. Identify if a pothole is present in frame
2. Localize where the pothole is in frame
3. Map the pixel output to world coordinates
4. Avoid the pothole

