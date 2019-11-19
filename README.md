# Robo-Traverse

## Intro ##
The QMIND DAIR-Imaging team focuses on the development of an image-based traversal algorithm to determine the viability for a Turtle3Bot to traverse a given path (straight infront). Traversibility is measure using IMU data provided on board the robot with the intention that a neural network will be able to determine the smoothest, and thus most traversable path to take.

It is expected that the results of this project could be applied alongside additional pathing algorithms and sensors for a robot to determine the smoothest path between any two arbitrary points.

## Objectives ##
The team is concentrated on determining the four main points below:
1. Collection of data and development of a complete dataset: Collecting IMU and positioning data from the robot along numerous material surfaces.
2. Clustering and experimentation: Examination of datasets to determine patterns and correlations, along with disguishing and eliminating sources of bias prior to testing.
3. Neural network training: Training a convolutional neural network to take on board camera data to predict the IMU results of a given material or the traversibility.
4. Implementation: Executing the neural network alongside a simple traversal algorithm that determines the ideal direction of travel for the robot.

#### Written by Jack Demeter ####
