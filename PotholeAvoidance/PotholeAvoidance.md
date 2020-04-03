# Autonomous Pothole Avoidance

### Jacob Laframboise, Jack Demeter, Jesse MacCormac, Daniella Ruisendaal, Anne Broughton
This work was presented at the *Canadian Undergraduate Conference on Artificial Intelligence* 2020. This work was done with *QMIND -  Queen's AI Hub*.

## Abstract
As robotics becomes ever more integrated into industry and society, improvements and insights into how robots avoid obstacles are critical to enhancing robotic abilities. We investigate this problem by developing an autonomous pothole avoidance system on the TurtleBot3 with the Robot Operating System. We developed a classifier CNN for pothole identification, and a localizer CNN to locate the pothole in frame. We calibrated pixel output to real-world coordinates, and developed a control module to avoid potholes. Our classifier CNN performed with 97.9% testing accuracy, and our localizer with a testing MSE loss of 0.4096 pixels. Each component successfully completes its task in the pipeline to avoid a pothole. We aim to improve component integration for full autonomy, and improve our dataset with more variation to enhance our model’s ability to generalize. 

## Methods

### Hardware and ROS
The solution was implemented on a TurtleBot3 Waffle Pi1 with a Raspberry Pi Camera Module v2.1 on the Robot Operating System (ROS). 

### Classifier
A convolutional neural network (CNN) was used to identify if a pothole is in the image frame. This network comprised of five convolutional blocks, where each block consisted of a convolutional layer, max pooling, and dropout. The first convolutional layer had 16 nodes, doubling each subsequent convolutional layer. A (3,3) kernel with rectified linear unit activation was used. Dropout was applied with increasing strength towards the latter convolutional blocks. The model was completed with a 128-node dense layer followed by flattening, and a final single dense node with sigmoid activation. This model takes in a (128, 128, 3) RGB input, and outputs a one or zero to indicate whether a pothole is in frame. A 20% test split was used on a dataset totaling 7002 labelled images sampled from video. Data was augmented with rotation, translation, and reflection. The model was trained with the Adam  optimizer using binary cross entropy loss for 10 epochs. 

### Localizer
A CNN with similar architecture was used to localize the pothole in the robot’s camera image. The difference is the model is finished with four dense nodes with linear activation for regression. This model takes in an RGB image of shape (128, 128, 3), and outputs a bounding box represented as (x, y, width, height) where (x, y) are the coordinates of the top left corner of the bounding box. 4248 images sampled from video were annotated using LabelImg, and a 20% testing split was used. Data was augmented with rotation, translation, and reflection. The model was trained for 20 epochs with the Adam optimizer on a mean squared error (MSE) loss function. 

### Camera calibration
To map pixel coordinates to 2D world coordinates, the camera was calibrated with an array of markers at known distances on the floor. GIMP1 was used to identify the pixel coordinates in an image captured with RViz for each point. The Homography algorithm was used to compute a linear map between pixel coordinates and 2D real-world coordinates. This calibration matrix was tested on new points to assess the calibration error.

### Avoidance logic
To avoid the bounding box coordinates, the angle at which the robot must turn to avoid the pothole plus a 10cm safety margin is calculated. The robot then drives straight until it is at the same latitude as the bounding box, when it moves forward the height of the bounding box. Here, the robot is past the bounding box of the pothole, so it turns back at twice the original angle. It drives the same distance, returning to course and completing a symmetric trapezoid.

## Results
The classifier reached an accuracy of 97.9% on the testing set. A qualitative examination of its errors shows that it tends to error when there are significant cracks in the road that may be mistaken for potholes.

The localizer achieved a MSE training loss of 0.5682 pixels and a MSE testing loss of 0.4096 pixels.

The avoidance algorithm was accurate in most cases; however, the algorithm occasionally over rotated the robot, breaching the safety margin. 

## Future work
We aim to integrate all components for fully autonomous operation identifying and avoiding potholes. The classifier and localizer are already integrated with the localizer only applied when the classifier detects a pothole. However, this will be integrated with the pothole avoidance logic, so that the robot can avoid a pothole it detects in frame with no intervention. Significant integration testing will be required. 

In order to reduce overfitting in the classifier and localizer models, we need to collect a more varied dataset with a greater variety of potholes. Furthermore, a camera mounted higher off the ground would allow for more detailed images of the ground ahead. This could be achieved with either a different model of robot, or a different camera mount. These improvements should facilitate fully autonomous performance. 