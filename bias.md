# Bias Exploration
By Jack Demeter

This folder analyzes potential sources of error and provides recommendation on how to mitigate them.

### Test Data
1. When collecting test data the team noticed a the robot would consistantly rotate to the right during travel on specific terains.
  Potential solutons: 
    - modify the motor data when collecting data to ensure that the robot moves in a straight line with constant velocity (potentially create a script to maintain orientation)
    - augment the data (specifically z rotation and x,y position to simulate left turning
2. The odom orientation/pose data stores the direction of the robot which could lead to predictions/clustering based on false constants
  Potential solutions:
    - reset the odom pose/orientation values to 0 during testing
    - drop the data from tests
    - remove initial bias from odom pose/orientation similar to time
3. Furthering the pattern seen in points 1 and 2 remove any non-delta values from the dataset to prevent any ML/NN from creating an identifier based on positional data.
4. Removing velocity may create more of a focus on other features and create better seperation.

