# Projektwettbewerb-KRT

This Project is for use of the course "Projektwettbewerb Konzepte der Regelungstechnik" or in English "Project Competition of Concepts of the Control Technique" at Institute for Systems Theory and Automatic Control of University of Stuttgart, Germany, held by Professor F. Allgoewer. This project design the control algorithm for a simulated vehicle so that it can automatically travel around a given circuit.

## Usage

1. Extract the file in a folder.
2. Open MATLAB and set current folder to where you have just extract the file.
3. Run the script init.m. After some time a figure window will pop up and show the trajectory of the vehicle on the track.
4. The control algorithm is included within the file controller.m, where one can modify it.

## Features

1. Route planning: We designed the reference route by concatenating various straight lines and arcs so that it fits the edge of the given track.
2. Control Algorithm: A PID control was applied on the steering angle according to the lateral position error, the lateral velocity error as well as the azimut angle error of the vehicle. PID control was also applied on the throttle opening based on a given reference velocity at corners, while it was open-loop controlled on straights. Finally, the gear selection is open-loop controlled based on current velocity.

## Contributions

The controller design part, within the file controller.m, was established by me, Y. Wang, and my colleague, P. Yang. The other parts that support the whole project, was given by J. M. Montenbruck back in 2013. For contact:

To me: [st171307@stud.uni-stuttgart.de](mailto:st171307@stud.uni-stuttgart.de)

To Mr. Montenbruck: [jan-maximilian.montenbruck@ist.uni-stuttgart.de](mailto:jan-maximilian.montenbruck@ist.uni-stuttgart.de)

## Acknowledgements

This project uses MATLAB software by MathWorks. MATLAB is a registered trademark of The MathWorks, Inc. For more information about MATLAB and to obtain a license, please visit [mathworks.com](https://www.mathworks.com/).
