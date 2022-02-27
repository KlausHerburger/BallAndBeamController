# BallAndBeamController
This project considers a robot on which there is a beam with a ball. The robot is given various reference positions in sequence, which it is to move to without the ball falling off the beam. For this purpose, the angle of the beam and the position of the robot can be actively controlled by a motor. A controller was designed that reacts correctly to reference position changes without violating the constraints, despite disturbances.

![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/63397065/155311403-0b453fcf-450d-4c66-9c52-e92c209b2891.gif)

## Task

The goal of the task is to design a controller in such a way that the controller reacts correctly to reference jumps despite disturbances without violating the manipulated variable constraints. Different reference positions are given one after the other, which have to be processed. As soon as the robot reaches a reference position and is in a certain area close to the reference position for a certain time and the ball is close enough to the center of the beam, the next reference position is set. The reference size jumps are limited to 1 meter. The goal is to work through all reference positions in the shortest possible time. 

![image](https://user-images.githubusercontent.com/63397065/155877066-8ad9a136-7233-4676-8c88-9420f66ffb83.png)

## Concept

In order to perform a model-based controller design, a model of the system under consideration is first required. For this purpose, the equations of motion for the robot, the seesaw and the ball were first derived using Lagrange's equations of the second kind. Then, the motors used were modeled, which have a nonlinear motor characteristic. During the system analysis, a model linearized around a suitable operating point was derived from the nonlinear equations of motion for the subsequent controller design. 

A Ricatti controller turned out to be the most suitable controller for the overall system. The Ricatti controller is a state controller whose controller values are determined via the minimization of a quadratic cost functional and belongs to the optimal controllers. The advantage over the classical design of state controllers by pole placement is that the goodness of individual states and the control energy consumption can be considered directly. For the evaluation of the state controller, a goodness measure is introduced for each requirement. 
Here, the oscillations in stabilization are specifically calculated using the
Quality measure <img src="https://latex.codecogs.com/svg.image?J_\xi" title="J_\xi" /> using the diagonal matrix of weighting factors <img src="https://latex.codecogs.com/svg.image?\mathbf{Q}" title="\mathbf{Q}" /> with 

<img src="https://latex.codecogs.com/svg.image?J_\xi&space;=&space;\int_{0}^{\infty}\mathbf{\xi^T}(t)\mathbf{Q}\mathbf{\xi}(t)dt" title="J_\xi = \int_{0}^{\infty}\mathbf{\xi^T}(t)\mathbf{Q}\mathbf{\xi}(t)dt" />

and the place energy expenditure <img src="https://latex.codecogs.com/svg.image?J_\nu" title="J_\nu" /> with 

<img src="https://latex.codecogs.com/svg.image?J_\xi&space;=&space;\frac{1}{2}\int_{0}^{\infty}\mathbf{S}*v(t)^2dt" title="J_\xi = \frac{1}{2}\int_{0}^{\infty}\mathbf{S}*v(t)^2dt" />

For the weighting factors <img src="https://latex.codecogs.com/svg.image?\mathbf{Q}" title="\mathbf{Q}" /> a diagonal matrix is used, so that the goodness of the individual states can be given directly. It has been found that the third state, describing the position of the ball, has to be weighted highest and the first state, and thus the position of the robot, has the second highest priority, so that the best results are obtained. The matrix <img src="https://latex.codecogs.com/svg.image?\mathbf{S}" title="\mathbf{S}" /> is also chosen as a diagonal matrix with entries proportional to the mass of the robot for the first manipulated variable and the masses of the beam and the ball for the second manipulated variable. For the quality measure to be evaluated, the system must be stable so that the infinite integral converges. Furthermore, the states must be measurable, i.e. the system must be observable. These properties were verified.
The optimal Riccati controller that minimizes the goodness measure and thus optimally satisfies the requirements can be computed via a Matlab command that is given the system matrix <img src="https://latex.codecogs.com/svg.image?\mathbf{A}" title="\mathbf{A}" /> and the input matrix <img src="https://latex.codecogs.com/svg.image?\mathbf{B}" title="\mathbf{B}" />, as well as the weighting matrices <img src="https://latex.codecogs.com/svg.image?\mathbf{Q}" title="\mathbf{Q}" /> and <img src="https://latex.codecogs.com/svg.image?\mathbf{S}" title="\mathbf{S}" />. It is a state controller, which always feeds back the current state of the system to the input. Since this cannot be measured directly, a Luenberger observer is also used for estimation.

In order for the robot to follow a given trajectory in a short time, good guidance behavior is essential. However, disturbances have more effect if the control is designed for fast steering behavior.
For this reason, a two-degree-of-freedom controller is used for trajectory planning. The feedforward control determines how the setpoint acts on the input of the system; the controller suppresses the disturbances. With this controller structure, the guidance behavior can be determined independently of the disturbance behavior.
The approach of flatness-based trajectory generation is to find a coordinate representation so that the differential equations of the system simplify. Thus, it is possible to use the flat output to specify the robot position as a nominal polynomial.
The robot does not move arbitrarily to a given position, but it can be determined how fast and with which course it should move there. In addition, the seesaw balances the ball against disturbances. The state variables of the system and the manipulated variable for the robot can be represented by a flat output and its time derivatives. From the nominal polynomial, a nominal trajectory is obtained directly by back transformation, which describes the path between the two rest positions before and after displacement of the robot. The nominal trajectory flows into the controller for disturbance compensation and the manipulated variable, which the system follows, is specified via the feedforward control.

## Results

The following two figures show how the robot successfully follows the reference positions and the ball is balanced on the beam, despite the occuring disturbances.

![fig2](https://user-images.githubusercontent.com/63397065/155877953-baff4b7b-3ec6-4bf2-a7b3-4a5ecbf6ff40.jpg)
![fig](https://user-images.githubusercontent.com/63397065/155877952-75d3f529-4ae6-4500-8abc-b0acccffd611.jpg)
