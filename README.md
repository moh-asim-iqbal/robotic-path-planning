# Robotic-Path-Planning
Gradient decent path planning algorithm for 3 degree of freedom robotic manipulator. Algorithm takes end effectory goal position, manipulator starting joint configuration, and 2d approximation of obstacles. Result is list of joint configurations that lead robotic manipulator to goal while also avoiding obtacles. To verify and derive spatial constraints, 3D CAD model of enviroment was created. For more detail and derivation refer to "Algorithm Report.pdf" in the repo.

### Robotic Concepts Used By Algorithm
- Inverse Kinematics
- Attractive and repulsive fields
- Jacobian matrices calculation
- Joint torque calculations

#### CAD model of scenario 
![alt text](https://github.com/moh-asim-iqbal/robotic-path-planning/blob/master/img/manipulator.png?raw=true)
  
#### DH convention model of manipulator
![alt text](https://github.com/moh-asim-iqbal/robotic-path-planning/blob/master/img/Dh-convention.png)

#### Example Algorithm Result
Each column (left to right) represents a manipulator joint based on DH convention. All joints are rotary and results are in degrees.
![alt text](https://github.com/moh-asim-iqbal/robotic-path-planning/blob/master/img/path.png)

<object data="https://github.com/moh-asim-iqbal/robotic-path-planning/blob/master/Algorithm%20Report.pdf" type="application/pdf" width="700px" height="700px">
    <embed src="https://github.com/moh-asim-iqbal/robotic-path-planning/blob/master/Algorithm%20Report.pdf">
        <p>This browser does not support PDFs. Please download the PDF to view it: <a href="https://github.com/moh-asim-iqbal/robotic-path-planning/blob/master/Algorithm%20Report.pdf">Download PDF</a>.</p>
    </embed>
</object>
