% Demo_06: Check required motor velocities from commanded velocities
%  Author: Ivan A. Calle Flores
%
%  NOTE: Recommended values
%   - VMAX = 0.5;
%   - WMAX = PI/4
clc; clearvars; close all
NAME_ROBOT = 'R2D2-01';   % 'R2D2-00', 'R2D2-01', 'R2D2-R1', 'R2D2-R2'
VMAX = 1*0.5;
WMAX = 1*pi/4;

%----------------------------------------------------------------------%
% 1. CREATE AN OBJECT "my_robot" OF THE TYPE "R2D2-xx"
%----------------------------------------------------------------------%
my_robot = robot(NAME_ROBOT);


%----------------------------------------------------------------------%
% 2. COMPUTE THE ANGULAR VELOCICITIES OF THE MOTORS
%----------------------------------------------------------------------%
%  2.1. COMPUTE ANGULAR MOTOR VELOCITIES
[w_motor1, w_motor2] = my_robot.compute_motor_velocities(VMAX,WMAX);
%  2.2. SHOW RESULTS
disp('Angular velocities of the motors:')
disp([w_motor1  w_motor2] )
%  2.3. CHECK CORRECTNESS
vv = 0.5*my_robot.r*(w_motor1 + w_motor2);
ww = 0.5*my_robot.r/my_robot.B*(w_motor1 - w_motor2);
disp('Velocity check:')
disp([vv  ww])
%  2.4. CHECK CORRECTNESS USING THE ROBOT
[v, w] = my_robot.compute_robot_velocities(w_motor1,w_motor2);
disp('Velocity check 2:')
disp([v  w])