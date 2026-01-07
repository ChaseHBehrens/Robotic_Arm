%% Step 1: Define your symbolic DH table
% See sym_example.m for guidance on how to define symbolic variables
% Remember: you CAN freely mix symbolic variables and plain-ol numbers.

robot = Robot();

syms theta1 theta2 theta3 theta4;

L0 = 36.076;
L1 = 96.326 - L0;
L2x = 24;
L2y = 128;
phi = atan2d(L2x,L2y);
L2 = sqrt(L2x^2 + L2y^2);
L3 = 124;
L4 = 133.4;

DH_Table = [0,                 L0, 0,  0;
            theta1,            L1, 0,  -90;
            theta2 + phi - 90, 0,  L2, 0;
            theta3 + 90 - phi, 0,  L3, 0;
            theta4,            0,  L4, 0];

%% Step 2: Pass your symbolic DH table into dh2fk to get your symbolic 
% FK matrix
% It's really that simple. MATLAB will leave everything unsolved
% Show this to an SA for SIGN-OFF #4

DH_Transform = robot.dh2fk(DH_Table);
disp(DH_Transform);

%% Step 3: Feed your symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.

robot_matrix = matlabFunction(DH_Transform, "File", "fk_3001");
disp(robot_matrix);

% Write the fk_3001 function in Robot.m to complete sign-off #5

%disp(robot.fk3001([0, 0, 0, 0]));

% Curiosity bonus (0 points): replicate the timeit experiment I did in
% sym_example.m to compare the matlabFunction FK function to just using
% subs to substitute the variables.

%% Lab 4: Differential Kinematics

% Beta = 10.6197;
% 
% x = (L2*sind(theta2 + Beta) + L3*cosd(theta2 + theta3) + L4*cosd(theta2 + theta3 + theta4))*cosd(theta1);
% y = (L2*sind(theta2 + Beta) + L3*cosd(theta2 + theta3) + L4*cosd(theta2 + theta3 + theta4))*sind(theta1);
% z = L0 + L1 + L2*cosd(theta2 + Beta) - L3*sind(theta2 + theta3) - L4*sind(theta2 + theta3 + theta4);
% 
% Jp = [diff(x, theta1), diff(x, theta2), diff(x, theta3), diff(x, theta4);
%       diff(y, theta1), diff(y, theta2), diff(y, theta3), diff(y, theta4);
%       diff(z, theta1), diff(z, theta2), diff(z, theta3), diff(z, theta4);];
% 
% disp(Jp);
% 
% zhat1 = [0; 0; 1];
% zhat2 = [-sind(theta1); cosd(theta1); 0]; 
% zhat3 = [-sind(theta1); cosd(theta1); 0];
% zhat4 = [-sind(theta1); cosd(theta1); 0];
% 
% Jo = [zhat1, zhat2, zhat3, zhat4];
% 
% robot_jac = [Jp;Jo];
% 
% J = matlabFunction(robot_jac, "File", "dk_3001");
% 
% disp(robot.jacob3001([0,0,0,0]));