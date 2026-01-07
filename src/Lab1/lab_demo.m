%% USE THIS FILE TO GET SIGN OFF 3

robot = Robot();  % Create a robot object to use
interpolate_time = 5; % TODO: choose a reasonable time to interpolate

%% Send the robot to its 0 position
% We don't need to collect data yet, as we're just putting the robot at its
% starting point

robot.writeJoints([0, 0, 0, 0]);
pause(6);
    
%% Send the robot to an arbitrary position
% TODO: Choose a joint position
pos1 = [-50 45 -80 -65];  

% TODO: Initialize an array full of 0s to store the positions
currPos = zeros(4);

% See https://www.mathworks.com/help/matlab/ref/zeros.html
% We do this because allocating memory takes a lot of time, so we only want
% to do it once

% Initialize another array for the timestamps (or use the same array as the
% positions, your call)
jointReadings = zeros(3,4);
allReadings = zeros(interpolate_time*21, 4);

% TODO: Call a function to move the robot
robot.writeJoints(pos1);

% Collect data as the robot movesshould be hundreds or more.
tic;  % Start timer
toc_Count = 1;
while toc < interpolate_time
    % Read current joint positions (not velocities though!)
    jointReadings = robot.getJointsReadings();
    allReadings(toc_Count, :) = jointReadings(1, :);
    
    % disp(jointReadings(1, :));

    % Store the positions and timesetamps in an array 
    % Hint: use 'toc' to get the current elapsed time since tic
    toc_Count = toc_Count + 1;
end

%disp(allReadings);

%% Make your figure
% To use subfigures, you'll use the subplots feature of MATLAB.
% https://www.mathworks.com/help/matlab/ref/subplot.html

subplot(4,1,1);
x = (0:(interpolate_time*21 - 1))/21.0;
y_1 = transpose(allReadings(:, 1));
plot(x, y_1);
xlabel("time (s)");
ylabel("motor 1 (deg)");

subplot(4,1,2);
y_2 = transpose(allReadings(:, 2));
plot(x, y_2);
xlabel("time (sec)");
ylabel("motor 2 (deg)");

subplot(4,1,3);
y_3 = transpose(allReadings(:, 3));
plot(x, y_3);
xlabel("time (sec)");
ylabel("motor 3 (deg)");

subplot(4,1,4);
y_4 = transpose(allReadings(:, 4));
plot(x, y_4);
xlabel("time (sec)");
ylabel("motor 4 (deg)");

mat_1 = zeros(1, (interpolate_time * 21) - 1);

for i = 1:1:(interpolate_time * 21 - 1)
    mat_1(i) = y_1(i+1) - y_1(i);
end
mat_1 = mat_1*21;
disp("MOTOR_1-----");
disp("mean: " + mean(mat_1));
disp("median: " + median(mat_1));
disp("max: " + max(mat_1));
disp("min: " + min(mat_1));

mat_2 = zeros(1, (interpolate_time * 21) - 1);

for i = 1:1:(interpolate_time * 21 - 1)
    mat_2(i) = y_2(i+1) - y_2(i);
end
mat_2 = mat_2*21;
disp("MOTOR_2-----");
disp("mean: " + mean(mat_2));
disp("median: " + median(mat_2));
disp("max: " + max(mat_2));
disp("min: " + min(mat_2));

mat_3 = zeros(1, (interpolate_time * 21) - 1);

for i = 1:1:(interpolate_time * 21 - 1)
    mat_3(i) = y_3(i+1) - y_3(i);
end
mat_3 = mat_3*21;
disp("MOTOR_3-----");
disp("mean: " + mean(mat_3));
disp("median: " + median(mat_3));
disp("max: " + max(mat_3));
disp("min: " + min(mat_3));

mat_4 = zeros(1, (interpolate_time * 21) - 1);

for i = 1:1:(interpolate_time * 21 - 1)
    mat_4(i) = y_4(i+1) - y_4(i);
end
mat_4 = mat_4*21;
disp("MOTOR_4-----");
disp("mean: " + mean(mat_4));
disp("median: " + median(mat_4));
disp("max: " + max(mat_4));
disp("min: " + min(mat_4));

%% MATLAB may have some functions for the requested statistics...