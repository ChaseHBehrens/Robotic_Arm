% Create a robot object
robot = Robot();
interpolate_time = 1;

% Define three joint positions to form the vertices of your triangle
% Make sure theta_1 = 0 for all of these configurations!


% Move your robot to the first vertex
%disp(robot.fk3001([0, 0, 0, 0]));

robot.writeTime(interpolate_time);

p1 = [0 -16 -13 106];
p2 = [0  14 -42  90];
p3 = [0   6   4  73];

points = [p1; p2; p3];
point_ct = 3;
loop_ct = 3;

jointReadings = zeros(3,4);
t2Readings = zeros(interpolate_time*21*point_ct*loop_ct, 1);
t3Readings = zeros(interpolate_time*21*point_ct*loop_ct, 1);
t4Readings = zeros(interpolate_time*21*point_ct*loop_ct, 1);

taskReadings = zeros(4,4);
xReadings = zeros(interpolate_time*21*point_ct*loop_ct, 1);
yReadings = zeros(interpolate_time*21*point_ct*loop_ct, 1);
zReadings = zeros(interpolate_time*21*point_ct*loop_ct, 1);

% Create a for loop that sends your robot to each vertex, recording the
% joint-space position the entire time
toc_Count = 1;
for j=1:loop_ct
    for i=1:point_ct
        % See lab_demo.m from lab 1 for inspiration on how to move your robot
        % and collect data
        %disp("-----------------------------------------");
        robot.writeJoints(points(i,:));
        %disp(robot.fk3001(jointReadings(1,:)));
        %disp("-----------------------------------------");
        %disp(robot.fk3001(points(i,:)));
        %disp("-----------------------------------------");

        % Collect data as the robot movesshould be hundreds or more.
        tic;  % Start timer
        while toc < interpolate_time
            % Read current joint positions (not velocities though!)
            jointReadings = robot.getJointsReadings();
            t2Readings(toc_Count) = jointReadings(1,2);
            t3Readings(toc_Count) = jointReadings(1,3);
            t4Readings(toc_Count) = jointReadings(1,4);

            taskReadings = robot.fk3001(jointReadings(1, :));
            xReadings(toc_Count) = taskReadings(1,4);
            yReadings(toc_Count) = taskReadings(2,4);
            zReadings(toc_Count) = taskReadings(3,4);

            % Store the positions and timesetamps in an array 
            % Hint: use 'toc' to get the current elapsed time since tic
            toc_Count = toc_Count + 1;
        end
    end
end

%robot.writeJoints([0 0 0 0]);

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html

%disp("-----------------------------------------");
%disp(xReadings);
%disp("-----------------------------------------");
%disp(yReadings);
%disp("-----------------------------------------");
%disp(zReadings);
%disp("-----------------------------------------");
%scatter3(xReadings, yReadings, zReadings);
scatter3(t2Readings, t3Readings, t4Readings);
disp("-----------------------------------------");
disp(xReadings);
disp("-----------------------------------------");
disp(yReadings);
disp("-----------------------------------------");
disp(zReadings);
disp("-----------------------------------------");
%scatter3([0 1 2], [3 4 2], [1 4 3]);

% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
