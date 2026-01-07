% Create a robot object
robot = Robot();
travel_time = 1;  % CHOOSE SOMETHING REASONABLE
increment = 0.01;

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your robot


%robot.writeJoints(robot.ik3001([130 105 260 30]));
%robot.writeTime(1);

%p0 = [281.4 0 224.326 0];
%p1 = [125 0 155 -45];
%p2 = [225 -0 155 -45];
%p3 = [175 0 65 -45];

%disp(robot.fk3001([0 -16 -13 106]));
%disp(robot.fk3001([0 14 -42 90]));
%disp(robot.fk3001([0 6 4 73]));

%p0_ik = robot.ik3001(p0);
%robot.writeJoints(p0_ik);
%pause(1);

%----------------------- SIGNOFF 3 -----------------------------
t0 = 1;
t1 = 2;
q0 = pi/2;
q1 = pi/3;
dt = 0.01;
coeff_mat_cubic = TrajGenerator.cubic_traj([q0;q1;0;0;],t0,t1);
coeff_mat_quintic = TrajGenerator.quintic_traj([q0;q1;0;0;0;0;],t0,t1);

time_mat = linspace(dt+t0,t1,(t1-t0)/dt);
%disp(time_mat);
robot.writeTime(increment);


% --------------------- TRIANGULATION --------------------------
coeff_mat_01 = zeros(4,4);
coeff_mat_12 = zeros(4,4);
coeff_mat_23 = zeros(4,4);
coeff_mat_31 = zeros(4,4);

for i = 1:1:4
    coeff_mat_01(i,:) = TrajGenerator.cubic_traj([p0(i);p1(i);0;0;],0,travel_time);
    coeff_mat_12(i,:) = TrajGenerator.cubic_traj([p1(i);p2(i);0;0;],0,travel_time);
    coeff_mat_23(i,:) = TrajGenerator.cubic_traj([p2(i);p3(i);0;0;],0,travel_time);
    coeff_mat_31(i,:) = TrajGenerator.cubic_traj([p3(i);p1(i);0;0;],0,travel_time);
end

points = zeros(3*size(time_mat,2),4);
joints = zeros(3*size(time_mat,2),4);
%disp(points);
%points_quintic = zeros(size(time_mat,1),4);

% ------------------- SIGNOFF 3 --------------------------------
% joints_cubic = zeros(size(time_mat,1),1);
% joints_quintic = zeros(size(time_mat,1),1);
% omegas_cubic = zeros(size(time_mat,1)-1,1);
% omegas_quintic = zeros(size(time_mat,1)-1,1);
% alphas_cubic = zeros(size(time_mat,1)-1,1);
% alphas_quintic = zeros(size(time_mat,1)-1,1);
% for i = 1:1:size(time_mat,2)
%     temp_mat = TrajGenerator.eval_traj(coeff_mat_cubic,time_mat(i));
%     joints_cubic(i) = temp_mat(1,1);
%     temp_mat = TrajGenerator.eval_traj(coeff_mat_quintic,time_mat(i));
%     joints_quintic(i) = temp_mat(1,1);
% end
% for i = 1:1:size(time_mat,2)-1
%     omegas_cubic(i) = joints_cubic(i+1) - joints_cubic(i);
%     omegas_quintic(i) = joints_quintic(i+1) - joints_quintic(i);
% end
% for i = 1:1:size(time_mat,2)-2
%     alphas_cubic(i) = omegas_cubic(i+1) - omegas_cubic(i);
%     alphas_quintic(i) = omegas_quintic(i+1) - omegas_quintic(i);
% end


% For each leg of your triangle, calculate the TASK SPACE trajectory
% between each vetex. Remember, to calculate a task space trajectory 
% you will need to create individual trajectories for x, y, z, and alpha.

% Move your robot to the first vertex

%disp(size(time_mat,2));

% Create a for loop that sends your robot to each vertex, recording the
% JOINT-SPACE position the entire time
for i = 1:1:4
    for j = 1:1:size(time_mat,2)
        acc = 0;
        if (i == 1)
            acc = j;
            points(acc,:) = TrajGenerator.eval_traj(coeff_mat_01,time_mat(j));
        elseif (i == 2)
            acc = j;
            points(acc,:) = TrajGenerator.eval_traj(coeff_mat_12,time_mat(j));
        elseif (i == 3)
            acc = j+size(time_mat,2);
            points(acc,:) = TrajGenerator.eval_traj(coeff_mat_23,time_mat(j));
        elseif (i == 4)
            acc = j+2*size(time_mat,2);
            points(acc,:) = TrajGenerator.eval_traj(coeff_mat_31,time_mat(j));
        end
        %currJoints = robot.getJointsReadings();
        %joints(acc,:) = currJoints(1,:);

        robot.writeJoints(robot.ik3001([points(acc,:)]));
        pause(increment);
    end
end

%disp(points);

%scatter3(joints(:,2),joints(:,3),joints(:,4));


% --------------------- SIGNOFF 3 --------------------------
subplot(3,1,1);
plot(time_mat,joints_cubic,'b');
hold on;
plot(time_mat,joints_quintic,'--r');
hold off;
legend({'Cubic','Quintic'});

subplot(3,1,2);
plot(time_mat(1:end-1), (omegas_cubic * 100),'b');
hold on;
plot(time_mat(1:end-1), (omegas_quintic * 100),'--r');
hold off;
legend({'Cubic','Quintic'});

subplot(3,1,3);
plot(time_mat(1:end-2),alphas_cubic,'b');
hold on;
plot(time_mat(1:end-2),alphas_quintic,'--r');
hold off;
legend({'Cubic','Quintic'});

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html


% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
