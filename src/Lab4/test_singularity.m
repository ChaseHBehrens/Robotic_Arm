% Move your robot to a point where Y=0, X > 150 and Z > 150

robot = Robot();

robot.writeTime(1);

p0 = [0 200 300 0];
p1 = [0 -200 300 0];
t0 = 0;
t1 = 5;

p0IK = robot.ik3001(p0);
robot.writeJoints(p0IK);
pause(1);

coeff_mat = zeros(4,6);

% Generate a trajectory that will move your robot in a straight line from
% your current (X, Y, Z) to the point (X, -Y, Z)

%% Execute the trajectory
travelTime = t1-t0;
robot.writeTime((t1-t0)/100000);

for i = 1:1:4
    coeff_mat(i,:) = TrajGenerator.quintic_traj([p0(i);p1(i);0;0;0;0;],t0,t1);
end

dets = zeros(1,316);

i=0;
tic;
while toc < travelTime
    % Evaluate your trajectory, and send your robot to its next setpoint
    next_point = TrajGenerator.eval_traj(coeff_mat,toc);
    next_joints = robot.ik3001(next_point);
    robot.writeJoints(next_joints);

    i=i+1;
    % Record the joint position of your robot
    joints = robot.getJointsPositionReadings();

    % Record the determinant of the Jacobian
    j = robot.jacob3001(joints);

    % Test if your robot is too close to a singularity, and stop if it is
    if robot.atSingularity(j, 0.1)
        plot(dets);
        error("Approaching Singularity");
    end 

    Jp = j(1:3, 1:4);
    JpT = transpose(Jp);
    d = det(Jp*JpT);
    dets(1,i) = d;
end

plot(dets);