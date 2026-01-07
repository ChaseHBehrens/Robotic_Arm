% In this file, write your "main" script for picking up the balls
% This should only be 10-15 lines long, the vast majority of the work
% should be in your Robot and ImageProcessor classes

% Ultimately it's up to you to decide how to organize this, but the
% "detect_balls" and "pick_up_ball" functions should do a lot of the work
% for you.

robot = Robot();
IP = ImageProcessor();
robot.go_home();
disp("Place objects onto checkerboard. Press any key to continue.");
waitforbuttonpress;

[centroids, colors] = IP.detect_centroids();
centroids = IP.correct_centroids(centroids);
for i = 1:size(centroids, 1)
    robot.pick_up_ball([centroids(i, :), -90], colors(i));
end

robot.go_home();