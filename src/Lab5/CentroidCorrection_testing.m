robot = Robot();
IP = ImageProcessor();

I = IP.camera.getImage();
imshow(I);

pause(10);
robot.writeTime(2);
robot.writeJoints([0 0 0 0]);
pause(2);


disp("hello");
disp(IP.correct_centroids(IP.detect_centroids()));
robot.pick_up_ball([IP.correct_centroids(IP.detect_centroids()), -90], "red");