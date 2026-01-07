% IP = ImageProcessor();
robot = Robot();
robot.writeTime(2);
robot.writeJoints([0 0 0 0]);
pause(2);
robot.pick_up_ball([125 -100 55 -90], "red");
% robot.blocking_js_move([0 0 0 0]);
% disp("process started");
% disp(robot.getJointsPositionReadings());
% 
% robot.blocking_js_move(robot.ik3001([125 -100 55 -90]));
% disp("pre-alocation finalized");
% robot.open_gripper();
% %disp(robot.fk3001(robot.getJointsPositionReadings()));
% 
% robot.blocking_ts_move([125 -100 25 -90]);
% robot.close_gripper();
% disp("ball acquired... depositing");
% 
% robot.blocking_js_move(robot.ik3001([125 -100 55 -90]));
% robot.blocking_js_move(robot.ik3001([0 -120 55 -90]));
% robot.blocking_js_move(robot.ik3001([0 -120 25 -90]));
% robot.open_gripper();
% robot.blocking_js_move(robot.ik3001([0 -120 55 -90]));
% disp("process finalized")
% robot.blocking_js_move([0 0 0 0]);
% robot.close_gripper();
% 
% I = IP.camera.getImage();
% imshow(I);