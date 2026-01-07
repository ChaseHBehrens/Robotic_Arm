debug(true);
IP = ImageProcessor();
robot = Robot();

%I = imread('checkerboard_image.jpg');

%IP.generate_static_mask();
I = IP.camera.getImage();
imshow(I);

pause(10);

disp("TForm");
disp(IP.camera.getTForm);
disp("Camera Pose");
disp(IP.camera.getCameraPose);
disp("Camera Instrinsics");
disp(IP.camera.getCameraInstrinsics());
disp("Camera rotation Matrix");
disp(IP.camera.getRotationMatrix());
disp("Camera Translation vector");
disp(IP.camera.getTranslationVector());

% gm = redMask(IP.generate_static_mask());

IP.detect_centroids();