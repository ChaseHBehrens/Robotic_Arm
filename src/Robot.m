%(c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
    end

    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(5);
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            if checkSafe(goals)
                goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            end
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwthis symbolic DH table into your dh2fk functiom voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end
        
        function readings = getJointsPositionReadings(self)
            readings = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        end

        function readings = getJointsVelocityReadings(self)
            readings = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
    
        %% LAB 1-----------------------------------------------------------
        % ALL code for lab 1 goes in this section ONLY

        function servo_jp(self, q)
        %SERVO_JP Send robot to a joint configuration
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions

            self.writeTime(0.001);
            self.writeJoints(q);
            
            % Hint: look at lab1_base.m to see how to move your robot
            % Hint 2: don't set the travel time to 0, as that causes 
            %         ~unintended behaviors~; do something small like
            %         0.001
        end
        
        function interpolate_jp(self, q, time)
        %INTERPOLATE_JP Send robot to a joint configuration over a period of time
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
        %    t: a scalar that tells how long to take to travel to the new position
        %       in milliseconds

            self.writeTime(time * 1000);
            self.writeJoints(q);

        end

        function q_curr = measure_js(robot, GETPOS, GETVEL) 
        %MEASURED_JS Get the current position and velocity of the robot
        % Inputs:
        %    GETPOS: a boolean indicating whether or not to retrieve joint
        %            positions
        %    GETVEL: a boolean indicating whether or not to retrieve joint
        %            velocities
        % Outputs:
        %    q_curr: a [2x4] matrix whose top row contains joint positions (0s if
        %            GETPOS is false), and whose bottom row contains joint 
        %            velocities
        
            % This line gets the current positions of the joints
            % (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            
            % This line gets the current joint velocities
            % (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);

            q_curr = zeros(2, 4);
            if (GETPOS)
                q_curr(1,:) = (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end
            if (GETVEL)
                q_curr(2,:) = (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
            end
            
            % Hint: Only collect the data you need; serial read operations
            % are expensive!
        end
        %% END LAB 1 CODE -------------------------------------------------
        %% BEGIN LAB 2 CODE -----------------------------------------------
        function ht = dh2mat(self, dh_row)
        %DH2MAT Gives the transformation matrix for a given row of a DH table
        % Inputs:
        %    dh_row: a [1x4] vector containing representing a single row of a DH
        %            table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        row

            theta = dh_row(1);
            d = dh_row(2);
            a = dh_row(3);
            alpha = dh_row(4);

            ht = [cosd(theta), -sind(theta)*cosd(alpha), sind(theta)*sind(alpha), a*cosd(theta);
                  sind(theta), cosd(theta)*cosd(alpha), -cosd(theta)*sind(alpha), a*sind(theta);
                  0,           sind(alpha),             cosd(alpha),              d;
                  0,           0,                       0,                        1];
            
            % Hint: Use the degree version of trig functions
        end

        function ht = dh2fk(self, dh_tab)
        %DH2MAT_R Calculates FK from a DH table
        % Inputs:
        %    dh_tab: a [nx4] matrix representing a DH table
        % Outputs: 
        %    ht: a [4x4] matrix representing the transformation defined by the DH
        %        table

            % YOUR CODE HERE
            % Hint: Recall from lecture that the full FK of a manipulator can
            % be obtained by post multiplying all the HT matrices from each row
            % of the DH table
    
            % Hint 2: You just wrote a function to calculate the HT matrix
            % corresponding to one row of the DH table

            ht = eye(4);
            rows = size(dh_tab, 1);
            for i = 1:1:rows
                ht = ht * self.dh2mat(dh_tab(i,:));
            end

        end

        % YOUR CODE HERE
        % Write a function fk3001 that takes in an array of four joint
        % values and uses your matlabFunction'd FK to generate the FK of
        % your robot

        function robotFK = fk3001(self, Thetas)
            robotFK = fk_3001(Thetas(1), Thetas(2), Thetas(3), Thetas(4));
        end

        % Hint: All non-static methods (which this is) should take "self" as
        % their first argument

        %% END LAB 2 CODE
        %% BEGIN LAB 3 CODE
        function point_in_workspace(point) 
            % point is [x, y, z, gamma] where gamma is in degrees.
            l0 = 96.326;
            l1 = 130.23;
            l2 = 124;
            l3 = 133.4;
            r = sqrt(point(1)^2 + point(2)^2);
            z = point(3);
            g = point(4);
            if g < -180 || g > 180
                error('Gamma must be between -180 and 180 degrees');
            end
            if r < l3*cosd(g) - l2*sind(g) - l1
                error('Out of workspace, radius too small.');
            end
            if r > l1 + l2 + l3*cosd(g)
                error('Out of workspace, radius too large.');
            end
            if r >= l3*cosd(g) - l2*sind(g) - l1 && ...
               r <= l3*cosd(g) - (l1+l2)*sind(g) && ...
               z > l0 + l2*cosd(g) + l3sind(g) + sqrt(l1^2 - (r - l3*cosd(g) + l2*sind(g))^2)
                error('Out of workspace, position too high for given radius.');
            end
            if r >= l3*cosd(g) - (l1+l2)*sind(g) && ...
               r <= l1 + l2 + l3*cosd(g) && ...
               z > l0 + l3*sind(g) + sqrt((l1+l2)^2 - (r - l3*cosd(g))^2)
                error('Out of workspace, position too high for given radius.');
            end
            if r >= l3*cosd(g) - l2*sind(g) - l1 && ...
               r <= l2 - l1 +l3*cosd(g) && ...
               z < l0 + l3*sind(g) + sqrt((l1 + l2)^2 - (r - l3*cosd(g)))
                error('Out of workspace, position too low for given radius.');
            end
            if r >= l2 - l1 +l3*cosd(g) && ...
               r <= (l1 - l2)*sind(-g) + l3*cosd(g) && ...
               z < l0 + l3*sind(g) + sqrt((l1 - l2)^2 - (r - l3*cosd(g))^2)
                error('Out of workspace, position too low for given radius.');
            end
            if r >= (l1 - l2)*sind(-g) + l3*cosd(g) && ...
               r <= l1 - l2*sind(-g) + l3*cosd(g) && ...
               z < l0 - l2*cosd(g) - l3*sind(-g) + sqrt(l1^2 - (r - l2*sind(g) - l3*cosd(g))^2)
                error('Out of workspace, position too low for given radius.');
            end
            if r >= l1 - l2*sind(-g) + l3*cosd(g) && ...
               r <= l1 + l2 + l3*cosd(g) && ...
               z < l0 + l3*sind(g) - sqrt(l2^2 - (r - l1 - l3*cosd(g))^2)
                error('Out of workspace, position too low for given radius.');
            end
        end
        function qs = ik3001(self, pos)
            %IK3001_R Calculates IK for the OMX arm
            % Inputs:
            %    pos: a [1x4] matrix [x, y, z, alpha] representing a target pose in
            %    task space
            % Outputs: 
            %    qs: a [1x4] matrix representing the joint values needed to reach the
            %    target position

            %point_in_workspace(pos);

            qs = zeros(1,4);

            x = pos(1);
            y = pos(2);
            z = pos(3);
            alpha = -pos(4);
            
            % YOUR CODE HERE
            L0 = 36.076;
            L1 = 96.326 - L0;
            L2x = 24;
            L2y = 128;
            phi = atan2d(L2x,L2y);
            L2 = sqrt(L2x^2 + L2y^2);
            L3 = 124;
            L4 = 133.4;

            A = sqrt(x^2 + y^2);
            B = A - L4*cosd(alpha);
            C = z + L4*sind(alpha) - L1 - L0;
            
            theta1 = atan2d(y,x);
            theta2 = 90 - atan2d(C,B) - acosd((L2^2 + B^2 + C^2 - L3^2)/(2*L2*sqrt(B^2+C^2))) - phi;
            theta3 = 90 - acosd((L2^2 + L3^2 - B^2 - C^2)/(2*L2*L3)) + phi;
            theta4 = alpha - theta3 - theta2;

            qs(1) = theta1;
            qs(2) = theta2;
            qs(3) = theta3;
            qs(4) = theta4;

            % Hint: MATLAB debugger is your very best friend in the
            % entire world right now

            % TODO: Before running calculations, make sure
            %   1. The length of the atan2d(L2x,L2y);x, y, z vector is less than the
            %      length of the extended robot arm
            %   2. The z-value is not negative
            % Feel free to add any other invalid input conditions,
            % but these are the bare minimum
        
        end
        %% END LAB 3 CODE
        %% BEGIN LAB 4 CODE
        function j = jacob3001(self, qpos) 
            %JACOB3001 Calculates the jacobian of the OMX arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            % Outputs:
            %   j: a [6x4] jacobian matrix of the robot in the given pos
            j = dk_3001(qpos(1), qpos(2), qpos(3), qpos(4));
        end

        function vs = dk3001(self, qpos, qvel)
            %DK3001 Calculates the forward velocity kinematics of the OMX
            %arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            %   qvel: a [1x4] matrix composed of joint angular velocities
            % Outputs:
            %   vs: a [6x1] matrix representing the linear and angular 
            %       velocity of the end effector in the base frame of the 
            %       robot

            j = self.jacob3001(qpos);
            
            if self.atSingularity(j, 0.01)
                error("Approaching Singularity");
            end 

            vs = j*transpose(qvel);
        end
            
        % YOUR CODE HERE: Write a function atSingularity as described in
        % the lab document
        function result = atSingularity(self, J, threshold)
           Jp = J(1:3, 1:4);
           JpT = transpose(Jp);
           d = det(Jp*JpT);
           result = d < threshold;
        end


    %% END LAB 4 CODE
    %% BEGIN LAB 5 CODE
    function blocking_js_move(self, qpos, nvargs)
        arguments
            self Robot;
            qpos double;
            nvargs.time double = 2;
        end
        %BLOCKING_JS_MOVE moves the robot to a position in joint space
        %before exiting
        % Inputs:
        %   qpos: a [1x4] matrix of joint positions
        %   time: (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        
        % YOUR CODE HERE
        % NOTE: this funciton should not exit until the robot has stopped
        % moving
        self.writeTime(nvargs.time);
        self.writeJoints(qpos);
        pause(nvargs.time);
    end

    function blocking_ts_move(self, pos, nvargs)
        arguments
            self Robot;
            pos double;
            nvargs.time double = 2;
            nvargs.mode string = "cubic"
        end
        %BLOCKING_TS_MOVE moves the robot in a straight line in task space 
        %to the target position before exiting
        % Inputs:
        %   pos: a [1x4] matrix representing a target x, y, z, alpha
        %   time (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        %   mode (optional): a string "cubic" or "quintic" indicating what 
        %                    type of trajectory to utilize

        % YOUR CODE HERE
        % NOTE: this funciton should not exit until the robot has stopped
        % moving
        increment = 0.01;        
        time_mat = linspace(increment,nvargs.time,nvargs.time/increment);
        self.writeTime(increment);

        readings = self.getJointsPositionReadings();
        theta1 = readings(1);
        theta2 = readings(2);
        theta3 = readings(3);
        theta4 = readings(4);

        curr_pos = zeros(1,4);

        L0 = 36.076;
        L1 = 96.326 - L0;
        L2x = 24;
        L2y = 128;
        phi = atan2d(L2x,L2y);
        L2 = sqrt(L2x^2 + L2y^2);
        L3 = 124;
        L4 = 133.4;
        Beta = 10.6197;
        curr_pos(1,1) = (L2*sind(theta2 + Beta) + L3*cosd(theta2 + theta3) + L4*cosd(theta2 + theta3 + theta4))*cosd(theta1);
        curr_pos(1,2) = (L2*sind(theta2 + Beta) + L3*cosd(theta2 + theta3) + L4*cosd(theta2 + theta3 + theta4))*sind(theta1);
        curr_pos(1,3) = L0 + L1 + L2*cosd(theta2 + Beta) - L3*sind(theta2 + theta3) - L4*sind(theta2 + theta3 + theta4);
        curr_pos(1,4) = -readings(2)-readings(3)-readings(4);

        disp(curr_pos);

        if(nvargs.mode == "cubic")
            coeff_mat = zeros(4, 4);
            for i = 1:1:4
                coeff_mat(i,:) = TrajGenerator.cubic_traj([curr_pos(i);pos(i);0;0;],0,nvargs.time);
            end            
        elseif(nvargs.mode == "quintic")
            coeff_mat = zeros(4, 6);                
            for i = 1:1:4
                coeff_mat(i,:) = TrajGenerator.quintic_traj([curr_pos(i);pos(i);0;0;0;0;],0,nvargs.time);
            end
        end    
        % Create Points and Velocity Profile Matrices
        % points = zeros(3*size(time_mat,2),4);
        
        tic;
        while toc < nvargs.time
            next_point = TrajGenerator.eval_traj(coeff_mat,toc);
            next_joints = self.ik3001(next_point);
            self.writeJoints(next_joints);

            % Check for singularity
            joints = self.getJointsPositionReadings();
            j = self.jacob3001(joints);
            if self.atSingularity(j, 0.1)
                error("Approaching Singularity");
            end
        end
    end
    
    function open_gripper(self)
        self.writeGripper(true);
    end

    function close_gripper(self)
        self.writeGripper(false);
    end

        % @@@@@@@@@@@@@@
    % YOUR CODE HERE
    % @@@@@@@@@@@@@@
    % Write a function pick_up_ball that satisfies the following
    % requirements:
    function pick_up_ball(self, pos, color, nvargs)
        arguments
            self Robot;
            pos double;
            color string;
            nvargs.zoffset double = 55;
            nvargs.z_ball double = 25;
            nvargs.z_drop double = 25;
            nvargs.z_lift double = 55;
        end
        %PICK_UP_BALL picks up a ball and deposits it in the correct color bin
        % Inputs:
        %   pos: a [1x2] matrix representing the position of the ball in the XY
        %        frame of the robot
        %   color: a string indicating what color bin the ball should be placed
        %          in
        %   z_offset (optional): the z-position at which to begin the straight
        %                        vertical trajectory 
        %   z_ball (optional): the z-posiiton at which to stop the vertical 
        %                      trajectory and grasp the ball
        alignment_point = pos;
        alignment_point(3) = nvargs.zoffset;
        self.blocking_ts_move(alignment_point);
        self.open_gripper();

        pick_up_point = pos;
        pick_up_point(3) = nvargs.z_ball;
        self.blocking_ts_move(pick_up_point);
        self.close_gripper();
        
        self.blocking_ts_move(alignment_point);

        switch color
            case "green"
                drop_point = [100, -200, nvargs.z_lift, -90];
            case "red"
                drop_point = [50, -200, nvargs.z_lift, -90];
            case "yellow"
                drop_point = [0, -200, nvargs.z_lift, -90];
            case "orange"
                drop_point = [-50, -200, nvargs.z_lift, -90];
            otherwise
                Error("invalid color")
        end
        
        self.blocking_ts_move(drop_point);

        drop_point(3) = nvargs.z_drop;
        self.blocking_ts_move(drop_point);
        self.open_gripper();

        drop_point(3) = nvargs.z_lift;
        self.blocking_ts_move(drop_point);
    end

    function go_home(self)
        self.writeTime(2);
        self.writeJoints([0 0 0 0]);
        pause(2);
    end

    end % end methods

end % end class 
