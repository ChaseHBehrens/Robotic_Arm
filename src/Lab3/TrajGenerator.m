classdef TrajGenerator < handle  
    methods(Static)
        % TODO: Fill in the arguments for these methods
        function coeffs = cubic_traj(qs, t0, t1)
        %CUBIC_TRAJ Generates coefficients for a cubic trajectory
        % Inputs:
        %   q0 = initial position
        %   q1 = final position
        %   t = change in time
        % Outputs:
        %   coeffs: a [1x4] matrix of cubic trajectory coefficients

        cubic_mat = [1 t0 t0^2   t0^3;
                     1 t1 t1^2   t1^3;
                     0  1 2*t0 3*t0^2;
                     0  1 2*t1 3*t1^2;];
        
        coeffs = transpose(cubic_mat\qs);

            % Hint: to solve the linear system of equations b = Ax,
            %       use x = A \ b
        end

        % TODO: Fill in the arguments for these methods
        function coeffs = quintic_traj(qs, t0, t1)
        %CUBIC_TRAJ Generates coefficients for a quintic trajectory
        % Inputs:
        %   TODO: Describe input args
        % Outputs:
        %   coeffs: a [1x6] matrix of cubic trajectory coefficients

        quintic_mat = [1 t0 t0^2   t0^3    t0^4    t0^5;
                       1 t1 t1^2   t1^3    t1^4    t1^5;
                       0  1 2*t0 3*t0^2  4*t0^3  5*t0^4;
                       0  1 2*t1 3*t1^2  4*t1^3  5*t1^4;
                       0  0    2   6*t0 12*t0^2 20*t0^3;
                       0  0    2   6*t1 12*t1^2 20*t1^3;];
        
        coeffs = transpose(quintic_mat\qs);

        end

        function state = eval_traj(coeff_mat, t) 
        %EVAL_TRAJ Evaluates multiple trajectories
        % Inputs:
        %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
        %              cubic or quintic trajectory coefficients
        %   t: a time in seconds at which to evaluate the trajectories
        % Outputs:
        %   state: a [nx1] column vector containing the results of 
        %          evaluating the input trajectories at time t
            state = zeros(size(coeff_mat,1), 1);
            t0 = 0;
            t1 = t;
            if (size(coeff_mat,2) == 4)
                mat = [1 t0 t0^2 t0^3;
                       1 t1 t1^2 t1^3;
                       0  1 2*t0 3*t0;
                       0  1 2*t1 3*t1;];
            elseif (size(coeff_mat,2) == 6)
                mat = [1 t0 t0^2   t0^3    t0^4    t0^5;
                       1 t1 t1^2   t1^3    t1^4    t1^5;
                       0  1 2*t0 3*t0^2  4*t0^3  5*t0^4;
                       0  1 2*t1 3*t1^2  4*t1^3  5*t1^4;
                       0  0    2   6*t0 12*t0^2 20*t0^3;
                       0  0    2   6*t1 12*t1^2 20*t1^3;];
            end
            for i = 1:1:size(coeff_mat,1)
                output = mat*transpose(coeff_mat(i,:));
                state(i) = output(2);
            end
        end
    end
end