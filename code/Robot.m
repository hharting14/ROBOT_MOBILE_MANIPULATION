classdef Robot
    
    properties
        
        % For the mobile part:
        l               
        w               
        r               % wheels radius
        psi             % wheels angles
        q               % shape [phi,x,y]              
        Vb              
        u               
        
        % For the 5R arm robot:
        theta           
        theta_dot       
        M_be            
        Blists          
        T_be            
        T_se            
        Trajectory      
        
        % For controller:
        kp
        ki
    end
    
    properties(Dependent)
        config         
        H_0             
        H_phi 
        Je
    end
    
    properties(Constant)
        h = 0.0963;     
    end
    
    methods
        
        % Robot constructor:
        function obj = Robot(wheel_radius, half_width, half_length, T_b0, M_0e, Blists, thetalist)            
            obj.l = half_length;
            obj.w = half_width;
            obj.r = wheel_radius;
            obj.Blists = Blists;
            
            if nargin < 7
                obj.M_be = T_b0 * M_0e;
            else 
                obj.theta = thetalist;
                obj.M_be = T_b0 * M_0e;
                
                % Arm robot kinematics: Tbe
                obj.T_be = FKinBody(obj.M_be, Blists, thetalist);
            end
        end
                                  
        % Gets the configuration of the robot and sets SE(3) matrix:
        function val = get.config(obj)
           R = Rotz(obj.q(1));                % Rotation matrix (phi)
           p = [obj.q(2), obj.q(3), obj.h]';  % Position vector (x, y, z)
           val = RpToTrans(R,p);              % SE(3) matrix
        end
        
        % Sets u value for the robot:
        function obj = set.u(obj,val)
             obj.u = val;
        end
        
        % Calculates H(0). Later... u = H(0)Vb
        function val = get.H_0(obj)
           val = (1/obj.r) * [-obj.l - obj.w, 1, -1;...
                               obj.l + obj.w, 1, 1;...
                               obj.l + obj.w, 1, -1;...
                              -obj.l - obj.w, 1, 1];
        end
        
        % Calculates H(phi):
        function val = get.H_phi(obj)
           val = (1/obj.r) * [-obj.l - obj.w, 1, -1;...
                               obj.l + obj.w, 1, 1;...
                               obj.l + obj.w, 1, -1;...
                              -obj.l - obj.w, 1, 1]...
                           * [1, 0, 0;...
                              0, cos(obj.q(3)), sin(obj.q(3));...
                              0,-sin(obj.q(3)), cos(obj.q(3))];
        end
        
        % Calculates Jacobian:
        function val = get.Je(obj)
            J_arm = JacobianBody(obj.Blists, obj.theta);  % Getting J_arm
            F = pinv(obj.H_0, 0.0001);                    % F matrix
            F6 = [0, 0, 0, 0; 0, 0, 0, 0; F; 0, 0, 0, 0]; % F6 matrix
            J_base = Adjoint(TransInv(obj.T_be)) * F6;    % Getting J_base
            val = [J_base, J_arm];
        end
        
        % Forward kinematics
        function obj = forwardkinematic(obj)
            val = FKinBody(obj.M_be, obj.Blists, obj.theta);
            obj.T_be = val;                                  % Tbe matrix
            obj.T_se = obj.config * val;                     % Tse matrix
        end
        
        % Calculates trajectory:
        function Trajectory = cell2traj(obj, grasp_flag)
            
            % Cell array for the trajectory:
            vert_size = numel(obj.Trajectory);     
            horz_size = 13;
            Trajectory = zeros(vert_size, horz_size);
            
            % Setting the trajectory data for each step:
            for i = 1 : vert_size
                Rp = obj.Trajectory{i}(1:3, 1:4);
                R = transpose(Rp(1:3, 1:3));
                
                % Setting the correct dimensions for the output trajectory:
                Trajectory(i,:) = [reshape(R, 1, []), transpose(Rp(1:3, 4)), grasp_flag];
            end
        end       
        
        function Traj = TrajectoryGenerator(obj, Tstart, Tend, T, dt,...
                grasp_flag, method, scaling_order)
            
            % Rounding the number of points for the trajectory:
            N = round(T/dt + 1);
            
            if nargin < 7
                if strcmp(method, 'Screw')
                    obj.Trajectory = ScrewTrajectory(Tstart, Tend, T, N, 3);
                elseif strcmp(method, 'Cartesian')
                    obj.Trajectory = CartesianTrajectory(Tstart, Tend, T, N, 3);
                end
                
            else
                 if strcmp(method, 'Screw')
                    obj.Trajectory = ScrewTrajectory(Tstart, Tend, T, N,...
                        scaling_order);
                 elseif strcmp(method, 'Cartesian')
                    obj.Trajectory = CartesianTrajectory(Tstart, Tend, T, N,...
                        scaling_order);
                 end               
            end
            
            % Form the desired trajectory parameter: 
            Traj = cell2traj(obj, grasp_flag);                     
        end
         
        function obj = nextStateWheeled(obj, u, dt, maxSpeed)
            
            % Arbitrary wmax: 12.3 rads/s
            u(find(u >= maxSpeed) == 1) = 12.3;
            
            % Odometry - mobile robot:
            obj.Vb = pinv(obj.H_0) * u';                       % Vb
            Vb6 = [0, 0, obj.Vb(1), obj.Vb(2), obj.Vb(3), 0]'; % Vb6 
            T_bk_bknext = MatrixExp6(VecTose3(Vb6 * dt));      % delta Tb
            T_s_bknext = obj.config * T_bk_bknext;             % Tsb[k+1]
            
            % Assigning new q = [phi, x, y]
            obj.q = [atan2(T_s_bknext(2,1), T_s_bknext(1,1)),...
                           T_s_bknext(1,4), T_s_bknext(2,4)];
                        
            % Updating wheel angle
            obj.psi = obj.psi + u * dt;     
        end
                    
        % Odometry - arm robot:    
        function obj = nextState(obj, config, speed, dt, maxSpeed)
            
            % Doing wheeled odometry first:
            obj = nextStateWheeled(obj, speed(1:4), dt, maxSpeed(1:4));
            
            % Evaluate if there's any speed greater than maxSpeed:
            speed = speed(5:9);
            maxSpeed = maxSpeed(5:9);
            speed(find((speed >= maxSpeed) == 1)) = 12.3;
            
            % Updating arm configuration:
            obj.theta = config(4:8) + speed * dt; 
            obj.theta = obj.theta';
        end
               
        function [Animation, Xerr, obj] = FeedbackControl(obj, dt, Td,...
               maxspeed, grasp, jointLimits)
           
           % Initializing variables:
           N = size(Td, 3);
           Animation = zeros(N, 13);
           Xerr = zeros(6, N-1);
           V = zeros(6, N-1);
           
           % 3 pos coordinates + 5 joints + 4 wheel angles + 1 grasp
           Animation(1,:) = [obj.q, obj.theta', obj.psi, grasp(1)];
           for i = 1: N-1
               
               % Perform forward kinematics:                
               obj = obj.forwardkinematic;
               
               % Current configuration of the end-effector:
               Tse = obj.T_se;
               obj.T_be = FKinBody(obj.M_be, obj.Blists, obj.theta);
               Xd = (1/dt) * se3ToVec(MatrixLog6(TransInv(Td(:,:,i)) * Td(:,:,i+1)));
               
               % Calculating error:
               xerr = se3ToVec(MatrixLog6(TransInv(Tse) * Td(:,:,i)));
               Xerr(:,i) = xerr;
               
               % Error integration:
               Xerr_sum = sum(Xerr, 2);
               
               % Velocity input control (Feedforward + PI Feedback):
               V(:,i) = Adjoint(TransInv(Tse) * Td(:,:,i)) * Xd...
                   + obj.kp * xerr + obj.ki * Xerr_sum * dt;
               
               % Speed vector = [u, theta_dot]'
               speed = pinv(obj.Je, 0.0090) * V(:,i); 
               obj.u = speed(1:4);
               obj.theta_dot = speed(5:9);
               speed = [obj.u', obj.theta_dot'];
               config = [obj.q, obj.theta', obj.psi];
               
               % Testing joint limit to avoid collisions:
               Je_new = testJointLimits(obj, jointLimits, speed, config, dt, maxspeed);
               speed = pinv(Je_new, 0.0090) * V(:,i); 
               obj.u = speed(1:4);
               obj.theta_dot = speed(5:9);
               speed = [obj.u', obj.theta_dot'];
               
               % Updating robot properties:
               obj = obj.nextState(config, speed, dt, maxspeed); 
               Animation(i+1,:) = [obj.q, obj.theta', obj.psi, grasp(i)];
           end
        end
       
        function Je = testJointLimits(obj, jointLimits, speed, config, dt, maxspeed)
           
           % Testing values:
           obj_test = obj.nextState(config, speed, dt, maxspeed);
           num_joint = size(jointLimits, 2);
           Je = obj.Je;
           theta_test = obj_test.theta';
           
           % If the value is not in the allowed interval setting to zero:
           for i = 1:num_joint
               if theta_test(i) > jointLimits(1,i)||theta_test(i) < jointLimits(2,i)
                   Je(:,i+4) = zeros(6,1);
               end                   
           end
        end      
    end
end
