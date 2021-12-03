% Capstone Project 6: Mobile Manipulation
clc
clear

% Robot dim:
r = 0.0475;         % wheel radius
l = 0.47/2;         % forward-backward distance between the wheels
w = 0.3/2;          % side-to-side distance between wheels
h = 0.0963;         % arm height

% Kinematics of the robot:
T_b0 = RpToTrans(eye(3),[0.1662,0,0.0026]');
M_0e = RpToTrans(eye(3),[0.033,0,0.6546]');
Blists = [[0,0,1,0,0.033,0]',...
          [0,-1,0,-0.5076,0,0]',...
          [0,-1,0,-0.3526,0,0]'...
          [0,-1,0,-0.2176,0,0]',...
          [0,0,1,0,0,0]'];
  
% Initializing arm joints vector:
theta = [0, 0, 0, 0, 0]';

% Cube initial and end configurations:
T_sc_initial = RpToTrans(eye(3),[1,0,0.025]');
T_sc_goal = RpToTrans(Rotz(-pi/2),[0,-1,0.025]');

% newTask:
% RpToTrans(eye(3),[1,0.5,0.025]');
% RpToTrans(Rotz(-pi/2),[1,-0.5,0.025]');

% Parametrizing:
a = pi/6;
T_ce_standoff = [[-sin(a),0,-cos(a),0]', [0,1,0,0]', [cos(a),0,-sin(a),0]',...
                [0,0,0.25,1]'];
T_ce_grasp = [[-sin(a),0,-cos(a),0]', [0,1,0,0]', [cos(a),0,-sin(a),0]',...
                [0,0,0,1]'];

% End-effector planned configuration (reference) 
T_se_initial = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.5; 0, 0, 0, 1];
T_standoff_initial = T_sc_initial * T_ce_standoff;
T_grasp = T_sc_initial * T_ce_grasp;
T_standoff_final = T_sc_goal * T_ce_standoff;
T_release = T_sc_goal * T_ce_grasp;

% Construct a cell array to store all configurations:
T_configure = {T_se_initial, T_standoff_initial, T_grasp, T_grasp,...
    T_standoff_initial, T_standoff_final, T_release, T_release,...
    T_standoff_final};
disp('Initialization completed');

% Creating robot obj from class:      
MyRobot = Robot(r, w, l, T_b0, M_0e, Blists, theta);
disp('Robot object created!');

% Robot and controller parameters:
dt = 0.01;                         
T_total = 14;                      
T_duration = trajDuration(T_total);         
Traj = [];
grasp_state = 0;
MyRobot.q = [0, 0, 0];
MyRobot.theta = [0, 0, 0.2, -1.67, 0]';
MyRobot.psi = [-pi/4, pi/4, -pi/4, pi/4]; 
MyRobot.kp = 1.5 * eye(6);            % best kp = 1.5 ki = 0
MyRobot.ki = 0.0 * eye(6);            % overshoot kp = 3 ki = 3
maxspeed = 12.3 * ones(1,9);

% Designed values to avoid collision : [max; min]
jointLimits = [[pi,-pi]',[pi,-pi]',[pi,-pi]',[pi,-pi]',[pi,-pi]'];

for i = 1:8
    % Open and close grip according to desied behaviour:
    if i == 3 
        grasp_state = 1;
    elseif i == 7
        grasp_state = 0;
    end
    
    % Generate the trajectory for the robot:
    Trajectory = MyRobot.TrajectoryGenerator(T_configure{i}, T_configure{i+1},...
                                             T_duration(i), dt, grasp_state, 'Screw', 5);
    Traj = [Traj; Trajectory];
end

csvwrite('Trayectory.csv', Traj);
disp('Trajectory generated!');

% Extracting desired trajectory and grasp state to feed the controller:
N = size(Traj, 1);
Td = zeros(4, 4, N);
grasp = zeros(1, N);

for i = 1 : N   
    % Getting desired rotation:
    Rd = reshape(Traj(i,1:9), [3, 3]);
    Rd = Rd';
    
    % Getting desired position:
    Pd = Traj(i,10:12); 
    Pd = Pd';
    
    % Joining into T desired matrix:
    Tdi = RpToTrans(Rd,Pd);
    
    % Appending each trajectory and grasp state:
    Td(:,:,i) = Tdi;
    grasp(i) = Traj(i, 13);
end

% Feedback control for the robot:
[Animation, Xerr] = MyRobot.FeedbackControl(dt, Td, maxspeed, grasp, jointLimits);
disp('Feedback control applied, animation file generated');

% Plotting error twist (actual config - ref config):
p = plot(Xerr','LineWidth', 1.5);
title('Xerr vs Time','FontSize',12,'FontWeight','bold')
xlabel('Time (0.01s)','FontSize',12,'FontWeight','bold');
ylabel('Error Twist','FontSize',12,'FontWeight','bold');
legend(p,'Wx','Wy','Wz','Vx','Vy','Vz')
grid on;

csvwrite('Animation.csv', Animation);
disp('Successfully plot Xerror vs Time');
save('Xerr.mat','Xerr');
disp('Done!');
