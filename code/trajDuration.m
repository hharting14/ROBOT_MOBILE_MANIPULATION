function durations = trajDuration(T)
% Calculate T for each step of the trajectory based on the Euclidean
% distance between start and end point

% Getting the variables form work space:
T_se_initial = evalin('base', 'T_se_initial');
T_standoff_initial = evalin('base', 'T_standoff_initial');
T_grasp = evalin('base', 'T_grasp');
T_standoff_final = evalin('base', 'T_standoff_final');
T_release = evalin('base', 'T_release');

% Calculating Euclidean distances between x, y, z coordinates:
d1 = norm(T_se_initial(1:3, 4) - T_standoff_initial(1:3, 4));
d2 = norm(T_standoff_initial(1:3, 4) - T_grasp(1:3, 4));
d5 = norm(T_standoff_initial(1:3, 4) - T_standoff_final(1:3, 4));
d6 = norm(T_standoff_final(1:3, 4) - T_release(1:3, 4));

% Total distances:
d_total = d1 + d2 * 2 + d5 + d6 * 2;

% Durations of each trajectory:

% Opening and closing the gripper: at least 0.625 secs
t3 = 0.63;
t7 = 0.63;

% Initial -> Standoff up init, Standoff up init -> Standoff down init (grasp)
t1 = d1 * (T - t3 - t7) / d_total;  
t2 = d2 * (T - t3 - t7) / d_total;

% Down and up above initial cube config:
t4 = t2;

% Standoff up init -> Standoff up final, Standoff up final-> Standoff down final (grasp)
t5 = d5 * (T - t3 - t7) / d_total;
t6 = d6 * (T - t3 - t7) / d_total;

% Down and up above final cube config:
t8 = t6;

% Setting the whole trajectory durations: 
durations = [t1, t2, t3, t4, t5, t6, t7, t8];

% Round to integer multiple of 0.01:
durations = round(durations * 10^2) / 10^2;           

end

