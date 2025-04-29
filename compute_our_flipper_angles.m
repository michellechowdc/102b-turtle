% link to paper: https://www.nature.com/articles/s41598-022-21459-y

%%%%%%%%%%% FIRST RUN replicate_flipper_coords.m %%%%%%%%%
% To get xt, yt, zt coordinate vectors (each 1000 x 1 doubles)

% Desired end coordinates copied over from replicate_flipper_coords.m
% computed with SCL = 610 mm, straight carapace length, aka shell length
% to rescale turtle aka get different shell length, change s_d and
% recompute?? for s_f = s_d / 610

% servo configuration order: pitch then yaw then roll
% (matching the paper's labeling is roll (x axis) then yaw (y axis) then pitch (-z axis, offset) )

% servo offset distances, measured as distance between each servo's axis of rotation 
L1 = 64; % mm, base_to_servo1
L2 = 66.2; % servo1_to_2
L3 = 30; % servo2_to_3

% used for offset distance from end of __ servo to end of flipper, mm
% their flipper (in final turtle robot) was 234 mm long
flipper_length = 313;
xt_f = xt + flipper_length;
yt_f = yt + flipper_length;
zt_f = zt + flipper_length;

% initialize servo angles
n = length(xt);
theta_roll = zeros(n, 1);
theta_yaw  = zeros(n, 1);
theta_pitch = zeros(n, 1);

for i = 1:n
    % Get target point
    x_pt = xt_f(i);
    y_pt = yt_f(i);
    z_pt = zt_f(i);
    
    % Calculate roll angle (rotation about Z)
    theta_roll(i) = atan2(y_pt, x_pt);
    
    % Transform point into roll frame
    R_roll = [cos(theta_roll(i)), -sin(theta_roll(i)), 0;
              sin(theta_roll(i)),  cos(theta_roll(i)), 0;
              0,                   0,                  1];
    P1 = R_roll' * ([x_pt; y_pt; z_pt] - [0; 0; L1]);
    
    % Calculate yaw angle (rotation about Y in roll frame)
    theta_yaw(i) = atan2(P1(1), P1(3));  % X and Z of P1
    
    % Transform into yaw frame
    R_yaw = [cos(theta_yaw(i)), 0, sin(theta_yaw(i));
             0,                1, 0;
            -sin(theta_yaw(i)), 0, cos(theta_yaw(i))];
    P2 = R_yaw' * (P1 - [0; 0; L2]);
    
    % Compute pitch angle (rotation about X in yaw frame)
    theta_pitch(i) = atan2(-P2(2), P2(3));  % Y and Z of P2 - offset from yaw axis
end

% convert to degrees
theta_roll_deg = rad2deg(theta_roll);
theta_yaw_deg = rad2deg(theta_yaw);
theta_pitch_deg = rad2deg(theta_pitch);

%% uniform sample every 10th angle
theta_roll_100 = theta_roll(1:10:end);
theta_yaw_100  = theta_yaw(1:10:end);
theta_pitch_100 = theta_pitch(1:10:end);

