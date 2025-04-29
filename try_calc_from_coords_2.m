% Gimbal arm lengths (updated values)
L1 = 57.0;       % Base to pitch servo
L2 = 115.67;     % Pitch to yaw
L3 = 310.0;      % Yaw to rod tip

% Time vector
t = linspace(0, 2*pi, 1000)';

% Path equations
xt = 117.1 - 182.9*cos(t*1.496) + 42.63*sin(t*1.496) - 13.3*cos(2*t*1.496) - 54.58*sin(2*t*1.496) + ...
     17.38*cos(3*t*1.496) - 8.86*sin(3*t*1.496) - 5.189*cos(4*t*1.496) + 1.512*sin(4*t*1.496) - ...
     2.687*cos(5*t*1.496) - 2.955*sin(5*t*1.496) + 2.653*cos(6*t*1.496) - 2.801*sin(6*t*1.496) - ...
     0.3433*cos(7*t*1.496) - 1.62*sin(7*t*1.496) - 0.08304*cos(8*t*1.496) - 1.225*sin(8*t*1.496);

yt = 303 + 219.6*cos(t*1.496) - 164.3*sin(t*1.496) + 84.45*cos(2*t*1.496) - 65.54*sin(2*t*1.496) + ...
     26.33*cos(3*t*1.496) + 45.71*sin(3*t*1.496) - 11.23*cos(4*t*1.496) + 20.05*sin(4*t*1.496) - ...
     0.5956*cos(5*t*1.496) - 2.631*sin(5*t*1.496) - 2.667*cos(6*t*1.496) + 0.6726*sin(6*t*1.496) + ...
     0.678*cos(7*t*1.496) - 0.7038*sin(7*t*1.496) + 2.542*cos(8*t*1.496) - 0.9941*sin(8*t*1.496);

zt = 262.7 + 11.65*cos(t*1.496) - 63.16*sin(t*1.496) - 33.56*cos(2*t*1.496) + 82.36*sin(2*t*1.496) - ...
     38.04*cos(3*t*1.496) + 23.7*sin(3*t*1.496) - 14.84*cos(4*t*1.496) - 8.802*sin(4*t*1.496) - ...
     7.894*cos(5*t*1.496) - 4.65*sin(5*t*1.496) - 6.884*cos(6*t*1.496) - 6.088*sin(6*t*1.496) + ...
     2.615*cos(7*t*1.496) - 3.547*sin(7*t*1.496) - 1.716*cos(8*t*1.496) + 1.293*sin(8*t*1.496);

% Plotting the full rod tip path
figure;
hold on; grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Gimbal Arm Visualization with Rod Tip Trajectory');
plot3(xt, yt, zt, 'b-', 'LineWidth', 1.2); % Rod tip trajectory

% Plot arm segments every 10 steps
for i = 1:10:length(t)
    tip = [xt(i); yt(i); zt(i)];
    
    % Inverse kinematics to get angles (approximate solution)
    vec = tip - [0;0;0];
    pitchAngle = atan2(vec(2), sqrt(vec(1)^2 + vec(3)^2));
    yawAngle   = atan2(vec(1), vec(3));
    rollAngle  = 0; % Not visualized in position here, tip only

    % Base position
    P0 = [0; 0; 0];
    
    % Pitch servo axis - along Y
    Ry = @(theta) [cos(theta), 0, sin(theta);
                   0,         1, 0;
                  -sin(theta), 0, cos(theta)];
    % Yaw servo axis - along X
    Rx = @(theta) [1, 0, 0;
                   0, cos(theta), -sin(theta);
                   0, sin(theta),  cos(theta)];
    
    % Segment 1: Base to pitch
    P1 = P0 + [0; L1; 0];
    
    % Segment 2: Pitch to yaw
    R_pitch = Ry(pitchAngle);
    P2 = P1 + R_pitch * [0; 0; L2];
    
    % Segment 3: Yaw to rod tip
    R_yaw = Rx(yawAngle);
    P3 = P2 + R_pitch * R_yaw * [0; 0; L3];
    
    % Draw arms
    plot3([P0(1) P1(1)], [P0(2) P1(2)], [P0(3) P1(3)], 'r-', 'LineWidth', 1.5); % base to pitch
    plot3([P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], 'g-', 'LineWidth', 1.5); % pitch to yaw
    plot3([P2(1) P3(1)], [P2(2) P3(2)], [P2(3) P3(3)], 'k-', 'LineWidth', 1.5); % yaw to tip
end

legend('Rod Tip Path', 'Base to Pitch', 'Pitch to Yaw', 'Yaw to Tip');
view(3);
