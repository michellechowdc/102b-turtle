% Arm segment lengths (mm)
L1 = 57.0;     % Base to pitch
L2 = 115.67;   % Pitch to yaw
L3 = 310.0;    % Yaw to rod tip

% Time vector
t = linspace(0, 2*pi, 300);  % Use 300 points for smoother animation

% Target trajectory
xt =  117.1 - 182.9*cos(t*1.496) + 42.63*sin(t*1.496) ...
    - 13.3*cos(2*t*1.496) - 54.58*sin(2*t*1.496) ...
    + 17.38*cos(3*t*1.496) - 8.86*sin(3*t*1.496) ...
    - 5.189*cos(4*t*1.496) + 1.512*sin(4*t*1.496) ...
    - 2.687*cos(5*t*1.496) - 2.955*sin(5*t*1.496) ...
    + 2.653*cos(6*t*1.496) - 2.801*sin(6*t*1.496) ...
    - 0.3433*cos(7*t*1.496) - 1.62*sin(7*t*1.496) ...
    - 0.08304*cos(8*t*1.496) - 1.225*sin(8*t*1.496);

yt =  303 + 219.6*cos(t*1.496) - 164.3*sin(t*1.496) ...
    + 84.45*cos(2*t*1.496) - 65.54*sin(2*t*1.496) ...
    + 26.33*cos(3*t*1.496) + 45.71*sin(3*t*1.496) ...
    - 11.23*cos(4*t*1.496) + 20.05*sin(4*t*1.496) ...
    - 0.5956*cos(5*t*1.496) - 2.631*sin(5*t*1.496) ...
    - 2.667*cos(6*t*1.496) + 0.6726*sin(6*t*1.496) ...
    + 0.678*cos(7*t*1.496) - 0.7038*sin(7*t*1.496) ...
    + 2.542*cos(8*t*1.496) - 0.9941*sin(8*t*1.496);

zt =  262.7 + 11.65*cos(t*1.496) - 63.16*sin(t*1.496) ...
    - 33.56*cos(2*t*1.496) + 82.36*sin(2*t*1.496) ...
    - 38.04*cos(3*t*1.496) + 23.7*sin(3*t*1.496) ...
    - 14.84*cos(4*t*1.496) - 8.802*sin(4*t*1.496) ...
    - 7.894*cos(5*t*1.496) - 4.65*sin(5*t*1.496) ...
    - 6.884*cos(6*t*1.496) - 6.088*sin(6*t*1.496) ...
    + 2.615*cos(7*t*1.496) - 3.547*sin(7*t*1.496) ...
    - 1.716*cos(8*t*1.496) + 1.293*sin(8*t*1.496);

% Inverse Kinematics (estimate pitch and yaw from position)
% Assume:
% - Pitch is elevation angle (XZ)
% - Yaw is rotation around Z axis (XY)
pitch_angles = atan2(zt - L1, sqrt(xt.^2 + yt.^2 - L1^2));
yaw_angles = atan2(yt, xt);

% Animation
figure;
axis equal;
grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Animated Robot Arm Following Path');
xlim([-300 300]);
ylim([0 600]);
zlim([0 500]);
view(3);
hold on;

% Plot full path
plot3(xt, yt, zt, 'Color', [0.7 0.7 0.7], 'LineStyle', '--');

% Initialize graphics objects
hBase = plot3(0,0,0,'ko','MarkerSize',6,'MarkerFaceColor','k');
hLink1 = plot3([0 0], [0 0], [0 0], 'r-', 'LineWidth', 2); % pitch
hLink2 = plot3([0 0], [0 0], [0 0], 'g-', 'LineWidth', 2); % yaw
hTip   = plot3(0,0,0,'bo','MarkerSize',6,'MarkerFaceColor','b');

for i = 1:length(t)
    % Base (origin)
    base = [0; 0; 0];

    % Pitch axis: rotates in XZ plane
    pitch = pitch_angles(i);
    yaw = yaw_angles(i);

    % Pitch link end
    p1 = base + [0; 0; L1];

    % Rotate pitch around X to get direction
    R_pitch = [1 0 0;
               0 cos(pitch) -sin(pitch);
               0 sin(pitch)  cos(pitch)];

    p2 = p1 + R_pitch * [0; 0; L2];

    % Rotate yaw around Z after pitch
    R_yaw = [cos(yaw) -sin(yaw) 0;
             sin(yaw)  cos(yaw) 0;
             0         0        1];

    p3 = p2 + R_pitch * R_yaw * [0; 0; L3];

    % Update links
    set(hLink1, 'XData', [base(1) p1(1)], 'YData', [base(2) p1(2)], 'ZData', [base(3) p1(3)]);
    set(hLink2, 'XData', [p1(1) p2(1)], 'YData', [p1(2) p2(2)], 'ZData', [p1(3) p2(3)]);
    set(hTip,   'XData', p3(1), 'YData', p3(2), 'ZData', p3(3));

    pause(0.01);
end
