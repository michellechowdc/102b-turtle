% turtle body parameters
% full_bodyWeight = 4 / 2.205; % approx. convert lb to kg by dividing by 2.205
% g = 9.81; % m/s^2
% bodyCOM = [0.1, 0.0, 0.05]; % m (x, y, z)

% flipper parameters
flipperSurfArea = 0.02; % m^2
flipperCOM = [0.25, 0.0, 0.0]; % m from servo axis
fluidDensity = 1000; % kg/m^3 (water)

% motor specs: MG 996R servo
% https://towerpro.com.tw/product/mg996r/
stallTorque_kgcm = 9.4; % stall torque at 4.8V
stallTorque_Nm = stallTorque_kgcm * 9.81 / 100; % convert to N·m

servoSpeed_sec_per_60deg = 0.19;  % at 4.8V
servoSpeed_rad_per_sec = deg2rad(60) / servoSpeed_sec_per_60deg;

% Estimate linear tip speed from angular speed
% flipperLength = norm(flipperCOM); % m
flipperLength = 0.310; % m
tipVelocity = servoSpeed_rad_per_sec * flipperLength; % m/s

% Drag-based thrust estimate: F = 0.5 * rho * A * v^2
thrustPerFlipper = 0.5 * fluidDensity * flipperSurfArea * tipVelocity^2; % N
totalThrust = 2 * thrustPerFlipper;

% Required torque to generate that thrust: T = F * r
requiredTorque = thrustPerFlipper * flipperLength; % Nm
feasible = requiredTorque <= stallTorque_Nm;

% Output
fprintf('--- Flipper Thrust Estimation ---\n');
fprintf('Motor stall torque: %.2f Nm\n', stallTorque_Nm);
fprintf('Flipper tip velocity: %.2f m/s\n', tipVelocity);
fprintf('Thrust per flipper: %.2f N\n', thrustPerFlipper);
fprintf('Total thrust (2 flippers): %.2f N\n', totalThrust);
fprintf('Required torque per motor: %.2f Nm\n', requiredTorque);
fprintf('Torque within motor capacity: %s\n', string(feasible));
