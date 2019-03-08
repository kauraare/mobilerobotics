function request_new_carrot = PID(husky_config, state_vector, carrot)
% Make the robot move to the carrot using PID with P only

% Set parameters
gain_angular = 0.5;
gain_vel = 1;

request_new_carrot = 1; % always request new carrot

% Calculate carrot in local polar coordinates
carrot(3) = atan2(carrot(2)-state_vector(2), carrot(1)-state_vector(1)); % align rotation at carrot with displacement
local_carrot = Global2Local(state_vector(1:3), carrot');
delta_angle = local_carrot(3);
delta_range = norm(local_carrot(1:2));

% Calculate speeds with proportional controller
angular_vel = -gain_angular * delta_angle;
vel = -gain_vel * delta_range;

% Send speed command
SendSpeedCommand(vel, angular_vel, husky_config.control_channel);

end