function request_new_carrot = TurnDriveTurn(husky_config, state_vector, carrot)
%%% Make the robot move to the carrot.

% Set parameters
angle_threshold = pi/12;
range_threshold = 0.25;
angular_vel = 0.3; % rad/sec
vel = 0.3; % m/s
request_new_carrot = 0;

% Calculate carrot in local polar coordinates
carrot(3) = atan2(carrot(2)-state_vector(2), carrot(1)-state_vector(1)); % align rotation at carrot with displacement
local_carrot = Global2Local(state_vector(1:3), carrot');
delta_angle = local_carrot(3);
delta_range = norm(local_carrot(1:2));


% time_turn = angular_vel / delta_angle;
% time_drive = delta_x / vel;

% GetWheelOdometry(mailbox, config.wheel_odometry_channel, true);
% last_source_timestamp = wheel_odometry.source_timestamp;

% wheel_odometry = GetWheelOdometry(mailbox, ...
%     config.wheel_odometry_channel, ...
%     true);
% if wheel_odometry.source_timestamp ~= last_source_timestamp
%     tbc = [wheel_odometry.x; wheel_odometry.y; wheel_odometry.yaw];
%     tab = tcomp(tab,tbc);
%     delta_angle = local_carrot(3) - tab(3);
%     delta_range = sqrt(local_carrot(1)^2 + local_carrot(2)^2) ...
%         - sqrt(tab(1)^2 + tab(2)^2);
% end

if abs(delta_angle) > angle_threshold
    'Turning'
    if delta_angle > 0
        SendSpeedCommand(0, -angular_vel, husky_config.control_channel);
    else
        SendSpeedCommand(0, angular_vel, husky_config.control_channel);
    end
    pause(0.01);
elseif delta_range > range_threshold
    'Driving forward'
    SendSpeedCommand(vel, 0, husky_config.control_channel);
    pause(0.01);
else
    request_new_carrot = 1;
    SendSpeedCommand(0, 0, husky_config.control_channel);
    pause(0.01);
    return 
end

% last_source_timestamp = wheel_odometry.source_timestamp;
end