function request_new_carrot = TurnDriveTurn(husky_config, state_vector, carrot)
% Make the robot move to the carrot.
angle_threshold = pi/12;
range_threshold = 0.25;

carrot(3) = atan(carrot(2) / carrot(1));
local_carrot = Global2Local(state_vector(1:3), carrot');

angular_vel = pi/6; % rad/sec
vel = 0.1; % m/s

% delta_angle = local_carrot(3);
% delta_x = sqrt(local_carrot(1)^2 + local_carrot(2)^2);
% 
% time_turn = angular_vel / delta_angle;
% time_drive = delta_x / vel;

% GetWheelOdometry(mailbox, config.wheel_odometry_channel, true);
% last_source_timestamp = wheel_odometry.source_timestamp;

delta_angle = local_carrot(3);
delta_range = sqrt(local_carrot(1)^2 + local_carrot(2)^2);
request_new_carrot = 0;

%     wheel_odometry = GetWheelOdometry(mailbox, ...
%                                       config.wheel_odometry_channel, ...
%                                       true);
%     if wheel_odometry.source_timestamp ~= last_source_timestamp
%         tbc = [wheel_odometry.x; wheel_odometry.y; wheel_odometry.yaw];
%         tab = tcomp(tab,tbc);
%         delta_angle = local_carrot(3) - tab(3);
%         delta_range = sqrt(local_carrot(1)^2 + local_carrot(2)^2) ...
%                       - sqrt(tab(1)^2 + tab(2)^2);
%     end

if delta_angle > angle_threshold
    'Turning'
    SendSpeedCommand(0, angular_vel, husky_config.control_channel);
    pause(0.01);
elseif delta_range > range_threshold
    'Driving'
    SendSpeedCommand(vel, 0, husky_config.control_channel);
    pause(0.01);
else
    request_new_carrot = 1;
    SendSpeedCommand(0, 0, husky_config.control_channel);
    pause(0.01);
    return

end
%     last_source_timestamp = wheel_odometry.source_timestamp;


end