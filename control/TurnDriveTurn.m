function TurnDriveTurn(husky_config, state_vector, carrot)
% Make the robot move to the carrot.

carrot(3) = atan(carrot(2) / carrot(1));
local_carrot = Global2Local(state_vector(1:3)', carrot');

angular_vel = pi/18; % rad/sec
vel = 0.1; % m/s

delta_angle = local_carrot(3);
delta_x = sqrt(local_carrot(1)^2 + local_carrot(2)^2);

time_turn = angular_vel / delta_angle
time_drive = delta_x / vel

tic
while true
    if toc > 5
        break
    else
        x = 'hi'
    SendSpeedCommand(0.5, 0, husky_config.control_channel)
end

tic 
while toc < time_turn
    'turn'
    SendSpeedCommand(0, angular_vel, husky_config.control_channel);
    pause(0.01);
end

tic 
while toc < time_drive
    'drive'
    SendSpeedCommand(vel, 0, husky_config.control_channel);
    pause(0.01)
end
    
SendSpeedCommand(0, 0, husky_config.control_channel)


end