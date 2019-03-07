online = 1;
% Connect to robot
clear mexmoos;

husky_id = 4; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);
config.host = '192.168.0.14';

client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client, 'SERVERPORT','9000');
pause(4.0); % give mexmoos a chance to connect (important!)

% First tell it not to move at all
% SendSpeedCommand(0, 0, config.control_channel)
found_target_flag = 0;
counter = 1;
state_vector = [0; 4; 0];
state_cov = ones(3,3);
accum_time = 1e5;
req_new_carrot = 1;
plan_flag = 1;
target_location = Local2Global([0;0;0],[3;0;0]);

% the route we want to take
trajectories = zeros(3,2,3);
trajectories(:,:,1) = [[0,6];[1,5];[1,5]];
trajectories(:,:,2) = [[0,6];[1,5];[0,5]];
trajectories(:,:,3) = [[0,6];[1,5];[0,5]];
traj_counter = 1;

while true
    % Fetch latest messages from mex-moos
    pause(0.25)
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
    wheel_odometry_all = GetWheelOdometry(mailbox, ...
                                          config.wheel_odometry_channel, ...
                                          false);
    wheel_odometry = ComposeWheelOdom(wheel_odometry_all);
    
    [state_vector, state_cov] = SLAMUpdate(wheel_odometry, ...
        [5; 0], ...
        state_vector, state_cov);
    
%     if mod(counter, 1e6)
%         plan_flag = 1;
%     end
    
    if req_new_carrot && plan_flag
        carrots = trajectories(:,:,traj_counter);
        carrot = carrots(end,:);
        carrot_num = 1;
        plan_flag = 0;
        traj_counter = traj_counter + 1;
    elseif req_new_carrot
        if carrot_num == size(carrots, 1)
            plan_flag = 1;
        else
            carrot = carrots(end-carrot_num,:);
            carrot_num = carrot_num + 1;
        end
    end
    req_new_carrot = 0;
    
    %   MOVE
    req_new_carrot = TurnDriveTurn(config, state_vector, carrot);
    
    carrot
    state_vector(1:3)'
    
    counter = counter + 1;
    
end