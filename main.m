startup_cdt

clear mexmoos;

husky_id = 4; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);

client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);

mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client, 'SERVERPORT','9000');
pause(4.0); % give mexmoos a chance to connect (important!)

% First tell it not to move at all
SendSpeedCommand(0, 0, config.control_channel)
found_target_flag = 0;
counter = 1;
state_vector = [1; 4; 0];
state_cov = ones(3,3);
accum_time = 1e5;
req_new_carrot = 1;
plan_flag = 1;
target_location = Local2Global([0;0;0],[3;0;0]);  
while true
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);
    disp(wheel_odometry)

    
%   TARGET DETECTION
	found_target = TargetDetector(config, stereo_images);
%     if we have found a target update the target location!
    if ~(isnan(found_target) == 0)
%         Do we need tl_o convert target location into global frame?
        disp("~ FOUND THE TARGET AT ~")
        found_target 
        local_target(1) = found_target(1) * cos(found_target(2));
        local_target(2) = found_target(1) * sin(found_target(2));
        local_target(3) = found_target(2);
        target_location = Local2Global(state_vector(1:3)',local_target');
    end
   
    
%   POLE DETECTION
	[ranges, bearings] = DetectPoles(scan);

%   SLAM
    start_timestamp = wheel_odometry.source_timestamp;
    last_source_timestamp = start_timestamp;
    tab = [0; 0; 0];
    while wheel.odometry.source_timestamp - start_timestamp < accum_time
        if wheel_odometry.source_timestamp ~= last_source_timestamp
            tbc = [wheel_odometry.x; wheel_odometry.y; wheel_odometry.yaw];
            tab = tcomp(tab,tbc);
            wheel_odometry.source_timestamp = last_source_timestamp;
        end
    end
    [state_vector, state_cov] = SLAMUpdate(tab, [ranges, bearings], ...
                                           state_vector, state_cov);
        
%   ROUTE PLANNING
    if mod(counter, 100)
        plan_flag = 1;
    end
    
    if req_new_carrot && plan_flag
        carrots = RRTStar(target_location, state_vector);
        carrot = carrots(end,:);
        carrot_num = 1;
        plan_flag = 0;
    elseif req_new_carrot
        carrot = carrots(end-carrot_num,:);
        carrot_num = carrot_num + 1;
    end

%   MOVE
    req_new_carrot = TurnDriveTurn(config, state_vector, carrot);
    
    counter = counter + 1;
end