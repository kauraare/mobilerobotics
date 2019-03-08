startup_cdt

clear mexmoos;

husky_id = 2; % Modify for your Husky

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
counter = 1;
start_position = [0, 0, 0];
state_vector = start_position';
state_cov = ones(3,3);
accum_time = 1e5;
req_new_carrot = 1;
plan_flag = 1;
got_here = 0;
on_target = 0;
decay = 5; % decay for exponential moving average for target
target_distance_threshold = 0.3; % distance to target when to stop
reached_target = 0;
target_location_array = Local2Global(state_vector',[5;pi/2]);
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
    
    
    if reached_target==0
        %   TARGET DETECTION
        found_target = TargetDetector(config, stereo_images);
        disp("~ FOUND THE TARGET AT ~")
        disp(found_target)
        
        % Append array
        if size(state_vector,1)==1
            target_location_array(:,counter+1) = Local2Global(state_vector(1:3),found_target);
        else
            target_location_array(:,counter+1) = Local2Global(state_vector(1:3)',found_target);
        end
        
        % Find moving average
        local_target = NaN;
        weights = exp(linspace(0, -counter/decay, counter+1));
        weights(any(isnan(local_target),1)) = 0; % if localtarget is nan set weight to 0
        weights(1)=1e-100; % to ensure at least one is positive
        weights=weights/sum(weights);
        target_location = nansum(target_location_array.*weights,2);
        
        % Check if we have reached target
        
        
        if any(isnan(local_target(:,end))) && norm(target_location(1:2)-state_vector(1:2)) < target_distance_threshold
            
            reached_target=1; % close enough to target
        end
    else
        if on_target == 0
            if req_new_carrot == 1 && got_here
                on_target = 1;
            else
                target_location = Local2Global(state_vector(1:3)',[0.5,pi/2]);
                got_here = 1;
            end

        else
            target_location = start_position(1:2)';
        end
    end
    
    %   POLE DETECTION
    [ranges, bearings] = DetectPoles(scan);
    
    %   SLAM
    [state_vector, state_cov] = SLAMUpdate(wheel_odometry, ...
        [ranges;bearings], ...
        state_vector, state_cov);
    
    clf
    scatter(state_vector(1),state_vector(2),[],'g')
    hold on
    scatter(target_location(1), target_location(2), [], 'r')
    hold on
    for i = 4:2:size(state_vector)
        scatter(state_vector(i),state_vector(i+1),[],'b');
        hold on
    end
    axis([-1 7 -3.5 3.5])
    axis equal
    
    %   ROUTE PLANNING
    if mod(counter, 100)==0
        plan_flag = 1;
    end
    
    if req_new_carrot && plan_flag
        carrots = RRTStar(target_location', state_vector');
        carrot = carrots(1,:);
        carrot_num = 1;
        plan_flag = 0;
    elseif req_new_carrot
        carrot_num = carrot_num + 1;
        if carrot_num > size(carrots, 1)
            plan_flag = 1;
        else
            carrot = carrots(carrot_num,:);
        end
        
    else
        'hi';
    end
    
    scatter(carrot(1), carrot(2), [], 'c')
    hold on
    
    %   MOVE
    %     req_new_carrot = TurnDriveTurn(config, state_vector, carrot);
    req_new_carrot = TurnDriveTurn(config, state_vector, carrot);
    counter = counter + 1;
end