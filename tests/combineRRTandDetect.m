%TEST_POLE_DETECTOR Tests the pole detection and SLAM part of robot
%   Detailed explanation goes here

% Change this parameter to toggle between testing the robot online and
% using prerecorded data.
online = 0;
% Connect to robot
clear mexmoos;

husky_id = 4; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);
if online == 1
    client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
    mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
    mexmoos('REGISTER', config.laser_channel, 0.0);
    mexmoos('REGISTER', config.stereo_channel, 0.0);
    mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
    
    mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client, 'SERVERPORT','9000');
    pause(4.0); % give mexmoos a chance to connect (important!)
end
% initialise state and covariance vector
state_vector = [0 0 0];
covariance_matrix = eye(3);
while true
    if online
        "Testing Robot in Online Model"
        mailbox = mexmoos('FETCH');
        scan = GetLaserScans(mailbox, config.laser_channel, true);
        stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
        
    else
        "Testing Robot in Offline mode"
        image_number = string(578);
        scan_path = "data/pole_data/737488."+ image_number +"_scan.mat";
        image_path = "data/pole_data/737488."+image_number+"_images.mat";
        scan = load(scan_path);
        scan = scan.scan;
        stereo_images = load(image_path);
    end
    
    %      get ranges and angles of the poles
    [ranges, angles] = DetectPoles(scan);
    
    
    %      now pass these ranges and angles to SLAM
    [state_vector, covariance_matrix] = SLAMUpdate([0 0 0]', [ranges;angles], state_vector', covariance_matrix);
    
    target = [3 -.5];
    
% %     plot target location
    scatter(target(1),target(2),'r')
    hold on
    
    carrots = RRTStar(target',state_vector');
    
end