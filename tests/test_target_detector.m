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
state_vector = [0 4 0];
covariance_matrix = eye(3);
while true
     if online
        "Testing Robot in Online Model"
        mailbox = mexmoos('FETCH');
     	scan = GetLaserScans(mailbox, config.laser_channel, true);
        stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
     else
        "Testing Robot in Offline mode"
        image_number = 5826;
        scan = load("data/target_data/737488."+image_number+"_scan.mat");
        scan = scan.scan;
        load("data/target_data/737488."+image_number+"_images.mat");
     end
     
     % test target_detectors
%      left image
     figure(1)
     subplot(1,2,1)
     imshow(undistorted_stereo_images.left.rgb)
     img_size = size(undistorted_stereo_images.left.rgb);
     img_x_size = img_size(2);
     img_y_size = img_size(1);
     hold on 
     left_coords = FindTarget(undistorted_stereo_images.left.rgb);
     scatter(left_coords(1)+img_x_size/2, -left_coords(2)+img_y_size/2)
     
%      right image
     subplot(1,2,2)
     imshow(undistorted_stereo_images.right.rgb)
     hold on 
     right_coords = FindTarget(undistorted_stereo_images.right.rgb);
     img_size = size(undistorted_stereo_images.right.rgb);
     scatter(right_coords(1)+img_x_size/2, -right_coords(2)+img_y_size/2)
     
     target_location = TargetDetector(config, undistorted_stereo_images);
     
     [state_vector, covariance_matrix] = SLAMUpdate([0,0,0]',[target_location'],state_vector',covariance_matrix);
%      depth_estimate = 
%      plot own position and the target
     figure(2)
     scatter(state_vector(1),state_vector(2),[],'g')
     hold on 
     
     for i = 4:2:size(state_vector)
          scatter(state_vector(i),state_vector(i+1),[],'r');
          hold on
     end
      
     axis([-5 5 3 10])
break
     
end 
