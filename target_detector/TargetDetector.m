function target_location = TargetDetector(config, undistorted_stereo_images)
%
% target_location = TargetDetector(config, stereo_images)
%
% Get target location from image input.
%
% INPUTS:
%   config - Husky config.
%   stereo_images - raw image input from the cameras.
%
% OUTPUTS:
%   target_location - location of the target as a range and bearing (rad)
%   relative to the robot.


% Undistort the images.
%undistorted_stereo_images = UndistortStereoImage(stereo_images, ...
                                                 %config.camera_model);

% Find the target in each image.
%left_brightness = sqrt(sum(left_img, 3));
%normalised_left_img_red = rescale(left_img(:,:,1)./left_brightness, 0, 255);
baseline = undistorted_stereo_images.baseline;
focal_length = undistorted_stereo_images.left.fx;
left_coord = FindTarget(undistorted_stereo_images.left.rgb);
right_coord = FindTarget(undistorted_stereo_images.right.rgb);

% Do some geometry.
depth_estimate = focal_length * baseline / (left_coord(1) ...
                                            - right_coord(1));

% get angle

angle_estimate_left = atan2(left_coord(1),focal_length);
angle_estimate_right = atan2(right_coord(1),focal_length);
average_angle = 0.5*(angle_estimate_left+angle_estimate_right);

distance_from_stereo = depth_estimate/cos(average_angle);

global_frame_angle = -average_angle + pi/2;
target_location = [distance_from_stereo, global_frame_angle];
