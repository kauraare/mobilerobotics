function target_location = TargetDetector(config, stereo_images)
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
%   target_location - location of the target as a range and bearing
%   relative to the robot.


% Undistort the images.
undistorted_stereo_images = UndistortStereoImage(stereo_images, ...
                                                 config.camera_model);

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
an
