function [target_index] = FindTarget(image)
%FINDTARGET Finds the index of the centre of the target in the image.
threshold = 200

sz = size(image);
image(1:sz(1)/2, :, :) = zeros(sz(1)/2,sz(2),sz(3));
filtered_image = imgaussfilt(image, 3);
enhanced_red = filtered_image(:,:,1) - (filtered_image(:,:,2) + ...
                                    filtered_image(:,:,3))/2;
red_scaled = rescale(enhanced_red, 0, 255);
red_scaled(red_scaled<threshold) = 0;
[x, y] = meshgrid(1:sz(2), 1:sz(1));
target_index(1) = mean(x(red_scaled>=threshold));
target_index(2) = mean(y(red_scaled>=threshold));

% imshow(uint8(red_scaled))
% h = gca;
% h.Visible = 'On';
end

