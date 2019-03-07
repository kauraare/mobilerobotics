% function [ranges, angles] = DetectPoles(scan)
% %   call by using [xs,ys] = DetectPoles(scan)
% %   takes in scan and returns the ranges and angles to the poles detected
% %   as two arrays
%     sorted=sort(scan.reflectances,'descend');
%     threshold = sorted(50)+500;
%     [~,locs] = findpeaks(scan.reflectances, 'MinPeakHeight',threshold);
%     
%     angles = linspace(scan.start_angle, scan.start_angle+scan.step_size*(length(scan.reflectances)-1), length(scan.reflectances));
%     
%     ranges = scan.ranges(locs)';
%     angles = angles(locs);
% %     convert to radians and correct so straight ahead is given a bearing
% %     of zero
%     angles = -(angles*pi/180 - pi);
% end

function [filtered_ranges, filtered_angles] = DetectPoles(scan)
%   call by using [xs,ys] = DetectPoles(scan)
%   takes in scan and returns the ranges and angles to the poles detected
%   as two arrays
    range_thresh = 5;
    scan_concat = [scan.reflectances, scan.ranges];
    scan_concat(scan_concat(:,2)>=range_thresh,1) = 0;
%     figure
%     subplot(2,1,1)
%      plot(scan_concat(:,1))
%      subplot(2,1,2)
%      plot(scan_concat(:,2))
    sorted_concat=sortrows(scan_concat, 1, 'descend');
    sorted = sorted_concat(:,1);
    threshold = sorted(50)+350;
    [~,locs] = findpeaks(scan.reflectances, 'MinPeakHeight',threshold);
    
    angles = linspace(scan.start_angle, scan.start_angle+scan.step_size*(length(scan.reflectances)-1), length(scan.reflectances));
    
    ranges = scan.ranges(locs)';
    angles = angles(locs);
    filtered_ranges = ranges(1);
    filtered_angles = angles(1);
    range_nearness_threshold = 0.25;
    angle_nearness_threshold = 15;
    for i = 2:1:size(ranges,2)
        if all(abs(filtered_ranges - ranges(i)) > range_nearness_threshold) || ...
                all((filtered_angles - angles(i)) > angle_nearness_threshold)
            filtered_ranges = [filtered_ranges, ranges(i)];
            filtered_angles = [filtered_angles, angles(i)];
        end
    end
%     convert to radians and correct so straight ahead is given a bearing
%     of zero
    filtered_angles = -(filtered_angles*pi/180 - pi);
end
