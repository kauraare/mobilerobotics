function [ranges, angles] = DetectPoles(scan)
%   call by using [xs,ys] = DetectPoles(scan)
%   takes in scan and returns the ranges and angles to the poles detected
%   as two arrays
    sorted=sort(scan.reflectances,'descend');
    threshold = sorted(50)+500;
    [~,locs] = findpeaks(scan.reflectances, 'MinPeakHeight',threshold);
    
    angles = linspace(scan.start_angle, scan.start_angle+scan.step_size*(length(scan.reflectances)-1), length(scan.reflectances));
    
    ranges = scan.ranges(locs)';
    angles = angles(locs);
%     convert to radians and correct so straight ahead is given a bearing
%     of zero
    angles = angles*pi/180 - pi
end