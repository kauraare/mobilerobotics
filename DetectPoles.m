function [x,y] = DetectPoles(scan)
    sorted=sort(scan.reflectances,'descend');
    threshold = sorted(50)+500;
    [~,locs] = findpeaks(scan.reflectances, 'MinPeakHeight',threshold);
    
    angles = linspace(scan.start_angle, scan.start_angle+scan.step_size*(length(scan.reflectances)-1), length(scan.reflectances));
    
    ranges = scan.ranges(locs)';
    angles = angles(locs);
   
    x = ranges .* sind(angles);
    y = ranges .* cosd(angles);
end