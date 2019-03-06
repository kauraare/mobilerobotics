% this script creates a virtual world for our robot to navigate.
map_x = 8;
map_y = 8;
pole_width = .25;
pole_height = .25;
current_x = 0;
current_y = 4;
target_x = 7;
target_y = 4;
simulation_time_steps = 100;
current_bearing = pi/2; % will update this over time
view_range = 5; % how far the robot can see
field_of_view = pi/6;

no_of_poles = 10;

poles_positions = rand(no_of_poles,2);
poles_positions(:,1) = poles_positions(:,1)*map_x;
poles_positions(:,2) = poles_positions(:,2)*map_y;

state_vector = [current_x; current_y; current_bearing];
covariance_matrix = eye(3);


for time =1:simulation_time_steps
    clf
    detections = []
%     draw current position and field of view
    viscircles([current_x, current_y],0.5,'Color','r');
    hold on
    line([current_x,current_x+view_range*sin(current_bearing)], [current_y, current_y+view_range*cos(current_bearing)], 'Color', 'r');
    line([current_x,current_x+view_range*sin(current_bearing+field_of_view)],[current_y, current_y+view_range*cos(current_bearing+field_of_view)], 'Color', 'r');
    line([current_x,current_x+view_range*sin(current_bearing-field_of_view)],[current_y, current_y+view_range*cos(current_bearing-field_of_view)], 'Color', 'r');
%     draw target
    viscircles([target_x, target_y],0.5,'Color','g');
    hold on
    
%     plot poles 
    scatter(poles_positions(:,1), poles_positions(:,2))
    % simulated dectector, returns poles within the field of view
    relative_poles_positions = zeros(size(poles_positions));
    relative_poles_positions(:,1) = poles_positions(:,1) - current_x;
    relative_poles_positions(:,2) = poles_positions(:,2) - current_y;
    poles_distances = sqrt(sum(relative_poles_positions.^2,2));
    
    
    current_bearing_vector = [view_range*sin(current_bearing),view_range*cos(current_bearing)]
    % DO SLAM ON DETECTIONS
    MOTION = [0;0;0];
    [state_vector, covariance_matrix] = SLAMUpdate(MOTION, detections', state_vector,covariance_matrix);
    
    
%     draw detected poles
    for i = 4:2:size(state_vector)
        i
        rectangle('Position',[state_vector(i) state_vector(i+1) pole_width pole_height],'FaceColor','r')
    end
    
    
    % ROUTE PLANNING
    target_range = sqrt((target_x-current_x)^2+(target_y-current_y)^2);
    target_bearing = atan((target_x-current_x)/(target_y--current_y))+current_bearing;
    carrots = RRTStar([target_range,target_bearing], state_vector');
    
%     plot(carrot.coord(1), carrot.coord(2), 'x', 'Color',  'r')
%     RRTStar(target_location 

%     move to the next carrot
    current_x = carrots(end,1)
    current_y = carrots(end,2)
    
end


