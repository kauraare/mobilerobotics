% In the global frame.
x_bounds = [-1, 7];
y_bounds = [-3.5, 3.5];
robot_pose = [0; 0; 0];
target_position = [6; 0];
% Randomly position poles within 5x5 grid centred on (3, 0).
num_poles = 10;
pole_positions = rand(num_poles, 2);
pole_positions(:,1) = pole_positions(:,1) * 5 + 0.5;
pole_positions(:,2) = pole_positions(:,2) * 5 - 2.5;

% Initialise SLAM.
state_vector = robot_pose;
state_cov = eye(3);

% Initialise wheel_odometry.
wheel_odometry = [0; 0; 0];

% Initialise flags.
counter = 1;
req_new_carrot = 1;
plan_flag = 1;
reached_target = 0;
while true
    % Detect target in global frame. [x; y];
    target_location = target_position;
    
    % Observe poles. [range 1, range 2; bearing 1, bearing 2];
    pole_displacements = pole_positions - (state_vector(1:2) ...
                                           + wheel_odometry(1:2))';
    pole_ranges = sqrt(sum(pole_displacements.^2, 2));
    % pole_angles are angels in global frame between pole position and
    % robot position, without accounting for robot orientation.
    pole_angles = atan2(pole_displacements(:,2), pole_displacements(:,1));
    pole_bearings = pole_angles - state_vector(3);
    pole_observations = [pole_ranges'; pole_bearings'];

    % Plotting
    clf
    viscircles(state_vector(1:2)',0.5,'Color','g');
    hold on
    viscircles(target_position',0.5,'Color','r');
    hold on
    viscircles(pole_positions,0.05*ones(num_poles,1),'Color','b');
    scatter(pole_positions(:,1), pole_positions(:,2))
    hold on
    for i = 4:2:size(state_vector)
        scatter(state_vector(i),state_vector(i+1),[],'r');
        hold on
    end
    
    % SLAM.
    [state_vector, state_cov] = SLAMUpdate(wheel_odometry, ...
                                           pole_observations, ...
                                           state_vector, state_cov);

    % Route planning.
    if mod(counter, 1000) == 0
        plan_flag = 0;
    end
    
    if req_new_carrot && plan_flag
        carrots = RRTStar(target_location, state_vector);
        carrot = carrots(1,:);
        carrot_num = 1;
        plan_flag = 0;
    elseif req_new_carrot
        carrot_num = carrot_num + 1;
        carrot = carrots(carrot_num,:);
    end
    req_new_carrot = 0;
    
    % Move to carrot.
    carrot_displacement = carrot' - state_vector(1:2);
    wheel_odometry(1:2) = carrot_displacement(1:2);
    req_new_carrot = 1;
end