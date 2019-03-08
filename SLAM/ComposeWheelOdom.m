function composed_transform = ComposeWheelOdom(wheel_odometry)
% composed_transform.destination_timestamp = wheel_odometry(1).destination_timestamp;
% composed_transform.source_timestamp = wheel_odometry(end).source_timestamp;

tac = [wheel_odometry(1).x; wheel_odometry(1).y; wheel_odometry(1).yaw];
for i = 2:1:size(wheel_odometry, 2)
    tbc = [wheel_odometry(i).x; wheel_odometry(i).y; wheel_odometry(i).yaw];
    tac = tcomp(tac,tbc);
end

composed_transform = tac;
%     PLEASE CONSULT SAAD BEFORE CONSIDERING CHANGING THIS

composed_transform(3) = -composed_transform(3); % SERIOUSLY


end