function composed_transform = ComposeWheelOdom(wheel_odometry)
% composed_transform.destination_timestamp = wheel_odometry(1).destination_timestamp;
% composed_transform.source_timestamp = wheel_odometry(end).source_timestamp;

tba = [wheel_odometry(1).x; wheel_odometry(1).y; wheel_odometry(1).yaw];
for i = 2:1:size(wheel_odometry, 2)
    tcb = [wheel_odometry(i).x; wheel_odometry(i).y; wheel_odometry(i).yaw];
    tca = tcomp(tcb,tba);
    tba = tca;
end

composed_transform = tba;
composed_transform(3) = -composed_transform(3);

end