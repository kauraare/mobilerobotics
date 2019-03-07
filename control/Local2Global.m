function target_global = Local2Global(robot_global, target_local)
% target_local is in form [range,bearing]
%
% robot global is of form [x, y, theta]'
% target global is of form [x, y]'
target_local_cart = [
    target_local(1)*cos(target_local(2));
    target_local(1)*sin(target_local(2))
    ];
x = robot_global(3) - pi/2;
R = [cos(x), -sin(x); sin(x) cos(x)];
displacement_global = R * target_local_cart;
target_global = displacement_global + robot_global(1:2)';
end