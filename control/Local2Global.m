function target_global = Local2Global(robot_global, target_local)
% robot global is of form [x, y, theta]'
% target global is of form [x, y, theta]'
    angle = target_local(3) + robot_global(3);
    x = -robot_global(3);
    R = [cos(x), -sin(x); sin(x) cos(x)];
    displacement_global = R * target_local(1:2);
    target_global = [displacement_global + robot_global(1:2) ; angle]; 
end