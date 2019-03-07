function target_local = Global2Local(robot_global, target_global)
angle = target_global(3)-robot_global(3);
displacement = target_global(1:2) - robot_global(1:2);
x = robot_global(3);
R = [cos(x), -sin(x); sin(x) cos(x)];
displacement_rotated = R * displacement;
target_local = [displacement_rotated ; angle];
end