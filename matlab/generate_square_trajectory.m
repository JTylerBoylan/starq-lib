clc
clear
close all
%%

% Params
stride_frequency = 1; % Hz
stride_length = 100; % mm
publish_frequency = 100; % Hz

N = publish_frequency / stride_frequency;
t = linspace(0, 1/stride_frequency, N);

top_left = [-50; 0; -50];
top_right = top_left + [stride_length; 0; 0];
bottom_left = top_left - [0; 0; stride_length];
bottom_right = bottom_left + [stride_length; 0; 0];

Nps = N / 4;
top_traj = [linspace(top_left(1), top_right(1), Nps);
            linspace(top_left(2), top_right(2), Nps);
            linspace(top_left(3), top_right(3), Nps)];
right_traj = [linspace(top_right(1), bottom_right(1), Nps);
              linspace(top_right(2), bottom_right(2), Nps);
              linspace(top_right(3), bottom_right(3), Nps)];
bottom_traj = [linspace(bottom_right(1), bottom_left(1), Nps);
               linspace(bottom_right(2), bottom_left(2), Nps);
               linspace(bottom_right(3), bottom_left(3), Nps)];
left_traj = [linspace(bottom_left(1), top_left(1), Nps);
             linspace(bottom_left(2), top_left(2), Nps);
             linspace(bottom_left(3), top_left(3), Nps)];
traj = [top_traj, right_traj, bottom_traj, left_traj];

figure
title("Position Trajectory")
plot(traj(1,:), traj(3,:))
xlabel("X (mm)")
ylabel("Z (mm)")
axis equal

vel_traj = [diff(traj(1,:))./diff(t);
            diff(traj(2,:))./diff(t);
            diff(traj(3,:))./diff(t)];

figure
title("Velocity Trajectory")
plot(vel_traj(1,:), vel_traj(3,:))
xlabel("Vx (mm/s)")
ylabel("Vz (mm/s)")
axis equal


