function [output] = rmc_project(input)
    % Extract joint angles from input vector
    th1 = input(1);  % Joint angle 1
    th2 = input(2);  % Joint angle 2
    th3 = input(3);  % Joint angle 3

    % Defining the lengths of each link in the robot arm
    L1 = 10;  % Length of the first link
    L2 = 10;  % Length of the second link
    L3 = 10;  % Length of the third link
    P = L1 + L2 + L3;  % Total length of the arm, used for scaling the plot

    % Defining a base for plotting, not necessarily part of the robot
    base.z = 0:0.1:1;   % Z-coordinates of base points
    base.x = base.z * 0;  % X-coordinates of base points (all zeros)
    base.y = base.z * 0;  % Y-coordinates of base points (all zeros)

    % Plotting the base points (currently just at origin)
    plot(base.x, base.y)
    hold on;  
    grid on;  

    % Calculating transformation matrices using the joint angles
    T01 = TRANS(0, 0, 0, th1);  % Transformation from base to first joint
    T12 = TRANS(0, L1, 0, th2);  % Transformation from first to second joint
    T23 = TRANS(0, L2, 0, th3);  % Transformation from second to third joint

    % Calculating the end position of the robot arm in 2D plane
    X = L1 * cos(th1) + L2 * cos(th1 + th2) + L3 * cos(th1 + th2 + th3);  % X coordinate
    Y = L1 * sin(th1) + L2 * sin(th1 + th2) + L3 * sin(th1 + th2 + th3);  % Y coordinate
    ALPHA = wrapToPi(th1 + th2 + th3);  % Wrap angle to [-π, π]

    % Defining start and end points of each link for plotting
    link1.a = T01 * [0 0 0 1]';  % Start point of first link
    link1.b = T01 * [L1 0 0 1]';  % End point of first link

    link2.a = T01 * T12 * [0 0 0 1]';  % Start point of second link
    link2.b = T01 * T12 * [L2 0 0 1]';  % End point of second link

    link3.a = T01 * T12 * T23 * [0 0 0 1]';  % Start point of third link
    link3.b = T01 * T12 * T23 * [L3 0 0 1]';  % End point of third link

    % Ploting each link with specified options
    link1.X = [link1.a(1) link1.b(1)];
    link1.Y = [link1.a(2) link1.b(2)];
    link1.plot = plot(link1.X, link1.Y, '-o', 'Color', [1 0.5 0], 'LineWidth', 5);  % First link in orange

    link2.X = [link2.a(1) link2.b(1)];
    link2.Y = [link2.a(2) link2.b(2)];
    link2.plot = plot(link2.X, link2.Y, '-o', 'Color', [0 0.5 0], 'LineWidth', 5);  % Second link in green

    link3.X = [link3.a(1) link3.b(1)];
    link3.Y = [link3.a(2) link3.b(2)];
    link3.plot = plot(link3.X, link3.Y, '-o', 'Color', [0 0 1], 'LineWidth', 5);  % Third link in blue

    % Setting the plot axis limits based on the total length of the arm
    axis([-P*1.1 P*1.1 -P*1.1 P*1.1]);

    hold off;  % Release the plot hold
    output = [th1, th2, th3, X, Y, ALPHA];  % Output data
end

function T = TRANS(alpha, a, d, th)
    % Calculate a 4x4 transformation matrix based on input parameters
    % Input parameters: alpha (twist angle), a (link length), d (link offset), th (joint angle)
    T = [cos(th) -sin(th) 0 a;
         sin(th)*cos(alpha) cos(th)*cos(alpha) -sin(alpha) -sin(alpha)*d;
         sin(th)*sin(alpha) cos(th)*sin(alpha) cos(alpha) cos(alpha)*d;
         0 0 0 1];  % Standard format for a homogeneous transformation matrix
end


