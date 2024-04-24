function [output] = rmc_project(input_angles)
    % Extracting joint angles from the input vector
    theta1 = input_angles(1);  % Joint angle 1
    theta2 = input_angles(2);  % Joint angle 2
    theta3 = input_angles(3);  % Joint angle 3

    % Defining the lengths of each link in the robot arm
    link1_length = 10;  % Length of the first link
    link2_length = 10;  % Length of the second link
    link3_length = 10;  % Length of the third link
    total_length = link1_length + link2_length + link3_length;  % Total length of the arm

    % Defining a base for plotting (origin)
    base_z = 0:0.1:1;   % Z-coordinates of base points
    base_x = base_z * 0;  % X-coordinates of base points (all zeros)
    base_y = base_z * 0;  % Y-coordinates of base points (all zeros)

    % Plotting the base points
    plot(base_x, base_y, 'k')  % 'k' stands for black color
    hold on;  
    grid on;  

    % Calculating transformation matrices using the joint angles
    T01 = compute_transformation(0, 0, 0, theta1);  % Transformation from base to first joint
    T12 = compute_transformation(0, link1_length, 0, theta2);  % Transformation from first to second joint
    T23 = compute_transformation(0, link2_length, 0, theta3);  % Transformation from second to third joint

    % Calculating the end position of the robot arm in the 2D plane
    end_x = link1_length * cos(theta1) + link2_length * cos(theta1 + theta2) + link3_length * cos(theta1 + theta2 + theta3);
    end_y = link1_length * sin(theta1) + link2_length * sin(theta1 + theta2) + link3_length * sin(theta1 + theta2 + theta3);
    end_angle = wrapToPi(theta1 + theta2 + theta3);  % Normalize angle to the range [-π, π]

    % First link
    first_link_start = T01 * [0 0 0 1]';
    first_link_end = T01 * [link1_length 0 0 1]';
    first_link_X = [first_link_start(1), first_link_end(1)];
    first_link_Y = [first_link_start(2), first_link_end(2)];
    plot(first_link_X, first_link_Y, '-o', 'Color', [1 0.5 0], 'LineWidth', 5);  % Orange

    % Second link
    second_link_start = T01 * T12 * [0 0 0 1]';
    second_link_end = T01 * T12 * [link2_length 0 0 1]';
    second_link_X = [second_link_start(1), second_link_end(1)];
    second_link_Y = [second_link_start(2), second_link_end(2)];
    plot(second_link_X, second_link_Y, '-o', 'Color', [0 0.5 0], 'LineWidth', 5);  % Green

    % Third link
    third_link_start = T01 * T12 * T23 * [0 0 0 1]';
    third_link_end = T01 * T12 * T23 * [link3_length 0 0 1]';
    third_link_X = [third_link_start(1), third_link_end(1)];
    third_link_Y = [third_link_start(2), third_link_end(2)];
    plot(third_link_X, third_link_Y, '-o', 'Color', [0 0 1], 'LineWidth', 5);  % Blue

    % Setting the plot axis limits based on the total length of the arm
    axis([-total_length*1.1 total_length*1.1 -total_length*1.1 total_length*1.1]);

    hold off;  % Release the plot hold
    output = [theta1, theta2, theta3, end_x, end_y, end_angle];  % Output data
end

function T = compute_transformation(alpha, a, d, th)
    % Calculating a 4x4 transformation matrix based on input parameters
    T = [cos(th) -sin(th) 0 a;
         sin(th)*cos(alpha) cos(th)*cos(alpha) -sin(alpha) -sin(alpha)*d;
         sin(th)*sin(alpha) cos(th)*sin(alpha) cos(alpha) cos(alpha)*d;
         0 0 0 1];  % Standard format for a homogeneous transformation matrix
end
