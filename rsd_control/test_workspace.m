% Test case 1: link length = 0.2m, 0.2m

% link length
l1 = 0.2;
l2 = 0.22;

% Angle limit
th1_min = -pi;
th1_max = +pi;
th2_min = -pi/3*2;
th2_max = +pi/3*2;

points = 100000;

x_mat = double.empty(1,0);
y_mat = double.empty(1,0);

for i = 1:points
    i
    th1 = th1_min+(th1_max-th1_min)*rand();
    th2 = th2_min+(th2_max-th2_min)*rand();

    T = planar_kine(l1, l2, th1, th2);
    % Extract x position from T
    x_mat(i) = T(1, 4);
    % Extract y position from T
    y_mat(i) = T(2, 4);
end

%%
% plot x_mat and y_mat
plot(x_mat, y_mat, 'ob', 'MarkerFaceColor', 'b');
hold on;
plot(0,0, 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 20);

rectangle('Position', [0.21,-0.1, 0.2, 0.2], 'EdgeColor', 'r', 'LineWidth', 3)
set(gca, 'FontSize', 32, 'FontName', 'Times')

% text(0,0, '\leftarrow origin of manipulator', 'FontSize', 25)
% text(0.41, 0.1, '\leftarrow Given \nworkspace', 'FontSize', 25)
title('Link length l1: '+string(l1)+', l2: '+string(l2))
xlabel('Position x(m)'); ylabel('Position y(m)')