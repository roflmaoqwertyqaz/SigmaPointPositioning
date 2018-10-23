
function [] = draw_robot(state)
    axis([-2 2 -2 2]);
    square_corners = RobotConstants.radius*[0 1 1 -1 -1 1 1; 0 0 1 1 -1 -1 0];
    rotation_matrix = [cos(state(3)) -sin(state(3)); sin(state(3)) cos(state(3))];
    square_corners = rotation_matrix*square_corners;
    square_corners = square_corners+repmat([state(1); state(2)], 1, length(square_corners));
    plot(square_corners(1,:), square_corners(2,:));
end