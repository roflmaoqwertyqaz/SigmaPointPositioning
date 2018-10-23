
figure;
hold on;
dt = 0.01;
state = [-2 0 0 0 0 0 0 0 0];
for i=0:dt:1
    clf();
    hold on;
    state = drivetrain_transform(state, 0.5, 0.1, dt);
    draw_robot(state);
    hold off;
    pause(dt)
end