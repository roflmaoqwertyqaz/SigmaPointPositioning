function state = drivetrain_transform(laststate, forward_input, turn_input, dt, noise)
    if(~isvector(laststate))
        error('Input state must be a vector.');
    end
    if(length(laststate) ~= 9)
        error('Input state must have 9 dimensions.');
    end
    if(~(dt > 0))
        error('Delta time must be greater than 0');
    end
    state = zeros(1,9);
    rotation_matrix = [cos(-laststate(3)) -sin(-laststate(3)); sin(-laststate(3)) cos(-laststate(3))];
    speedvec = rotation_matrix*[laststate(4);laststate(5)];
    leftwheelspeed = (speedvec(1,:)-RobotConstants.radius*laststate(6))/(RobotConstants.wheel_radius);
    rightwheelspeed = (speedvec(1,:)+RobotConstants.radius*laststate(6))/(RobotConstants.wheel_radius);
    newleftwheelspeed = (leftwheelspeed+((torque_curve(leftwheelspeed, forward_input+turn_input)+noise*randn())/(RobotConstants.wheel_radius*RobotConstants.mass/2))*dt);
    newrightwheelspeed = (rightwheelspeed+((torque_curve(rightwheelspeed, forward_input-turn_input)+noise*randn())/(RobotConstants.wheel_radius*RobotConstants.mass/2))*dt);
    newleftspeed = newleftwheelspeed*(RobotConstants.wheel_radius);
    newrightspeed = newrightwheelspeed*(RobotConstants.wheel_radius);
    
    state(6) = (newleftspeed-newrightspeed)/(-2*RobotConstants.radius);
    
    newspeedvec = [newleftspeed+RobotConstants.radius*state(6); 0];
    state(3) = laststate(3) + state(6)*dt;
    back_rotation_matrix = [cos(state(3)) -sin(state(3)); sin(state(3)) cos(state(3))];
    state(4:5) = back_rotation_matrix*newspeedvec;
    state(1:2) = laststate(1:2)+state(4:5)*dt;
    state(7:8) = (state(4:5)-laststate(4:5))/dt;
    state(9) = (state(6)-laststate(6))/dt;
    
end 