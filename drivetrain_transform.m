function [state, encoder] = drivetrain_transform(laststate,forward_input, turn_input, dt, leftoffset, rightoffset)
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
    leftwheelaccel = (((torque_curve(leftwheelspeed, forward_input+turn_input)+leftoffset)/(RobotConstants.wheel_radius*RobotConstants.mass/2)));
    rightwheelaccel = (((torque_curve(rightwheelspeed, forward_input-turn_input)+rightoffset)/(RobotConstants.wheel_radius*RobotConstants.mass/2)));
    leftaccel = leftwheelaccel*(RobotConstants.wheel_radius);
    rightaccel = rightwheelaccel*(RobotConstants.wheel_radius);
    newleftspeed = leftwheelspeed*RobotConstants.wheel_radius + leftaccel*dt;
    newrightspeed = rightwheelspeed*RobotConstants.wheel_radius + rightaccel*dt;
    encoder(1:2) = [newleftspeed newrightspeed]*dt;
    state(9) = (rightaccel-leftaccel)/(2*RobotConstants.radius);
    
    newaccel = [(leftaccel-rightaccel)/2 + rightaccel; 0];
    state(6) = ((newrightspeed-newleftspeed)/(2*RobotConstants.radius));
    newspeed = [(newleftspeed-newrightspeed)/2 + newrightspeed; 0];
    state(3) = laststate(3) + state(6)*dt;
    back_rotation_matrix = [cos(state(3)) -sin(state(3)); sin(state(3)) cos(state(3))];
    state(7:8) = back_rotation_matrix*newaccel;
    state(4:5) = back_rotation_matrix*newspeed;
    state(1:2) = laststate(1:2)+state(4:5)*dt;
end 