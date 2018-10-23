function state = drivetrain_transform(laststate, forward_input, turn_input, dt)
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
    
    top_speed = 3;
    top_accel = 20;
    top_turn_rate = 3*pi;
    top_turn_accel = 2*pi;
    
    speed = sqrt(laststate(4)^2 + laststate(5)^2);
    forward_accel = top_accel*forward_input*((top_speed-speed)/top_speed)
    
    turn_accel = top_turn_accel*turn_input*((top_turn_rate-abs(laststate(6)))/top_turn_rate);
    
    state(1) = laststate(1)+laststate(4)*dt+laststate(7)*dt^2;
    state(2) = laststate(2)+laststate(5)*dt+laststate(8)*dt^2;
    state(3) = laststate(3)+laststate(6)*dt+laststate(9)*dt^2;
    state(4) = laststate(4)+laststate(7)*dt;
    state(5) = laststate(5)+laststate(8)*dt;
    state(6) = laststate(6)+laststate(9)*dt;
    state(7) = forward_accel*cos(laststate(3));
    state(8) = forward_accel*sin(laststate(3));
    state(9) = turn_accel;
    
end