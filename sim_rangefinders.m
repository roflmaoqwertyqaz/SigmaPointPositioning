function [rangefinder_data] = sim_rangefinders(state, noises, biases, angles)
    if(length(noises) ~= length(biases) || length(noises) ~= length(angles))
        error('Noise vector, bias vector, and angle offset vector must be the same length.');
    end
    rangefinder_data = zeros(1, length(angles));
    for i=1:length(angles)
        rangefinder_data(i) = square_ray_trace(state(1), state(2), state(3), angles(i), RobotConstants.field_size) ...
            + noises(i)+biases(i); 
    end
end