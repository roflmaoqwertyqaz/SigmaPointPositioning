function [imu_data] = sim_imu(state, noises, biases)
    accels = [state(7); state(8)];
    rotation_mat = [cos(state(3)) -sin(state(3)); sin(state(3)) cos(state(3))];
    imu_data(1:2) = (rotation_mat*accels)' + noises(1:2) + biases(1:2);
    imu_data(3) = state(6) + noises(3) + biases(3);
end

