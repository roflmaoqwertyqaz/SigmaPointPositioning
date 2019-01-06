
figure;
hold on;

true_state = generate_initial_true_state();
est_state = generate_initial_est_state()

covariance = generate_initial_covariance_matrix();

encoder = [0 0];
for i=0:SigmaPointConstants.sim_dt:25
    
    forward = 0;
    turn = 0;
    
    if(i < 3)
        forward = 1;
        turn = 0.2;
    elseif(i < 5)
        forward = -1;
        turn = 0.2;
    elseif(i < 8)
        forward = 0;
        turn = 0.2;
    elseif(i < 12)
        forward = 0.5;
        turn = -0.2;
    elseif(i < 16)
        forward = 0;
        turn = -0.8;
    elseif(i < 19)
        forward = 1;
        turn = 0;
    elseif(i < 24)
        forward = -1;
        turn = 0;
    else
        forward = 0;
        turn = 0;
    end
    
    [state, encoder_increment] = drivetrain_transform(true_state(1:9)', forward, ...
        turn, SigmaPointConstants.sim_dt,SigmaPointConstants.wheel_noise_std*randn(), ...
        SigmaPointConstants.wheel_noise_std*randn());
    true_state = state'; 
    encoder = encoder + encoder_increment;
    
    if(mod(i, SigmaPointConstants.sample_dt) < SigmaPointConstants.sim_dt/2)
        clf();
        hold on;
        draw_robot(true_state);
        
        sigma_points = generate_sigma_points(est_state, covariance);
        [apriori_state, apriori_measurement, propagated_states, propagated_measurements] = ...
            propagate_and_combine_sigma_points(sigma_points, forward, turn, ...
            SigmaPointConstants.sample_dt);
        apriori_covariance = compute_apriori_covariance(propagated_states, apriori_state);
        state_uncertainty = compute_state_uncertainty(propagated_states, ...
            propagated_measurements, apriori_state, apriori_measurement);
        measurement_uncertainty = compute_measurement_uncertainty(propagated_measurements, ...
            apriori_measurement);
        gains = state_uncertainty/measurement_uncertainty;
        imu_data = sim_imu(true_state, ...
            [SigmaPointConstants.accel_noise_std*randn(), ...
            SigmaPointConstants.accel_noise_std*randn(), ...
            SigmaPointConstants.gyro_noise_std*randn()], ...
            SigmaPointConstants.imu_bias);
        rangefinder_data = sim_rangefinders(true_state, ...
            [SigmaPointConstants.rangefinder_noise_std*randn(), ...
            SigmaPointConstants.rangefinder_noise_std*randn(), ...
            SigmaPointConstants.rangefinder_noise_std*randn()], ...
            SigmaPointConstants.rangefinder_bias, ...
            SigmaPointConstants.rangefinder_angle_offset);
        true_measurements = [encoder imu_data rangefinder_data]';
        measurement_error = true_measurements-apriori_measurement;
        est_state(1:9) = apriori_state+gains*measurement_error;
        covariance(1:9, 1:9) = apriori_covariance-gains*measurement_uncertainty*gains.'
        
        draw_robot(est_state(1:9)');
        encoder = [0 0];
        txt = ['t=' num2str(i)];
        text(1.5, 1.5, txt);
        hold off;
        pause(SigmaPointConstants.sample_dt)
    end
   
end
    legend();