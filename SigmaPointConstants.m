classdef SigmaPointConstants
    properties(Constant)
        %initial state standard deviations
        initial_pos_std = 0
        initial_ang_std = 0.0000*pi/180;
        initial_vel_std = 0
        initial_angvel_std = 0
        initial_accel_std = 0
        initial_angaccel_std = 0
        
        %true noise standard deviations        
        wheel_noise_std = 0.2;
        
        %estimated noise standard deviations
        est_wheel_noise_std = 0.2;
        est_accel_bias_noise_std = 0.001;
        est_gyro_bias_noise_std = 0.001;
        est_rangefinder_bias_noise_std = 0.001;
        
        %true sensor standard deviations
        accel_noise_std = 0.03;
        gyro_noise_std = 0.05;
        rangefinder_noise_std = 0.05;
        
        %estimated noise standard deviations
        est_encoder_noise_std = 0.01;
        est_accel_noise_std = 0.03;
        est_gyro_noise_std = 0.05;
        est_rangefinder_noise_std = 0.05;
        
        %true biases
        imu_bias = [0, 0, 0];
        rangefinder_bias = [-0.1, -0.1, -0.1];
        
        %estimated biases
        est_imu_bias = [0, 0, 0];
        est_rangefinder_bias = [-0.1, -0.1, -0.1];
        
        %sensor configs
        initial_x = -1.5;
        initial_y = 0;
        initial_theta = 0;
        rangefinder_angle_offset = [0, pi/2, 3*pi/4];
        sim_dt = 0.0001;
        sample_dt = 0.02;
        
        %sigma point weight constants
        nx = 9;
        nq = 2;
        nm = 8;
        L = SigmaPointConstants.nx+SigmaPointConstants.nq+SigmaPointConstants.nm;
        ns = 2*SigmaPointConstants.L+1;
            
        alpha = 0.1;
        beta = 2;
        
        kappa = 3-SigmaPointConstants.L;
        lambda = SigmaPointConstants.alpha^2* (SigmaPointConstants.L + SigmaPointConstants.kappa) ...
            - SigmaPointConstants.L;
        gamma = sqrt(SigmaPointConstants.L + SigmaPointConstants.lambda);
        W_m_0 = SigmaPointConstants.lambda / (SigmaPointConstants.L + SigmaPointConstants.lambda);
        W_c_0  = SigmaPointConstants.W_m_0 + 1 - SigmaPointConstants.alpha^2 ...
            + SigmaPointConstants.beta;
        W_i    = 1/(2*(SigmaPointConstants.L + SigmaPointConstants.lambda));
        
        
    end
   
end