function torque = torque_curve(wheel_vel, input)
    torque = input*RobotConstants.wheel_stall_torque*...
        basic_torque_curve(sign(input)*wheel_vel/RobotConstants.wheel_top_angspeed);
end