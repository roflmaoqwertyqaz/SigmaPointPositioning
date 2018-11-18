
figure;
hold on;
majordt = 0.02;
minordt = 0.0001;
state = [-1 0 0 0 0 0 0 0 0];
for i=0:minordt:8
    
    forward = 0;
    
    if(i < 3)
        forward = 1;
    else 
        forward = -1;
    end
    
    state = drivetrain_transform(state, forward, 0.05, minordt,10);
    if(mod(i, majordt) < minordt/2)
        clf();
        hold on;
        draw_robot(state);
        state
        txt = ['t=' num2str(i)];
        text(1.5, 1.5, txt);
        hold off;
        pause(majordt)
    end
end