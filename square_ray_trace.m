function [len] = square_ray_trace (x, y, theta, offset, square_size)
    t = mod(theta+offset, 2*pi);
    if(t >= 0 && t <= pi/2)
        len = min((square_size-x)/cos(t), (square_size-y)/cos(t-pi/2));
    elseif(t >= pi/2 && t <= pi)
        len = min((square_size-y)/cos(t-pi/2), (square_size+x)/cos(t-pi));
    elseif(t >= pi && t <= 3*pi/2)
        len = min((square_size+x)/cos(t-pi), (square_size+y)/cos(t-3*pi/2));
    else
        len = min((square_size+y)/cos(t-3*pi/2), (square_size-x)/cos(t));
    end
end