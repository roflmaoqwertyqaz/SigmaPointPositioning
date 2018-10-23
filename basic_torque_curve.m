function t = basic_torque_curve(s)
     if(s < 0)
        t = 1;
     else
         t = 1-s;
     end
end