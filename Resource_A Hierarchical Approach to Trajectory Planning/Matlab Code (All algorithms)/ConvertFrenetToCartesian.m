function [x, y, theta] = ConvertFrenetToCartesian(s,l)
if (s <= 40)
    xr = s;
    yr = 0;
    theta = 0;
elseif ((s > 40)&&(s <= 40 + pi * 5))
    ang = (s - 40) / 10;
    xr = 40 + 10 * cos(pi/2 - ang);
    yr = -10 + 10 * sin(pi/2 - ang);
    theta = -ang;
elseif ((s > 40 + pi * 5)&&(s <= 50 + pi * 5))
    ds = s - 40 - pi * 5;
    theta = -0.5 * pi;
    xr = 50;
    yr = -10 - ds;
elseif ((s > 50 + pi * 5)&&(s <= 50 + pi * 10))
    ds = s - 50 - pi * 5;
    ang = ds / 5;
    theta = -0.5 * pi + ang;
    xr = 55 + 5 * cos(pi + ang);
    yr = -20 + 5 * sin(pi + ang);
elseif ((s > 50 + pi * 10)&&(s <= 70 + pi * 10))
    ds = s - 50 - pi * 10;
    theta = 0.5 * pi;
    xr = 60;
    yr = -20 + ds;
end
if ((s > 70 + pi * 10)&&(s <= 70 + pi * 15))   
    ds = s - 70 - pi * 10; 
    ang = ds / 10; 
    xc = 50;
    yc = 0
    xr = xc + 10 * cos(ang);
    yr = yc + 10 * sin(ang);
    theta = 0.5 * pi + ang ;
end
if (s > 70 + pi * 15)         
    ds = s - 70 - pi * 15; 
    xr = 50 - ds;
    yr = 10;
    theta = pi;
end
x = xr - l * cos(pi/2 + theta);
y = yr - l * sin(pi/2 + theta);
end