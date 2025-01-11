function [x, y, theta, r] = ConvertFrenetToCartesian_R(s,l)
if (s <= 50)
    xr = s;
    yr = 0;
    theta = 0;
    r = 1000000; 
elseif ((s > 50)&&(s <= 50 + pi * 5))
    ang = (s - 50) / 10;
    xr = 50 + 10 * cos(pi/2 - ang);
    yr = -10 + 10 * sin(pi/2 - ang);
    theta = -ang;
    r  = 10; 
    r = r-l;
elseif ((s > 50 + pi * 5)&&(s <= 90 + pi * 5))
    ds = s - 50 - pi * 5;
    theta = -0.5 * pi;
    xr = 60;
    yr = -10 - ds;
    r = 1000000; 
elseif ((s > 90 + pi * 5)&&(s <= 90 + pi * 10))
    ds = s - 90 - pi * 5;
    ang = ds / 5;
    theta = -0.5 * pi + ang;
    xr = 65 + 5 * cos(pi + ang);
    yr = -50 + 5 * sin(pi + ang);
    r = 5; 
    r = r+l;
elseif (s > 90 + pi * 10)
    ds = s - 90 - pi * 10;
    theta = 0.5 * pi;
    xr = 70;
    yr = -50 + ds;
    r = 1000000; 
end
x = xr - l * cos(pi/2 + theta);
y = yr - l * sin(pi/2 + theta);
end