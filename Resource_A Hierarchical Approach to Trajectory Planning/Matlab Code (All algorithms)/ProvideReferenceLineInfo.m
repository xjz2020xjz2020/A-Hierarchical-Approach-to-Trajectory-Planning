function [xr, yr, lb, rb, theta] = ProvideReferenceLineInfo(s)
lb = -2 - cos(0.2 * s + 2.8166138) * 0.2;   % 道路边界 左界 -2 ± 0.2 
rb = 5 + cos(0.78 * s - 0.8166138) * 0.15;  % 道路边界 右界 5 ± 0.15
% lb = -2.2;   % 道路边界 左界 -2 ± 0.2    % 2022 0 324
% rb = 5.15;  % 道路边界 右界 5 ± 0.15
if ((s >= -10)&&(s <= 40))       % 1）    水平直线
    xr = s;
    yr = 0;
    theta = 0;
end

if ((s > 40)&&(s <= 40 + pi * 5))  % 2） 下行直角转弯，角度pi/2 ，半径10， 
    ds = s - 40;
    ang = ds / 10;
    xc = 40;
    yc = -10;
    xr = xc + 10 * cos(pi/2 - ang);
    yr = yc + 10 * sin(pi/2 - ang);
    theta = -ang;
end

if ((s > 40 + pi * 5)&&(s <= 50 + pi * 5))  % 3） 下行直线  
    ds = s - 40 - pi * 5;
    theta = -0.5 * pi;
    xr = 50;
    yr = -10 - ds;
end

if ((s > 50 + pi * 5)&&(s <= 50 + pi * 10))   % 4）
    ds = s - 50 - pi * 5;                     %    上行回头弯，角度pi ，半径5， 
    ang = ds / 5;
    theta = -0.5 * pi + ang;
    xc = 55;
    yc = -20;
    xr = xc + 5 * cos(pi + ang);
    yr = yc + 5 * sin(pi + ang);
end
if ((s > 50 + pi * 10)&&(s <= 70 + pi * 10))   % 5） 上行直线， 
    ds = s - 50 - pi * 10;                     
    theta = 0.5 * pi;
    xr = 60;
    yr = -20 + ds;
end
if ((s > 70 + pi * 10)&&(s <= 70 + pi * 15))   % 6）   左拐直角，5*pi
    ds = s - 70 - pi * 10; 
    ang = ds / 10; 
    xc = 50;
    yc = 0
    xr = xc + 10 * cos(ang);
    yr = yc + 10 * sin(ang);
    theta = 0.5 * pi + ang ;
end
if (s > 70 + pi * 15)          % 7）   向左直行
    ds = s - 70 - pi * 15; 
    xr = 50 - ds;
    yr = 10;
    theta = pi;
end

end