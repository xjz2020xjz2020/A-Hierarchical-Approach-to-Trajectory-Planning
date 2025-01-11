function [lf,vfe] = Cruisingmode(s, l, ind0, n0)
global obstacles_ vehicle_geometrics_ Nobs
vmax = 25;
smin = 3;   %  与前车的最小安全间距
t=0.05;
s0 = s(ind0);
l0 = l(ind0);
l00 = l(ind0-1);
s1 = [];
j=0;
% 找出当前时刻前面他车的s方向距离
for ii = 1 : Nobs
    ss =  obstacles_{1,ii}.s(ind0);
    if ( s0 >  ss )
        s1(ii) = Inf;
        j=j+1;
    else
        s1(ii) = ss - s0;
    end
end

%  若自车骑跨车道，前面障碍车不止一辆  ？？？ ？？？ ？？？  
%简单做法是让自车变道，在横向l上与前方直接跟车的坐标完全相等
% s2数组保存对当前时刻前面他车的s方向距离的序号排序，由近到远 
s2 = [];
for ii = 1 : Nobs -j
    i = find( s1 == min(s1) );  
    s1(i) = Inf;
    s2(ii) = i;
end

% ind 保存跟车序号
% 寻找最近  且  本车道车辆作为障碍车辆存入 l1
for ii = 1 : Nobs -j
    ind= s2(ii);
    l1 =  obstacles_{1,ind}.l(ind0);
    if ((abs(l1 - l0)) < vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual)  
           break;
    end  
end
%     l1 = l0 + 0.8* (l1 -l0 );
    l1 = l0 + 1.0* (l1 -l0 );
    
%     lf0 = [l0,  0.5*(l0 + l1), l1];
%     t1 = [1, (0.5*n0 +1), (n0 +1)];
%     t2 = [1:(n0 +1)];
%     lf= spline(t1, lf0, t2);
%     lf(1)= lf(1)+0.01;
 
    lf= linspace(l00, l1, (n0 +1));
     lf(1)= [];

    vfe = ( s(ind0 + n0) - s(ind0 + n0 -1) ) / t;

end