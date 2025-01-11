function [sf,lf,vfe] = Cruisingmode(s, l, v0, ind0, n0)
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

%  若自车骑跨车道，前面障碍车不止一辆  ？？？ ？？？ ？？？  可能有Bug
%简单做法是让自车变道，在横向l上与前方直接跟车的坐标完全相等
% s2数组保存对当前时刻前面他车的s方向距离的序号排序，由近到远 
s2 = [];
for ii = 1 : Nobs -j
    i = find( s1 == min(s1) );  
    s1(i) = Inf;
    s2(ii) = i;
end

% ind 保存跟车序号
for ii = 1 : Nobs -j
    ind= s2(ii);
    l1 =  obstacles_{1,ind}.l(ind0);
    if ((abs(l1 - l0)) < vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual)  
           break;
    end  
end

sd = obstacles_{1,ind}.s(ind0)  -  s0;
vf = ( obstacles_{1,ind}.s(2) - obstacles_{1,ind}.s(1) )/t;

if (v0 <= vf  ) 
%  前方障碍车辆比自车速度快，加速
    vterm = min (vf, vmax);
    a = (vterm - v0)/( n0 * t ) ;   
else
 %  前方障碍车辆比自车速度慢，可加速也可减速
      for ii = 1 : n0
             amax0(ii) = 2* ( (vf - v0  )* ii * t + sd - smin )/( ii * t )^2 ;
      end
      
      amin0 = - v0  /( n0 * t ) ;
      if(( min( amax0 ) >=  5 ) )
             a = 5;
      elseif ( 5 > min( amax0 ) >  amin0 )
             a =  min( amax0 );
      else
             a = 0;
      end
end

for ii = 1 : n0
    sf(ii) = s0 + v0 * ii * t +0.5 * a * ( ii * t )^2 ;
%     sf(ii) = s0 + v0 * ii * t ;
%     lf(ii) = l0 ;                                         %%%
end  

%     lf0 = [l0,  0.5*(l0 + l1), l1];
%     t1 = [1, 5, 10];
%     t2 = [1,2,3,4,5,6,7,8,9,10];
%     lf= spline(t1, lf0, t2);

    lf0 = [l00,  0.5*(l00 + l1), l1];
    t1 = [1, 6, 11];
    t2 = [1,2,3,4,5,6,7,8,9,10,11];
    lf= spline(t1, lf0, t2);
    lf(1)= [];

    vfe = v0 + a *  n0 * t ;
%     vfe = v0  ;

end








