function [v, a, phy, w, k, r, min_r, ind, r_temp, minNo] = FinalFormInitialGuess(x, y, theta, tf, lim_r)
% 速度v，加速度a，前轮转角phy，前轮的转向角速度w, 曲率k，曲率半径r，最小半径min_r，小于lim_r的节点号ind，小于lim_r的半径r_mins，最小半径minNo
Nfe = length(x);
v = zeros(1, Nfe);
a = zeros(1, Nfe);
dt = tf / (Nfe - 1);
phy = zeros(1, Nfe);
w = zeros(1, Nfe);
dy = zeros(1, Nfe);
ddy = zeros(1, Nfe);
r = zeros(1, Nfe);
k = zeros(1, Nfe);
delta_t = 0.05;

for ii = 2 : Nfe
    v(ii) = sqrt(((x(ii) - x(ii-1)) / dt)^2 + ((y(ii) - y(ii-1)) / dt)^2);
end
for ii = 2 : Nfe
    a(ii) = (v(ii) - v(ii-1)) / dt;
end

global vehicle_kinematics_ vehicle_geometrics_
wb = vehicle_geometrics_.vehicle_wheelbase;
for ii = 2 : (Nfe-1)
    phy(ii) = atan((theta(ii+1) - theta(ii)) * wb / (dt * v(ii)));  % phy 前轮转角
end
ind = find(phy > vehicle_kinematics_.vehicle_phy_max); phy(ind) = vehicle_kinematics_.vehicle_phy_max;
ind = find(phy < -vehicle_kinematics_.vehicle_phy_max); phy(ind) = -vehicle_kinematics_.vehicle_phy_max;

for ii = 2 : (Nfe-1)
    w(ii) = (phy(ii+1) - phy(ii)) / dt;                           % w  前轮的转向角速度
end
ind = find(w > vehicle_kinematics_.vehicle_w_max); w(ind) = vehicle_kinematics_.vehicle_w_max;
ind = find(w < -vehicle_kinematics_.vehicle_w_max); w(ind) = -vehicle_kinematics_.vehicle_w_max;

[k, r, min_r, ind, r_temp, minNo ] = curvatute_r ( x, y, delta_t, lim_r );

end

function [k, r, min_r, ind, r_temp, minNo ] = curvatute_r ( x, y, t,lim_r )
   Nfe = length(x);
   for ii = 2 : Nfe
         dx(ii) = ( x(ii) - x(ii-1) )/t ;
   end
   dx(1) = dx(2)

   for ii = 2 : Nfe
         ddx(ii) = ( dx(ii) - dx(ii-1) )/t ;
   end
   ddx(1) = ddx(2)

   for ii = 2 : Nfe
         dy(ii) = ( y(ii) - y(ii-1) )/t ;
   end
   dy(1) = dy(2)

   for ii = 2 : Nfe
         ddy(ii) = ( dy(ii) - dy(ii-1) )/t ;
   end
   ddy(1) = ddy(2)

   for ii = 1 : Nfe
      if (abs( dx(ii)*ddy(ii) - ddx(ii)*dy(ii) ) ~= 0)
           r(ii) = (dx(ii)^2 + dy(ii)^2)^1.5 /abs( dx(ii)*ddy(ii) - ddx(ii)*dy(ii) );
      else 
           r(ii) = (dx(ii)^2 + dy(ii)^2)^1.5 /( abs( dx(ii)*ddy(ii) - ddx(ii)*dy(ii) ) + 0.000001 );
      end
       k(ii) = 1 / r(ii) ; 
   end
   min_r = min ( r );
   ind = find( r <= lim_r);
   r_temp = r( ind );
   minNo = find( r <= min_r);
end
