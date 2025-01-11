function [v, a, phy, w] = FormInitialGuess(x, y, tf)  % 获得 各步长时间的 速度，加速度，前轮转角，前轮的转向角速度
Nfe = length(x);
v = zeros(1, Nfe);
a = zeros(1, Nfe);
dt = tf / (Nfe - 1);
for ii = 2 : Nfe
    v(ii) = sqrt(((x(ii) - x(ii-1)) / dt)^2 + ((y(ii) - y(ii-1)) / dt)^2);
end
for ii = 2 : Nfe
    a(ii) = (v(ii) - v(ii-1)) / dt;
end

theta = zeros(1,Nfe);
diff_x = diff(x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(y) ;
diff_y(end+1) = diff_y(end);  
theta = atan2(diff_y , diff_x);      % 航向角

ind = find(theta <=  - 2.5);     %  保证theta 在（-0.8PI， 1.2PI ）范围内
 if(~isempty(ind)  )
      theta0 = theta(1: (ind(1)-1 ));
      for jj = ind(1): Nfe
           thetaTemp =[];
           if (theta(jj) <=  - 2.5)
               thetaTemp =  theta(jj)+ 2 * pi;
           else
               thetaTemp =  theta(jj);   
           end
      theta0 = [theta0 thetaTemp];
      end
      theta = theta0;
   end

phy = zeros(1, Nfe);
w = zeros(1, Nfe);
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
end