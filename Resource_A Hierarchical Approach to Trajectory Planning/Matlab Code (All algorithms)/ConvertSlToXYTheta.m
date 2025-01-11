function [x, y, theta, thetaFrenet] = ConvertSlToXYTheta(s, l)
    Nfe = length(s);
    x = zeros(1,Nfe); y = zeros(1,Nfe); theta = zeros(1,Nfe);
    for ii = 1 : Nfe
         [x(ii), y(ii), thetaFrenet(ii)] = ConvertFrenetToCartesian(s(ii), l(ii));
    end
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
%  theta = 0.8 * theta + 0.2 * thetaFrenet; 
end