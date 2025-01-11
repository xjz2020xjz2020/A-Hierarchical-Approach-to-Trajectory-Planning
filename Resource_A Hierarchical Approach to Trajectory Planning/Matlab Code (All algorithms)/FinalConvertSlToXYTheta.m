function [x, y, theta, thetaFrenet] = FinalConvertSlToXYTheta(s, l)
  global nSeg_  
  Nfe = length(s);
  for ii= 1:length( nSeg_ )
              nn = nSeg_( ii );
              if( nn >= 2 )
                  temps1 = (0.3*s(nn-2) +0.4*s(nn-1) +0.3*s(nn));
                  temps2 =(0.3*s(nn-1) +0.4*s(nn) + 0.3*s(nn+1));
                  temps3 =(0.3*s(nn) + 0.4*s(nn+1)+ 0.3*s(nn+2));
                  s(nn-1) =  temps1;
                  s(nn) =  temps2;
                  s(nn+1) =  temps3;  
              
                  templ1 = (0.3*l(nn-2) +0.4*l(nn-1) +0.3*l(nn));
                  templ2 =(0.3*l(nn-1) +0.4*l(nn) + 0.3*l(nn+1));
                  templ3 =(0.3*l(nn) + 0.4*l(nn+1)+ 0.3*l(nn+2));
                  l(nn-1) =  templ1;
                  l(nn) =  templ2;
                  l(nn+1) =  templ3;
              end
    end
 
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