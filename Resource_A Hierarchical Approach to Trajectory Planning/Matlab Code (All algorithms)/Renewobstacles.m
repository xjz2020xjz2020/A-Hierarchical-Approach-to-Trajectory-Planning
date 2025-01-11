function [ obstacles_temp ] = Renewobstacles( nn )
global obstacles_ Nobs
Nfe = length( obstacles_{1,1}.s );
  for ii = 1 : Nobs
    ds = obstacles_{1,ii}.s(2) - obstacles_{1,ii}.s(1); 
    start_s = obstacles_{1,ii}.s(nn);
    end_s = start_s + (Nfe - 1)* ds;  %  end_s 往后顺延，长度依然为Nfe 
    offset = obstacles_{1,ii}.l(1);
    x = []; y = []; t = [];
    for jj = 1 : Nfe
          [xr, yr, ~, ~, theta] = ProvideReferenceLineInfo(start_s + ds * (jj-1));
          x = [x, xr - offset * cos(pi/2 + theta)];
          y = [y, yr - offset * sin(pi/2 + theta)];
          t = [t, theta];
    end
    elem.x = x; elem.y = y; elem.theta = t; elem.s = linspace(start_s, end_s, Nfe); elem.l = ones(1,Nfe) .* offset; 
    obstacles_temp{1,ii} = elem;
 end
end