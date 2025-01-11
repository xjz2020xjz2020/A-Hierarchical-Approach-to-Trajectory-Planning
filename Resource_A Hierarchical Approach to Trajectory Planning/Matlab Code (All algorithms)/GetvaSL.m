function [vs,vl,as,al] = GetvaSL(s, l)
Nfe = length(s);
vs = zeros(1, Nfe);
vl = zeros(1, Nfe);
as = zeros(1, Nfe);
al = zeros(1, Nfe);
dt = 0.05;
for ii = 2 : Nfe
    vs(ii) = (s(ii) - s(ii-1)) / dt;
    vl(ii) = (l(ii) - l(ii-1)) / dt;
end
vs(1) = vs(2);   vl(1) = vl(2); 
for ii = 2 : Nfe
    as(ii) = (vs(ii) - vs(ii-1)) / dt;
    al(ii) = (vl(ii) -vl(ii-1)) / dt;
end
as(1) = as(2);   al(1) = al(2); 
end