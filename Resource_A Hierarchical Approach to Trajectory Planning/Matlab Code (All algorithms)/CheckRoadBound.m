function [s,l] = CheckRoadBound( sback,lback, lmin, lmax )
%% 对漂移作截断处理，并采用3阶贝塞尔曲线对首位附近的l坐标进行光滑处理
Nfe = length(lback);
s = sback ;  %  s值不变，传入函数后，直接传出。s值没有作任何处理。
l = lback ;  %  只改变路点l值的大小

ind_min = find( lback < lmin) ;
if(~isempty(ind_min) )
  [mina,minb] = Getseg (ind_min);
  for i = 1: length(mina)   % 漂移片段的个数
    if(( mina(i)~= 1)&&(minb(i)~= Nfe))
        if( ( minb(i) - mina(i) ) == 0  )
              l(mina(i)) =  0.5* ( lback(mina(i)-1) +  lback(mina(i)+1) );
        elseif ( ( minb(i) - mina(i) ) == 1  )
              l(mina(i)) =  0.5* ( lback(mina(i)-1) +  lback(minb(i)+1) );
              l(minb(i)) =  l(mina(i)) ;
        elseif (   ( ( minb(i) - mina(i) )  >=  2  ) && ( ( minb(i) - mina(i) )  <=  8  )   )
              for j = ( mina(i)+1 ) :  ( minb(i) - 1 )
                     l(j) = lmin;
              end
              l(mina(i)) =  0.5* ( lback(mina(i)-1) +  lmin );
              l(minb(i)) =  0.5* ( lmin +  lback(minb(i)+1) );
        else
             aa = mina(i);  bb = minb(i);
             for ii = aa: bb
                l(ii) = lmin;
             end
             a1 = aa -6 ;  a2 = aa-3;  a3 = aa;     a4 = aa +3;
             b1 = bb -3 ;  b2 = bb;    b3 = bb +3;  b4 = bb +6;
             P0 = [];  P1 = [];  P2 = []; P3 = [];  P_t = [];
             if( a1 <1)
                 a1 = a2;
             end
             if( b4 > 201)
                 b4 = b3;
             end

             P0 = [l(a1), l(b1)];
             P1 = [l(a2), l(b2)];
             P2 = [l(a3), l(b3)];
             P3 = [l(a4), l(b4)];
             j=0;
             for t = 0:0.1111:1
                 P_t = (1-t)^3*P0 + 3*t*(1-t)^2*P1+ 3*t^2*(1-t)*P2+t^3*P3;
                 l(a1 +j )  = P_t(1);
                 l(b1 +j )  = P_t(2);
                 j = j + 1;
             end
        end
    else
              for j = mina(i) :  minb(i)
                     l(j) = lmin;
              end
    end
  end
end

ind_max = find( lback > lmax) ;
if(~isempty(ind_max) )
  [maxa,maxb] = Getseg (ind_max);
  for i = 1: length(maxa)
    if(( maxa(i)~= 1)&&(maxb(i)~= Nfe))
        if( ( maxb(i) - maxa(i) ) == 0  )
              l(maxa(i)) =  0.5* ( lback(maxa(i)-1) +  lback(maxa(i)+1) );
        elseif ( ( maxb(i) - maxa(i) ) == 1  )
              l(maxa(i)) =  0.5* ( lback(maxa(i)-1) +  lback(maxb(i)+1) );
              l(maxb(i)) =  l(maxa(i)) ;
        elseif (   ( ( maxb(i) - maxa(i) )  >=  2  ) && ( ( maxb(i) - maxa(i) )  <=  8  )   )
              for j = ( maxa(i)+1 ) :  ( maxb(i) - 1 )
                     l(j) = lmax ;
              end
              l(maxa(i)) =  0.5* ( lback(maxa(i)-1) +  lmax );
              l(maxb(i)) =  0.5* ( lmax +  lback(maxb(i)+1) );
        else
             aa = maxa(i);  bb = maxb(i); 
             for ii = aa: bb
                l(ii) = lmax ;
             end
             a1 = aa -6 ;  a2 = aa-3;  a3 = aa;     a4 = aa +3;
             b1 = bb -3 ;  b2 = bb;    b3 = bb +3;  b4 = bb +6;
             P0 = [];  P1 = [];  P2 = []; P3 = [];  P_t = [];
             P0 = [l(a1), l(b1)];
             P1 = [l(a2), l(b2)];
             P2 = [l(a3), l(b3)];
             P3 = [l(a4), l(b4)];
             j=0;
             for t = 0:0.1111:1
                 P_t = (1-t)^3*P0 + 3*t*(1-t)^2*P1+ 3*t^2*(1-t)*P2+t^3*P3;
                 l(a1 +j )  = P_t(1);
                 l(b1 +j )  = P_t(2); 
                 j = j + 1;
             end  
        end 
    else
              for j = mina(i) :  minb(i)
                     l(j) = lmax ;
              end
    end
  end
end
 ll = l(1 : 201);
 l=[];
 l=ll;
end

% function [l] = CheckRoadBound( lback, lmin, lmax )
% Nfe = length(lback);
% l = lback ;
% 
% ind_min = find( lback < lmin) ;
% if(~isempty(ind_min) )
%   [mina,minb] = Getseg (ind_min);
%   for i = 1: length(mina)
%     if(( mina(i)~= 1)&&(minb(i)~= Nfe))
%         if( ( minb(i) - mina(i) ) == 0  )
%               l(mina(i)) =  0.5* ( lback(mina(i)-1) +  lback(mina(i)+1) );
%         elseif ( ( minb(i) - mina(i) ) == 1  )
%               l(mina(i)) =  0.5* ( lback(mina(i)-1) +  lback(minb(i)+1) );
%               l(minb(i)) =  lback(mina(i)) ;
%         else
%               for j = ( mina(i)+1 ) :  ( minb(i) - 1 )
%                      l(j) = lmin ;
%               end
%               l(mina(i)) =  0.5* ( lback(mina(i)-1) +  lmin );
%               l(minb(i)) =  0.5* ( lmin +  lback(minb(i)+1) );
%         end
%     else
%               for j = mina(i) :  minb(i)
%                      l(j) = lmin ;
%               end
%     end
%   end
% end
% 
% ind_max = find( lback > lmax) ;
% if(~isempty(ind_max) )
%   [maxa,maxb] = Getseg (ind_max);
%   for i = 1: length(maxa)
%     if(( maxa(i)~= 1)&&(maxb(i)~= Nfe))
%         if( ( maxb(i) - maxa(i) ) == 0  )
%               l(maxa(i)) =  0.5* ( lback(maxa(i)-1) +  lback(maxa(i)+1) );
%         elseif ( ( maxb(i) - maxa(i) ) == 1  )
%               l(maxa(i)) =  0.5* ( lback(maxa(i)-1) +  lback(maxb(i)+1) );
%               l(maxb(i)) =  lback(maxa(i)) ;
%         else
%               for j = ( maxa(i)+1 ) :  ( maxb(i) - 1 )
%                      l(j) = lmax ;
%               end
%               l(maxa(i)) =  0.5* ( lback(maxa(i)-1) +  lmax );
%               l(maxb(i)) =  0.5* ( lmax +  lback(maxb(i)+1) );
%         end
%     else
%               for j = mina(i) :  minb(i)
%                      l(j) = lmax ;
%               end
%     end
%   end
% end
% 
% end

%%  function  Getseg()  usage example
%%  ind = [15 16  19  25]
%%  a =[15   19    25];   b =[16   19    25]
%% 
function [a , b] = Getseg( ind );
N = length(ind);
if (N == 1)
    a = ind(1);
    b = ind(1);
    return;
else
    a = ind(1);
    b = [];    
end

for ii = 2 : N
    if (ind(ii-1) ~= ( ind(ii)-1 ) ) 
          a =[a, ind(ii)];
          b = [b, ind(ii-1)];
    end
end
b = [b, ind(end)];
end






