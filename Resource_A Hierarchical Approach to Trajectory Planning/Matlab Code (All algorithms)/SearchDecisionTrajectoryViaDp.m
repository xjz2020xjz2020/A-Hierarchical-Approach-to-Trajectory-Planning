% ==============================================================================
function [x, y, theta, thetaFrenet,s, l] = SearchDecisionTrajectoryViaDp()
global state_space dp_ vehicle_geometrics_
NT = dp_.num_t_grids;
NS = dp_.num_s_grids;
NL = dp_.num_l_grids;
NOM = 0;
state_space = cell(NT, NS, NL);
node_default_status = [Inf, -1, -1, -Inf];    % cost val (1), parent ind (2-3), 本时间切片内纵向移动的格数s
% 状态空间初始化
for ii = 1 : NT
    for jj = 1 : NS
        for kk = 1 : NL
            state_space{ii, jj, kk} = node_default_status;
        end
    end
end

tic   %测试传统DP 算法的起始时间
for jj = 1 : NS
    for kk = 1 : NL
        [cost_val, s_achieved] = EstimateCostZero(jj,kk);               % cost_val 为 代价值, s_achieved 为 纵向移动的格数s
        state_space{1, jj, kk} = [cost_val, -999, -999, s_achieved];    % cost_val 最后寻找最小值
    end
end
for ii = 1 : (NT-1)
    for jj = 1 : NS
        for kk = 1 : NL
            cur_state_ind = [ii, jj, kk];
            for mm = 1 : NS
                for nn = 1 : NL
                    next_layer_state_ind = [ii+1, mm, nn];  % 计算每个 cur_state_ind与每个next_layer_state_ind之间的代价函数和可到达纵向移动的格数s.
                    [delta_cost, s_achieved] = EstimateCost(cur_state_ind, next_layer_state_ind);
                    cost = state_space{ii, jj, kk}(1) + delta_cost;
                    if (cost < state_space{ii+1, mm, nn}(1))
                        state_space{ii+1, mm, nn} = [cost, jj, kk, s_achieved];
                    end
                end
            end
        end
    end
end

toc     %测试传统DP算法的结束时间

% tic   %测试新的jump DP 算法的起始时间
% for jj = 1 : NS
%       kk = 1;
%       while (1)  
%           [cost_val, s_achieved, jOmmit] = EstimateCostZeroJP(jj,kk);        % cost_val 为 代价值, s_achieved 为 纵向移动的格数s
%            if (jOmmit == 0)
%                      state_space{1, jj, kk} = [cost_val, -999, -999, s_achieved];    % cost_val 最后寻找最小值
%            else
%                      for uu = kk: kk + jOmmit
%                               %  cost_val = cost_val;
%                               state_space{1, jj, uu} = [cost_val, -999, -999, s_achieved];    % cost_val 最后寻找最小值
%                      end
%                      kk = uu;
%            end
%            kk = kk + 1;
%            if ( kk  >  NL )
%                     break;
%            end
%       end
% end
% for ii = 1 : (NT-1)
%     for jj = 1 : NS
%         for kk = 1 : NL
%             cur_state_ind = [ii, jj, kk];
%             for mm = 1 : NS
%                 nn = 1;
%                 while (1)
%                     next_layer_state_ind = [ii+1, mm, nn];  % 计算每个 cur_state_ind与每个next_layer_state_ind之间的代价函数和可到达纵向移动的格数s.
%                     [delta_cost, s_achieved,jOmmit] = EstimateCostJP(cur_state_ind, next_layer_state_ind);
%                     cost = state_space{ii, jj, kk}(1) + delta_cost;
%                     if (jOmmit == 0)
%                          if (cost < state_space{ii+1, mm, nn}(1))
%                              state_space{ii+1, mm, nn} = [cost, jj, kk, s_achieved];
%                          end
%                     else
%                          for uu = nn: nn + jOmmit
%                               if (cost < state_space{ii+1, mm, uu}(1))
%                                    state_space{ii+1, mm, uu} = [cost, jj, kk, s_achieved];
%                               end
%                          end
%                          nn = nn + jOmmit;
%                     end 
%                     nn = nn + 1;
%                     if ( nn  >  NL )
%                             break;
%                     end
%                 end 
%             end
%         end
%     end
% end
% toc     %测试新的jump DP 算法的结束时间
 
% %%%   测试用
delete('statedata');
fid = fopen('statedata', 'w');
for ii = 1 : NT
    for jj = 1 : NS
        for kk = 1 : NL
            fprintf(fid, '%g %g %g %f   %g %g  %f \r\n', ii, jj, kk, state_space{ii, jj, kk}(1),state_space{ii, jj, kk}(2),state_space{ii, jj, kk}(3),state_space{ii, jj, kk}(4) );       
        end
    end
end
fclose(fid);
% %%%   测试用
% 寻找最优的末端结点
cur_best = Inf;
for jj = 1 : NS
    for kk = 1 : NL
        if (state_space{NT, jj, kk}(1) < cur_best)
            cur_best = state_space{NT, jj, kk}(1);
            cur_best_s_ind = jj;
            cur_best_l_ind = kk;
        end
    end
end
ind_s = cur_best_s_ind; ind_l = cur_best_l_ind;
child_s = cur_best_s_ind; child_l = cur_best_l_ind;
for ii = (NT-1) : -1 : 1
    cur_ind = state_space{ii+1, child_s, child_l}(2:3);
    ind_s = [cur_ind(1), ind_s];
    ind_l = [cur_ind(2), ind_l];
    child_s = cur_ind(1); child_l = cur_ind(2);
end                                               % 获得最优序列：ind_s=,如[5 4 2 4 6]和 ind_l,如[5 6 3 2 3]
% 结点回溯
global BV_ vehicle_geometrics_
s = BV_.s0; l = BV_.l0;
for ii = 1 : NT
    s = [s, dp_.ds(ind_s(ii)) + s(end)];
    [lb, rb] = ProvideRoadBound(s(end));
    lb = lb + ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
    rb = rb - ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
    l = [l, lb + (rb - lb) * dp_.dl(ind_l(ii))];
end
% 优化得到  s:         0           26.6667     46.6667     53.3333      73.3333      106.6667，  （6个  =NT+1
%           l:         0.7800      1.9226      2.4761      0.2865       -0.3201      0.3696      （6个  =NT+1 ）
% % 1）原有。   对六个点均匀插值，距离均等。v 和 a 均不连续。6个不同的v。

    [s, l] = ResampleSL(s, l);   %  s,l 分别为6点插值为201点，共 10s= 200 * 0.05s， 重采样并转化到笛卡尔坐标系  ，共10s= 5*2s
 
% 2）针对（s，l)坐标分别规划出5段5次多项式曲线。每段5次多项式曲线认为a=0,那么这一段几乎是平的，首尾曲率为0，单调上升或下降
%   S只会单调上升，但L的单调性不定。汽车会一直前向并左右蛇行。

%     [s, l] = ResampleSL_FiveOrderPoly(s, l);

% 3）针对（s，l)坐标分别规划出5条线段组成的贝塞尔曲线。

%    [s, l] = ResampleSL_Bezier(s, l); 

% 4）针对（s，l)坐标分别规划出5条线段组成的B样条曲线。
% %  tic
  stem = [s(1),s(21),s(41),s(61),s(81),s(101),s(121),s(141),s(161),s(181),s(201) ];
  ltem = [l(1),l(21),l(41),l(61),l(81),l(101),l(121),l(141),l(161),l(181),l(201) ];
  s=[]; l=[];
  [s, l] = ResampleSL_BSpline(stem, ltem); 
%   
% %    [s, l] = ResampleSL_BSpline(s, l); 
%  toc
  [x, y, theta, thetaFrenet] = ConvertSlToXYTheta(s, l);   %Frent坐标系下201个(s，l)转化为201个笛卡尔坐标的[x, y, theta] 
end

% function [s_full,l_full] = ResampleSL(s,l)
% s_full = []; l_full = [];
% global precise_timeline_index dp_
% for ii = 1 : dp_.num_t_grids   
%     ind1 = precise_timeline_index{1,ii}.ind1;
%     ind2 = precise_timeline_index{1,ii}.ind2;
%     len = ind2 - ind1;
%     if (ii == dp_.num_t_grids)
%         len = len + 1;
%     end
%     s_full = [s_full, linspace(s(ii), s(ii+1), len)];
%     l_full = [l_full, linspace(l(ii), l(ii+1), len)];
% end
% end

function [s_full,l_full] = ResampleSL(s,l)
s_full = s(1); l_full =  l(1);
global precise_timeline_index dp_
for ii = 1 : dp_.num_t_grids   
    ind1 = precise_timeline_index{1,ii}.ind1;
    ind2 = precise_timeline_index{1,ii}.ind2;
    len = ind2 - ind1;
    s_temp = linspace(s(ii), s(ii+1), (len+1));
    l_temp = linspace(l(ii), l(ii+1), (len+1));
    
    s_full = [s_full, s_temp(2: end)];
    l_full = [l_full, l_temp(2: end)];
end
end

function [s_full,l_full] = ResampleSL_FiveOrderPoly(s,l)
%%   programmed by Fuzhou Zhao at TSinghua University  2021/12/17 
%%   Get the S and L coordinate values  between the 6 optimized values in Frenet coordinate system
A=[];
B=[]; 
s_full = []; l_full = [];
delta_t =2;
Nfe = length(s);
    for i = 1 : Nfe -1
       [a, b] = GetPolyCoefficient_SL(i, s, l, delta_t);
       A=[A, a];
       B=[B, b];      
    end

   t=(0: 0.05: 5*delta_t )';
   for i = 1:length(t)
      j= floor( t(i)/2 )
      if (j < Nfe -1 )
         j=j+1;
       end
        % 纵向位置坐标
        s_full(i) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * A(:,j);
        % 横向位置坐标
        l_full(i) = [t(i)^5, t(i)^4, t(i)^3, t(i)^2, t(i), 1] * B(:,j);
   end 
end

function [s_full,l_full] = ResampleSL_Bezier(s,l)
%%   programmed by Fuzhou Zhao at TSinghua University  2021/12/19 
%%   Get the S and L coordinate values  between the 6 optimized values in Frenet coordinate system
SA=[];  LA=[];
delta_t=2;
Nfe = length(s);
for i = 1 : Nfe
       a = [s(i), delta_t*(i-1)];
       SA=[SA; a];
       b = [l(i), delta_t*(i-1)];
       LA=[LA; b];      
end

n = Nfe-1;
s_temp = [];
l_temp = [];
s_full = [];
l_full = [];

for t = 0:0.005:1
    if n == 1
        SA_t = SA;
        LA_t = LA;
        s_temp(end+1,:) = SA_t;
        l_temp(end+1,:) = LA_t;
    elseif n >= 2
        SA_t = [0, 0];
        LA_t = [0, 0];
        for i = 0:n
            k_C = factorial(n) / (factorial(i) * factorial(n-i));
            k_t = (1-t)^(n-i) * t^i;
            SA_t = SA_t + k_C * k_t * SA(i+1,:);
            LA_t = LA_t + k_C * k_t * LA(i+1,:);
        end
        s_temp(end+1,:) = SA_t(1);
        l_temp(end+1,:) = LA_t(1);    
    else
        disp('控制点输入有误，请重新输入')
    end
end
        s_full=s_temp';
        l_full=l_temp';
end

function [s_full,l_full] = ResampleSL_BSpline(s,l)
%%   programmed by Fuzhou Zhao at TSinghua University  2021/12/21 
%%   Get the S and L coordinate values  between the 6 optimized values in Frenet coordinate system
% global dp_ % 搜索空间参数
% dp_.num_t_grids = 5;  %    t  时间    切片个数    （时间网格个数）
% dp_.unit_time = 2.0;  %    2秒 / 单位时间网格
% 
% SA=[];  LA=[]; AA=[];  BB=[];
% delta_t=2;
% Nfe = length(s);
% for i = 1 : Nfe
%        a = [s(i), delta_t*(i-1)];
%        AA=[AA; a];
%        b = [l(i), delta_t*(i-1)];
%        BB=[BB; b];      
% end
%  SA=AA';
%  LA=BB';
% 
% n = Nfe-1;
% s_temp = [];
% l_temp = [];
% s_full = [];
% l_full = [];
% k = 4;                                    % k阶、k-1次B样条
% Bik = zeros(n+1, 1);
% NodeVector = U_quasi_uniform(n, k-1); % 准均匀B样条的节点矢量
%     for u = 0:0.005:1- 0.005   
%         for i = 0 : 1 : n
%             Bik(i+1, 1) = BaseFunction(i, k-1 , u, NodeVector);
%         end
%         p_SA = SA * Bik;
%         s_temp=[s_temp,p_SA];
%         p_LA = LA * Bik;
%         l_temp=[l_temp, p_LA];  
%     end
%         s_temp=[s_temp,[s(end), dp_.num_t_grids * dp_.unit_time]'];
%         l_temp=[l_temp,[l(end), dp_.num_t_grids * dp_.unit_time]'];
%     
%         s_full=s_temp(1,:);
%         l_full=l_temp(1,:);

         t1=[0,1,2,3,4,5,6,7,8,9,10];
         t2=[0:0.05:10];
         
%          t1=[0,2,4,6,8,10];
%          t2=[0:0.05:10];
         
         s_full = spline(t1, s, t2);
         l_full = spline(t1, l, t2);
end

function [a, b] =  GetPolyCoefficient_SL(i, S, L, delta_t)
%%   programmed by Fuzhou Zhao at TSinghua University  2021/12/17 
%%   Get the Coefficients of  S and L  of Fifth-order polynomial in Frenet coordinate system
n= length(S);
%   纵向S的节点速度设定，纵向S的坐标值是单调增函数
if( i == 1 )
     v1S= (S(i+1)-S(i))/delta_t; 
     v2S= (S(i+2)-S(i))/(delta_t*2);   
elseif( i == n-1 )
     v1S= (S(i+1)-S(i-1))/(delta_t*2); 
     v2S= (S(i+1)-S(i))/delta_t;       
else
     v1S= (S(i+1)-S(i-1))/(delta_t*2);  
     v2S= (S(i+2)-S(i))/(delta_t*2);      
end

%   横向L的节点速度设定
if( i == n-1 )
     v1L= (L(i+1)-L(i))/delta_t;
     v2L= v1L;        % 当前线段的为最后一段时，后节点与前节点速度相同
else
     v1L= (L(i+1)-L(i))/delta_t;
     v2L= (L(i+2)-L(i+1))/delta_t;
end

if( 2 <= i )
    vf=L(i+1)-L(i)
    vr=L(i)-L(i-1)
    if( vf*vr <0 )    % 当前线段的前节点为拐点，速度设为0
         v1L= 0;
    end
end

if( i <= n-2 )    
    vf=L(i+2)-L(i+1)
    vr=L(i+1)-L(i)
    if( vf*vr <0 )   % 当前线段的后节点为拐点，速度设为0
         v2L= 0;
    end
end

 a1S=0;  a1L=0;
 a2S=0;  a2L=0;

t1 = (i - 1 )* delta_t;
t2 =  i * delta_t;
SS = [S(i); v1S; a1S; S(i+1); v2S; a2S];
LL = [L(i); v1L; a1L; L(i+1); v2L; a2L];
T = [ t1^5      t1^4      t1^3     t1^2    t1   1;
      5*t1^4    4*t1^3    3*t1^2   2*t1    1    0;
      20*t1^3   12*t1^2   6*t1     2       0    0;
      t2^5      t2^4      t2^3     t2^2    t2   1;
      5*t2^4    4*t2^3    3*t2^2   2*t2    1    0;
      20*t2^3   12*t2^2   6*t2     2       0    0];
a = T \ SS;
b = T \ LL;
end

function [val, s1] = EstimateCostZero(jj,kk)
global dp_ BV_ precise_timeline_index vehicle_geometrics_
s0 = BV_.s0;
l0 = BV_.l0;
s1 = s0 + dp_.ds(jj);
[lb, rb] = ProvideRoadBound(s1);
lb = lb + ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
rb = rb - ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
l1 = lb + (rb - lb) * dp_.dl(kk);

Nfe = precise_timeline_index{1,1}.ind2 - precise_timeline_index{1,1}.ind1 + 1;
s_list = linspace(s0, s1, Nfe);
l_list = linspace(l0, l1, Nfe);
val = dp_.w_collision * EvaluateCollision(1, s_list, l_list) + ...
    dp_.w_lat_change * EvaluateLateralChange(s_list, l_list) + ...
    dp_.w_lon_achieved * EvaluateAchievement(s_list) + ...
    dp_.w_biasd * mean(abs(l_list));
end

function [val, s1] = EstimateCost(cur_state_ind, next_layer_state_ind)
global state_space dp_ precise_timeline_index vehicle_geometrics_
s0 = state_space{cur_state_ind(1), cur_state_ind(2), cur_state_ind(3)}(4);
[lb, rb] = ProvideRoadBound(s0);
lb = lb + ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
rb = rb - ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
l0 = lb + (rb - lb) * dp_.dl(cur_state_ind(3));

s1 = s0 + dp_.ds(next_layer_state_ind(2));
[lb, rb] = ProvideRoadBound(s1);
lb = lb + ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
rb = rb - ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
l1 = lb + (rb - lb) * dp_.dl(next_layer_state_ind(3));

Nfe = precise_timeline_index{1,next_layer_state_ind(1)}.ind2 - precise_timeline_index{1,next_layer_state_ind(1)}.ind1 + 1;
s_list = linspace(s0, s1, Nfe);
l_list = linspace(l0, l1, Nfe);
val = dp_.w_collision * EvaluateCollision(next_layer_state_ind(1), s_list, l_list) + ...
    dp_.w_lat_change * EvaluateLateralChange(s_list, l_list) + ...
    dp_.w_lon_achieved * EvaluateAchievement(s_list) + ...
    dp_.w_lon_change * EvaluateLongituteChange(cur_state_ind(2), next_layer_state_ind(2)) + ...
    dp_.w_biasd * mean(abs(l_list));
end
function [val, s1, j ] = EstimateCostZeroJP(jj, kk)
global dp_ BV_ precise_timeline_index vehicle_geometrics_
j = 0;
NL = dp_.num_l_grids;
s0 = BV_.s0;
l0 = BV_.l0;                                          %（s0，l0）起点位置
s1 = s0 + dp_.ds(jj);                                 % 获得从s0开始到 偏移为ds(jj)的纵向终点位置s1
[lb, rb] = ProvideRoadBound(s1);                      % 以s1获得道路上下边界的宽度坐标
lb = lb + ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
rb = rb - ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;    % 道路上下边界需预留车身半宽
l1 = lb + (rb - lb) * dp_.dl(kk);                     % 获得从lb开始到 偏移为dl(kk)比例的横向终点位置l1
% headingAngle = atan2( (l1-l0), (s1-s0+0.00001));    %  added on 2022 05 06
Nfe = precise_timeline_index{1,1}.ind2 - precise_timeline_index{1,1}.ind1 + 1;
s_list = linspace(s0, s1, Nfe);                       % 纵向（s0，s1）分为40等份
l_list = linspace(l0, l1, Nfe);                       % 横向（l0，l1）分为40等份

[nCost, nCol, LLV] = EvaluateCollisionJP(1, s_list, l_list);
if (~isempty(nCol)) &&( NL > kk )
        val = nCost;
        nL0 =  nCol(1);
        nL1 =  nCol(end);
        for i = kk+1 : NL
            lltemp = lb + (rb - lb) * dp_.dl(i);
            l_listtemp = linspace(l0, lltemp, Nfe);
            minVal = min(l_listtemp(nL0) ,l_listtemp(nL1)) - (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);
            maxVal = max(l_listtemp(nL0) ,l_listtemp(nL1)) + (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);
            if ((LLV(2) < minVal)||(LLV(1) >  maxVal))
                 return
            else
                 j = j +1;
            end
        end
        if (j == ( NL - kk ) )
                return
        end
end
%      val = 20*abs(( BV_.initAngle - headingAngle))  + ...        %  added on 2022 05 06        
    val = dp_.w_collision * nCost + ...                                      % 希望绝对无碰撞，权重非常大
          dp_.w_lat_change * EvaluateLateralChange(s_list, l_list) + ...      % 希望不要侧向滑动
          dp_.w_lon_achieved * EvaluateAchievement(s_list) + ...              % 希望纵向移动速度尽量快
          dp_.w_biasd * mean(abs(l_list));                                    % 希望横向波动速度尽量小
end

function [val, s1, j ] = EstimateCostJP(cur_state_ind, next_layer_state_ind)
global state_space dp_ precise_timeline_index vehicle_geometrics_
j = 0;
NL = dp_.num_l_grids;
s0 = state_space{cur_state_ind(1), cur_state_ind(2), cur_state_ind(3)}(4);   % 第4个元素保存到达的s值
[lb, rb] = ProvideRoadBound(s0);
lb = lb + ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
rb = rb - ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
l0 = lb + (rb - lb) * dp_.dl(cur_state_ind(3));

s1 = s0 + dp_.ds(next_layer_state_ind(2));
[lb, rb] = ProvideRoadBound(s1);
lb = lb + ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
rb = rb - ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ) * 0.5;
kk = next_layer_state_ind(3);
l1 = lb + (rb - lb) * dp_.dl(kk);

Nfe = precise_timeline_index{1,next_layer_state_ind(1)}.ind2 - precise_timeline_index{1,next_layer_state_ind(1)}.ind1 + 1;
s_list = linspace(s0, s1, Nfe);
l_list = linspace(l0, l1, Nfe);

[nCost, nCol, LLV] = EvaluateCollisionJP(next_layer_state_ind(1), s_list, l_list);
if (~isempty(nCol)) &&( NL > kk )
        val = nCost;
        nL0 =  nCol(1);
        nL1 =  nCol(end);
        for i = kk+1 : NL
            lltemp = lb + (rb - lb) * dp_.dl(i);
            l_listtemp = linspace(l0, lltemp, Nfe);
            minVal = min(l_listtemp(nL0) ,l_listtemp(nL1)) - (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);
            maxVal = max(l_listtemp(nL0) ,l_listtemp(nL1)) + (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual);
            if ((LLV(2) < minVal)||(LLV(1) >  maxVal))    % 到达 Nc(j)节点，返回
                 return
            else
                 j = j +1;
            end
        end
        if (j == ( NL - kk ) )
                 return
        end
end
    val = dp_.w_collision * nCost + ...                                                                     % 希望绝对无碰撞，权重非常大
          dp_.w_lat_change * EvaluateLateralChange(s_list, l_list) + ...                                    % 希望不要侧向滑动
          dp_.w_lon_achieved * EvaluateAchievement(s_list) + ...                                            % 希望纵向移动速度尽量快
          dp_.w_lon_change * EvaluateLongituteChange(cur_state_ind(2), next_layer_state_ind(2)) + ...       % 加加速度希望小，舒适性好。即S方向距离要尽量小。
          dp_.w_biasd * mean(abs(l_list));                                                                  % 希望横向波动速度尽量小
end

function val = EvaluateLongituteChange(ind1, ind2)
global dp_
if (abs(ind1 - ind2) >= 3)
    val = dp_.w_Njerky;        % 加加速度希望小，舒适性好。即S方向距离要尽量小。
else
    val = 0;
end
end

function cost = EvaluateLateralChange(s_list, l_list)
% 我们不允许在ds = 0情况下L有变化，这相当于车辆在纯侧滑，显然严重违背车辆运动学
ds = s_list(end) - s_list(1) + 0.00000001;
dl = abs(l_list(end) - l_list(1));
cost = dl / ds;
end

function val = EvaluateAchievement(s_list)
global dp_
val = 1 - (s_list(end) - s_list(1)) / dp_.max_unit_s;  % 希望纵向移动速度尽量快
end

function cost = EvaluateCollision(ind_time, s_list, l_list)
cost = 0;
global obstacles_ precise_timeline_index Nobs
ind1 = precise_timeline_index{1,ind_time}.ind1;
ind2 = precise_timeline_index{1,ind_time}.ind2;
for ii = 1 : Nobs
    s = obstacles_{1,ii}.s(ind1 : ind2);
    l = obstacles_{1,ii}.l(ind1 : ind2);
    cost = cost + MeasureCollision(s,l,s_list,l_list);
end
end

function [cost, nCol, LLV] = EvaluateCollisionJP(ind_time, s_list, l_list)
cost = 0;
nCol=[];
LLV=[];
global obstacles_ precise_timeline_index Nobs
ind1 = precise_timeline_index{1,ind_time}.ind1;
ind2 = precise_timeline_index{1,ind_time}.ind2;
for ii = 1 : Nobs
    s = obstacles_{1,ii}.s(ind1 : ind2);
    l = obstacles_{1,ii}.l(ind1 : ind2);
    
    [local_cost, local_nCol, local_LLV] =MeasureCollisionJP(s,l,s_list,l_list);   
    if (~isempty(local_nCol))   % 5辆周车中，只要有1辆碰撞便返回。没必要全部检测
        cost = local_cost;
        nCol = local_nCol;
        LLV = local_LLV;
        return
    
    end
end
end

function val = MeasureCollision(s, l, s_ego, l_ego)
global vehicle_geometrics_ dp_
Nfe = length(s);
err_s = abs(s - s_ego);
ind = find(err_s <= ( vehicle_geometrics_.vehicle_length + vehicle_geometrics_.vehicle_residual ));
s = s(ind); l = l(ind); s_ego = s_ego(ind); l_ego = l_ego(ind);
err_l = abs(l - l_ego);
ind = find(err_l <= ( vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual ));
if (length(ind) > 0)
    val = dp_.w_Ncollision + length(ind) / Nfe;
else
    val = 0;
end
end

function [val, nCol, LLV] = MeasureCollisionJP(s, l, s_ego, l_ego)
global vehicle_geometrics_ dp_
val = 0;
nCol=[];
LLV=[];
Nfe = length(s);
err_s = abs(s - s_ego);
ind1 = find(err_s <= (vehicle_geometrics_.vehicle_length + vehicle_geometrics_.vehicle_residual));         % 可以在此增加 避撞长度 余量，  2021-12-9
s = s(ind1); l = l(ind1); s_ego = s_ego(ind1); l_ego = l_ego(ind1);
err_l = abs(l - l_ego);
ind2 = find(err_l <= (vehicle_geometrics_.vehicle_width + vehicle_geometrics_.vehicle_residual));          % 可以在此增加 避撞宽度 余量，  2021-12-9
    if (~isempty(ind2))
        nCol = ind1(ind2);
        LLV = [min(l(ind2)),max(l(ind2))];   % 通常情况下，l中元素等值。如不等值，则障碍车辆发生横向移动。
        val = dp_.w_Ncollision + length(ind2) / Nfe;
    end
end