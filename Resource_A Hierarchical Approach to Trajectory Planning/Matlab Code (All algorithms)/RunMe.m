% ==============================================================================
%  主程序
% ==============================================================================
clear all
close all
clc
% % 参数设置
global vehicle_geometrics_ % 车辆轮廓几何尺寸
vehicle_geometrics_.vehicle_wheelbase = 2.88;  %2.8
vehicle_geometrics_.vehicle_front_hang = 0.96;
vehicle_geometrics_.vehicle_rear_hang = 0.929;
vehicle_geometrics_.vehicle_width = 1.942;
vehicle_geometrics_.vehicle_residual = 0.1;
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + ...
    vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang;
global vehicle_kinematics_ % 车辆运动能力参数
vehicle_kinematics_.vehicle_v_ref = 20.0;
vehicle_kinematics_.vehicle_a_max = 0.5;
vehicle_kinematics_.vehicle_phy_max = 0.7;  % phy   前轮转角最大值
vehicle_kinematics_.vehicle_w_max = 0.5;    % w     前轮转角的角速度    w=d(phy)/dt 
global dp_ % 搜索空间参数
dp_.num_t_grids = 5;  %    t  时间    切片个数    （时间网格个数）
dp_.num_s_grids = 7;  %    s  指引线  切片个数为6  (单位时间网格下 指引线切片个数 ) 
dp_.num_l_grids = 8;  %    l  横向    切片个数为7
dp_.unit_time = 2.0;  %    2秒 / 单位时间网格
dp_.max_unit_s = dp_.unit_time * vehicle_kinematics_.vehicle_v_ref;   %    40m    单位网格时间的最大距离40m, 一共200m=40m*5
dp_.min_unit_s = 0;
dp_.ds = linspace(dp_.min_unit_s, dp_.max_unit_s, dp_.num_s_grids);
dp_.dl = linspace(0, 1, dp_.num_l_grids);

global obstacles_ Nobs precise_timeline precise_timeline_index % 障碍物随机生成
Nobs = 5;         %    子程序GenerateObstacles做全局变量引用
agv_vel = 22;     %    平均车速参数。  增加了速度，跑道长度增加   revised
precise_timeline = [0 : 0.05 : (dp_.unit_time * dp_.num_t_grids)]; %   共 10秒 = 2秒 *5
precise_timeline_index = cell(1,dp_.num_t_grids);
ind = round(linspace(1, length(precise_timeline), dp_.num_t_grids + 1));
for ii = 1 : dp_.num_t_grids
    elem.ind1 = ind(ii); elem.ind2 = ind(ii+1);
    precise_timeline_index{1,ii} = elem;
end
% obstacles_ = GenerateObstacles(agv_vel); % 随机得到5辆障碍车坐标（x，y，t,s,l）
% save  obstacles_
load  obstacles_
saveObstacles(obstacles_, 'obstacles_.txt');
obstaclesback_ = obstacles_;

global road_barriers_ % 道路边界散点
road_barriers_ = GenerateRoadBarrierGrids();  % 道路上边界点，各210个坐标点，s范围[-10, 200]

global BV_ % 边值
BV_.s0 = 0;
BV_.l0 = 0.78;
[BV_.x0, BV_.y0, BV_.theta0] = ConvertFrenetToCartesian(BV_.s0, BV_.l0);
BV_.v0 = 20;
% BV_.initAngle = 0.1;  %  added on 2022 05 06
BV_.phy0 = 0.18;

global nSeg_    % 区分轨迹生成类型     

% % DP搜索权重设置
dp_.w_collision = 1.0;
dp_.w_Ncollision = 10000;
dp_.w_Njerky = 10;         % 加加速度希望小，舒适性好。即S方向距离要尽量小。
dp_.w_lat_change = 1.0;
dp_.w_lon_change = 1.0;
dp_.w_lon_achieved = 10.0;
dp_.w_biasd = 0.5;
% % 决策轨迹并显示
lim_r = 7;
delta_n =18;
r_mins = 0 ;
sback = [BV_.s0]; lback = [BV_.l0];
lmin = -0.6;   lmax = 3.6 ;
mmm=0;

while ((~isempty(r_mins))|| ( length(sback) == 201 ) )
     mmm = mmm +1 ;
     tic; [x, y, theta, thetaFrenet, s, l] = SearchDecisionTrajectoryViaDp(); toc     % 轨迹路点的航向角 theta；  道路参考线线角度 thetaFrenet
    [vs,vl,as,al] = GetvaSL(s, l);
     % plot(x, y, '-r'); drawnow;    %  第一次轨迹规划线   后桥中心  为“红色实线”,不画框
     % hold on
     % qweFirst(x, y, theta);             %  第一次轨迹规划线   周车，自车轨迹痕迹，自车后桥中心轨迹，“红色实线”
     qweAnimEdge(x, y, theta);  

     % % 轨迹规划部分
     % cutting_rate = 0.8; % 只保留决策轨迹的前cutting_rate * 100 %时域，并仅在该时域上做轨迹规划
     cutting_rate = 1.0; % 只保留决策轨迹的前cutting_rate * 100 %时域，并仅在该时域上做轨迹规划
     tf = dp_.unit_time * dp_.num_t_grids * cutting_rate;
     temp = abs(precise_timeline - tf);
     ind_end = find(temp == min(temp)); ind_end = ind_end(1);
     Nfe = ind_end; x = x(1:Nfe); y = y(1:Nfe); theta = theta(1:Nfe);
     [v, a, phy, w, k, r, min_r, ind, r_mins] = FormInitialGuess(x, y, theta, tf, lim_r);  % 获得 各步长时间的 速度，加速度，前轮转角，前轮的转向角速度
     if ( ~isempty(r_mins) )
           [lf, vfe] = Cruisingmode(s,l, ind(1)-1, delta_n ); 
           
           for ii = 1 : delta_n
               l( ind(1) + ii -2 ) =  lf(ii) ;
           end     
           sback = [sback , s(2 : ind(1) + delta_n -2 )];
           lback = [lback , l(2 : ind(1) + delta_n -2 )];
                      
           if ( length(sback) > 201 )
                 sback = sback(1 : 201);
                 lback = lback(1 : 201);
           end 
           if ( mmm == 1 )
                 nSeg_ = length(sback);  %  nSeg_ 储存下一循环规划起点的序号（向前移动 delta_n节点就是DP规划，最近的delta_n节点就是ACC规划 ）
           else
                 nSeg_ = [nSeg_, length(sback)];  
           end
     else
           sback = [sback , s(2 : end )];
           lback = [lback , l(2 : end )];
           
           if ( length(sback) > 201 )
                sback = sback(1 : 201);
                lback = lback(1 : 201);
           end  
           break;
     end
    [x, y, theta, thetaFrenet] = ConvertSlToXYTheta(sback, lback);
    obstacles_ =  obstaclesback_ ;
%     qweAnim(x, y, theta); 
    
    BV_.s0 =  sback(end);
    BV_.l0 =  lback(end);
%     BV_.initAngle = atan2( (lback(end -1)-lback(end)), (sback(end -1)-sback(end)+0.00001));   %  added on 2022 05 06
    [BV_.x0, BV_.y0, BV_.theta0] = ConvertFrenetToCartesian(BV_.s0, BV_.l0);
    BV_.v0 = vfe;
    BV_.phy0 = 0;
    [ obstacles_temp ] = Renewobstacles( length (sback) );  
    obstacles_ = obstacles_temp ;
  if ( mmm >= 10 )
         break;
  end
end
    obstacles_ =  obstaclesback_ ;
    s=[]; l=[];
%     s = sback;
%     [l] = CheckRoadBound( lback, lmin, lmax );
    [s,l] = CheckRoadBound( sback,lback, lmin, lmax );
    % 对DP至ACC  以及 ACC至DP交界处的5点做样条插值处理。 

%  三阶贝塞尔拟合开始
    for  i =1: length(nSeg_)
       nn = nSeg_ (i);
       n1s = nn - delta_n -3 ;  n1m1 = nn - delta_n; n1m2 = nn - delta_n +3; n1e = nn - delta_n +6;
       P0 = [];  P1 = [];  P2 = []; P3 = [];  P_t = [];
       P0 = [l(n1s), s(n1s)];
       P1 = [0.1*(l(n1s) + l(n1m2)) + 0.8*(l(n1m1)), 0.2*(s(n1s) + s(n1m2)) + 0.6*(s(n1m1))];
       P2 = [0.1*(l(n1e) + l(n1m1)) + 0.8*(l(n1m2)), 0.2*(s(n1e) + s(n1m1)) + 0.6*(s(n1m2))];
       P3 = [l(n1e), s(n1e)];
       j=0;
       for t = 0:0.1111:1
            P_t = (1-t)^3*P0 + 3*t*(1-t)^2*P1+ 3*t^2*(1-t)*P2+t^3*P3;
            l(n1s +j )  = P_t(1);
            s(n1s +j )  = P_t(2); 
            j = j + 1;
       end
       j=0;
       for t = 0:0.1111:1
            ll(n1s +j )  = 0.5*l(n1s +j -1) + 0.5*l(n1s +j +1);
            ss(n1s +j )  = 0.5*s(n1s +j -1) + 0.5*s(n1s +j +1);
            j = j + 1;
       end
       j=0;
       for t = 0:0.1111:1
            l(n1s +j )  = ll(n1s +j );
            s(n1s +j )  = ss(n1s +j );
            j = j + 1;
       end
      
       n2s =  nn -5 ;  n2m1 = nn  -2; n2m2 = nn  +1;  n2e = nn  +4;
       P0 = [];  P1 = [];  P2 = [];   P3 = [];  P_t = [];
       P0 = [l(n2s), s(n2s)];
       P1 = [0.1*(l(n2s) + l(n2m2)) + 0.8*(l(n2m1)), 0.2*(s(n2s) + s(n2m2)) + 0.6*(s(n2m1))];
       P2 = [0.1*(l(n2e) + l(n2m1)) + 0.8*(l(n2m2)), 0.2*(s(n2e) + s(n2m1)) + 0.6*(s(n2m2))];
       P3 = [l(n2e), s(n2e)];
       j=0;
       for t = 0:0.1111:1
             P_t = (1-t)^3*P0 + 3*t*(1-t)^2*P1+ 3*t^2*(1-t)*P2+t^3*P3;
            l(n2s +j ) = P_t(1);
            s(n2s +j )  = P_t(2); 
            j = j + 1;
       end
       j=0;
       for t = 0:0.1111:1
            ll(n2s +j ) =  0.5*l(n2s +j -1)+  0.5*l(n2s +j +1);
            ss(n2s +j )  = 0.5*s(n2s +j -1)+  0.5*s(n2s +j +1);
            j = j + 1;
       end
       j=0;
       for t = 0:0.1111:1
            l(n2s +j ) =   ll(n2s +j );
            s(n2s +j )  =  ss(n2s +j );
            j = j + 1;
       end
    end
%  三阶贝塞尔拟合结束

    [vs,vl,as,al] = GetvaSL(s, l);
    [x, y, theta, thetaFrenet] = FinalConvertSlToXYTheta(s, l); 
    [v, a, phy, w, k, r, min_r, ind, r_mins, minNo] = FinalFormInitialGuess(x, y, theta, tf, lim_r); 
%    an= (v.^2).*k;
    qweAnim(x, y, theta);
    qweAnimtwoLines(x, y, theta, delta_n, minNo);
    traj3 = cell(1,1);
    %  theta  航向角；nSeg_ 交替规划算法中下一次DP算法的起始路点（其中 delta_n  为ACC路点的步长 ）
    elment.x = x; elment.y = y; elment.theta = theta; elment.nSeg_ =nSeg_;  elment.np = delta_n;  elment.minNo = minNo; 
    traj3{1,1} = elment;
    save traj3;
tic;
%  tic; [~, ~, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta, thetaFrenet); toc; % 画出前后桥的隧道，把前后桥坐标，后桥的角度，Freneti角度，以及rmin存CC
% [tun_v, tun_a, tun_phy, tun_w] = FormInitialGuessMid(xr, yr, tf);  % 获得 各步长时间的 速度，加速度，前轮转角，前轮的转向角速度
% % %%
% % plot(xr, yr, 'g');              %  第二次轨迹规划线   后桥中心  为“红色点线”（可能发生碰撞，沿着theta，朝矩形边界方向调整。造成曲线不连续）
% % hold on
% % plot(xf, yf, '-b');              %  第三次轨迹规划线   前桥中心  为“蓝色实线”（可能发生碰撞，沿着theta，朝矩形边界方向调整。造成曲线不连续）
% % hold on
% %%
% % WriteInitialGuessForFirstTimeNLP(x, y, theta, xr, yr, xf, yf, v, a, phy, w); 
% WriteInitialGuessForFirstTimeNLP(tun_v, tun_a, tun_phy, tun_w); 
% %原来决策得到的轨迹（x, y, theta），计算得到的（v, a, phy, w），据隧道调整后的xr, yr, xf, yf .存入文件 initial_guess0.INIVAL
% WriteParameters(tf, Nfe, x(end), y(end), theta(end)); % 写入文件BV_
% tic
% !ampl rrcir.run
% toc
% [cur_x, cur_y, cur_theta, cur_infeasibility] = LoadStates();
% %  plot(cur_x, cur_y, 'y'); drawnow;   %  2022 03 24
% qweAnim(cur_x, cur_y, cur_theta);      %  2022 03 24
% plot(x, y, 'b'); drawnow;   plot(xr, yr, 'r'); drawnow;   plot(cur_x, cur_y, 'k'); drawnow;   %  2022 03 24
% [cur_v, cur_a, cur_phy, cur_w] = FormInitialGuessMid(cur_x, cur_y, tf);
