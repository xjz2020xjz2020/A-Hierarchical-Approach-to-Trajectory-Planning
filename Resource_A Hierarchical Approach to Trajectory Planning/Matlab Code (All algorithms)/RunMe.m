% ==============================================================================
%  ������
% ==============================================================================
clear all
close all
clc
% % ��������
global vehicle_geometrics_ % �����������γߴ�
vehicle_geometrics_.vehicle_wheelbase = 2.88;  %2.8
vehicle_geometrics_.vehicle_front_hang = 0.96;
vehicle_geometrics_.vehicle_rear_hang = 0.929;
vehicle_geometrics_.vehicle_width = 1.942;
vehicle_geometrics_.vehicle_residual = 0.1;
vehicle_geometrics_.vehicle_length = vehicle_geometrics_.vehicle_wheelbase + ...
    vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_rear_hang;
global vehicle_kinematics_ % �����˶���������
vehicle_kinematics_.vehicle_v_ref = 20.0;
vehicle_kinematics_.vehicle_a_max = 0.5;
vehicle_kinematics_.vehicle_phy_max = 0.7;  % phy   ǰ��ת�����ֵ
vehicle_kinematics_.vehicle_w_max = 0.5;    % w     ǰ��ת�ǵĽ��ٶ�    w=d(phy)/dt 
global dp_ % �����ռ����
dp_.num_t_grids = 5;  %    t  ʱ��    ��Ƭ����    ��ʱ�����������
dp_.num_s_grids = 7;  %    s  ָ����  ��Ƭ����Ϊ6  (��λʱ�������� ָ������Ƭ���� ) 
dp_.num_l_grids = 8;  %    l  ����    ��Ƭ����Ϊ7
dp_.unit_time = 2.0;  %    2�� / ��λʱ������
dp_.max_unit_s = dp_.unit_time * vehicle_kinematics_.vehicle_v_ref;   %    40m    ��λ����ʱ���������40m, һ��200m=40m*5
dp_.min_unit_s = 0;
dp_.ds = linspace(dp_.min_unit_s, dp_.max_unit_s, dp_.num_s_grids);
dp_.dl = linspace(0, 1, dp_.num_l_grids);

global obstacles_ Nobs precise_timeline precise_timeline_index % �ϰ����������
Nobs = 5;         %    �ӳ���GenerateObstacles��ȫ�ֱ�������
agv_vel = 22;     %    ƽ�����ٲ�����  �������ٶȣ��ܵ���������   revised
precise_timeline = [0 : 0.05 : (dp_.unit_time * dp_.num_t_grids)]; %   �� 10�� = 2�� *5
precise_timeline_index = cell(1,dp_.num_t_grids);
ind = round(linspace(1, length(precise_timeline), dp_.num_t_grids + 1));
for ii = 1 : dp_.num_t_grids
    elem.ind1 = ind(ii); elem.ind2 = ind(ii+1);
    precise_timeline_index{1,ii} = elem;
end
% obstacles_ = GenerateObstacles(agv_vel); % ����õ�5���ϰ������꣨x��y��t,s,l��
% save  obstacles_
load  obstacles_
saveObstacles(obstacles_, 'obstacles_.txt');
obstaclesback_ = obstacles_;

global road_barriers_ % ��·�߽�ɢ��
road_barriers_ = GenerateRoadBarrierGrids();  % ��·�ϱ߽�㣬��210������㣬s��Χ[-10, 200]

global BV_ % ��ֵ
BV_.s0 = 0;
BV_.l0 = 0.78;
[BV_.x0, BV_.y0, BV_.theta0] = ConvertFrenetToCartesian(BV_.s0, BV_.l0);
BV_.v0 = 20;
% BV_.initAngle = 0.1;  %  added on 2022 05 06
BV_.phy0 = 0.18;

global nSeg_    % ���ֹ켣��������     

% % DP����Ȩ������
dp_.w_collision = 1.0;
dp_.w_Ncollision = 10000;
dp_.w_Njerky = 10;         % �Ӽ��ٶ�ϣ��С�������Ժá���S�������Ҫ����С��
dp_.w_lat_change = 1.0;
dp_.w_lon_change = 1.0;
dp_.w_lon_achieved = 10.0;
dp_.w_biasd = 0.5;
% % ���߹켣����ʾ
lim_r = 7;
delta_n =18;
r_mins = 0 ;
sback = [BV_.s0]; lback = [BV_.l0];
lmin = -0.6;   lmax = 3.6 ;
mmm=0;

while ((~isempty(r_mins))|| ( length(sback) == 201 ) )
     mmm = mmm +1 ;
     tic; [x, y, theta, thetaFrenet, s, l] = SearchDecisionTrajectoryViaDp(); toc     % �켣·��ĺ���� theta��  ��·�ο����߽Ƕ� thetaFrenet
    [vs,vl,as,al] = GetvaSL(s, l);
     % plot(x, y, '-r'); drawnow;    %  ��һ�ι켣�滮��   ��������  Ϊ����ɫʵ�ߡ�,������
     % hold on
     % qweFirst(x, y, theta);             %  ��һ�ι켣�滮��   �ܳ����Գ��켣�ۼ����Գ��������Ĺ켣������ɫʵ�ߡ�
     qweAnimEdge(x, y, theta);  

     % % �켣�滮����
     % cutting_rate = 0.8; % ֻ�������߹켣��ǰcutting_rate * 100 %ʱ�򣬲����ڸ�ʱ�������켣�滮
     cutting_rate = 1.0; % ֻ�������߹켣��ǰcutting_rate * 100 %ʱ�򣬲����ڸ�ʱ�������켣�滮
     tf = dp_.unit_time * dp_.num_t_grids * cutting_rate;
     temp = abs(precise_timeline - tf);
     ind_end = find(temp == min(temp)); ind_end = ind_end(1);
     Nfe = ind_end; x = x(1:Nfe); y = y(1:Nfe); theta = theta(1:Nfe);
     [v, a, phy, w, k, r, min_r, ind, r_mins] = FormInitialGuess(x, y, theta, tf, lim_r);  % ��� ������ʱ��� �ٶȣ����ٶȣ�ǰ��ת�ǣ�ǰ�ֵ�ת����ٶ�
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
                 nSeg_ = length(sback);  %  nSeg_ ������һѭ���滮������ţ���ǰ�ƶ� delta_n�ڵ����DP�滮�������delta_n�ڵ����ACC�滮 ��
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
    % ��DP��ACC  �Լ� ACC��DP���紦��5����������ֵ���� 

%  ���ױ�������Ͽ�ʼ
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
%  ���ױ�������Ͻ���

    [vs,vl,as,al] = GetvaSL(s, l);
    [x, y, theta, thetaFrenet] = FinalConvertSlToXYTheta(s, l); 
    [v, a, phy, w, k, r, min_r, ind, r_mins, minNo] = FinalFormInitialGuess(x, y, theta, tf, lim_r); 
%    an= (v.^2).*k;
    qweAnim(x, y, theta);
    qweAnimtwoLines(x, y, theta, delta_n, minNo);
    traj3 = cell(1,1);
    %  theta  ����ǣ�nSeg_ ����滮�㷨����һ��DP�㷨����ʼ·�㣨���� delta_n  ΪACC·��Ĳ��� ��
    elment.x = x; elment.y = y; elment.theta = theta; elment.nSeg_ =nSeg_;  elment.np = delta_n;  elment.minNo = minNo; 
    traj3{1,1} = elment;
    save traj3;
tic;
%  tic; [~, ~, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta, thetaFrenet); toc; % ����ǰ���ŵ��������ǰ�������꣬���ŵĽǶȣ�Freneti�Ƕȣ��Լ�rmin��CC
% [tun_v, tun_a, tun_phy, tun_w] = FormInitialGuessMid(xr, yr, tf);  % ��� ������ʱ��� �ٶȣ����ٶȣ�ǰ��ת�ǣ�ǰ�ֵ�ת����ٶ�
% % %%
% % plot(xr, yr, 'g');              %  �ڶ��ι켣�滮��   ��������  Ϊ����ɫ���ߡ������ܷ�����ײ������theta�������α߽緽�������������߲�������
% % hold on
% % plot(xf, yf, '-b');              %  �����ι켣�滮��   ǰ������  Ϊ����ɫʵ�ߡ������ܷ�����ײ������theta�������α߽緽�������������߲�������
% % hold on
% %%
% % WriteInitialGuessForFirstTimeNLP(x, y, theta, xr, yr, xf, yf, v, a, phy, w); 
% WriteInitialGuessForFirstTimeNLP(tun_v, tun_a, tun_phy, tun_w); 
% %ԭ�����ߵõ��Ĺ켣��x, y, theta��������õ��ģ�v, a, phy, w����������������xr, yr, xf, yf .�����ļ� initial_guess0.INIVAL
% WriteParameters(tf, Nfe, x(end), y(end), theta(end)); % д���ļ�BV_
% tic
% !ampl rrcir.run
% toc
% [cur_x, cur_y, cur_theta, cur_infeasibility] = LoadStates();
% %  plot(cur_x, cur_y, 'y'); drawnow;   %  2022 03 24
% qweAnim(cur_x, cur_y, cur_theta);      %  2022 03 24
% plot(x, y, 'b'); drawnow;   plot(xr, yr, 'r'); drawnow;   plot(cur_x, cur_y, 'k'); drawnow;   %  2022 03 24
% [cur_v, cur_a, cur_phy, cur_w] = FormInitialGuessMid(cur_x, cur_y, tf);
