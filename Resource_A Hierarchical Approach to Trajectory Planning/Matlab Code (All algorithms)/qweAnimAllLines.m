clear all
close all
clc

hold on;
load  obstacles_
Nobs = 5;
precise_timeline = [0: 0.05: 10];
% global obstacles_ Nobs precise_timeline  nSeg_

Nfe = length(obstacles_{ 1, 1 }.x);
road_left_barrier_x = []; road_left_barrier_y = []; road_right_barrier_x = []; road_right_barrier_y = [];
ref_x = []; ref_y = [];
for ii = -10 : 220
    [xr, yr, lb, rb, ref_theta] = ProvideReferenceLineInfo(ii);
    ref_x = [ref_x, xr]; ref_y = [ref_y, yr];
    road_left_barrier_x = [road_left_barrier_x, xr - lb * cos(pi/2 + ref_theta)];
    road_left_barrier_y = [road_left_barrier_y, yr - lb * sin(pi/2 + ref_theta)];
    road_right_barrier_x = [road_right_barrier_x, xr - rb * cos(pi/2 + ref_theta)];
    road_right_barrier_y = [road_right_barrier_y, yr - rb * sin(pi/2 + ref_theta)];
end
road_left_barrier_x = [road_left_barrier_x, -200, road_left_barrier_x(1)];
road_left_barrier_y = [road_left_barrier_y, 200, road_left_barrier_y(1)];
road_right_barrier_x = [road_right_barrier_x, 200, 200, -200, road_right_barrier_x(1)];
road_right_barrier_y = [road_right_barrier_y, 200, -200, -200, road_right_barrier_y(1)];

% colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39; 255,192,203;218,112,214;123,104,238;0,0,255;0,0,139;119,136,153;30,144,255;70,130,180;0,191,255;0,139,139;255,102,0;0,250,154;127,255,0;154,205,50;255,215,0;205,133,63;128,0,0;0,255,255;240,128,128;255,0,0;105,105,105;169,169,169;192,192,192;0,0,0] ./ 255;
% colorline = [0,255,0;0,0,255;255,0,0;255,0,255] ./ 255;  %  绿,蓝 红  紫
colorline = [0,255,0;0,255,255;255,0,0;0,0,255] ./ 255;  %  绿 青  红 蓝
axis equal
box on
set(gcf,'outerposition',get(0,'screensize'));

fill(road_left_barrier_x, road_left_barrier_y, [0.5, 0.5, 0.5]);
fill(road_right_barrier_x, road_right_barrier_y, [0.5, 0.5, 0.5]);
plot(ref_x, ref_y, 'k--');

axis equal;axis([-2 66 -31 16]);

for num = 1:4
load(['E:\智能车\避障超车\Paper A\Desion+Optimization Paper A (Replan 20220629)\traj',num2str(num),'.mat'])
     nSeg_= elment.nSeg_;      %  nSeg_ 储存下一循环规划起点的序号（向前移动 delta_n节点就是DP规划，  最近的delta_n节点就是ACC规划 ）
     np = elment.np;      
     xcc = elment.x; 
     ycc = elment.y; 
     thetacc = elment.theta; 
     
   if ( ~isempty(nSeg_) )
       for ii = 1:length( nSeg_ )
          if(ii == 1 )
              nStart = 1 ;
              nmiddle = nSeg_(1) - np;
              nend =  nSeg_(1);
          else
              nStart = nSeg_( ii-1 ) ;
              nmiddle = nSeg_(ii) - np;
              nend =  nSeg_(ii); 
          end 
          xDP = xcc(nStart : nmiddle);       
          yDP = ycc(nStart : nmiddle);
          xACC = xcc(nmiddle : nend);      
          yACC = ycc(nmiddle : nend);  
          plot(xDP, yDP,  'color',colorline(num,:));    
          plot(xACC, yACC, 'color',colorline(num,:), 'linestyle', '-.');  
%           plot(xACC, yACC, 'color',colorline(num,:), 'linestyle', '--'); 
      end
      nStart = nSeg_( end ); 
   else
      nStart = 1;
   end
      nmiddle = Nfe;
      xDP = xcc(nStart : nmiddle);  
      yDP = ycc(nStart : nmiddle);
      plot(xDP, yDP,'color', colorline(num,:));   
      plot(xcc(elment.minNo),ycc(elment.minNo), 'color',colorline(num,:), 'marker', 'o');  
%       text(xcc(minNo),ycc(minNo),'o','color','r'); 
end
      
xlabel('X / m');
ylabel('Y / m');

%   for ii = 1:40:Nfe
%         for kk = 1:Nobs
%             cur_obs = obstacles_{ 1, kk };
%             cur_x = cur_obs.x( ii );
%             cur_y = cur_obs.y( ii );
%             cur_theta = cur_obs.theta( ii );
%             V = CreateVehiclePolygon( cur_x, cur_y, cur_theta );
%             fill( V.x, V.y, colorpool( kk, : ) );
%         end
% 
%       V = CreateVehiclePolygon( xcc( ii ), ycc( ii ), thetacc( ii ) );
%       plot( V.x, V.y, 'color', colorpool( 2, : ) );
%       plot( xcc( 1:ii ), ycc( 1:ii ), 'k' );
%   end

function V = CreateVehiclePolygon(x, y, theta)
global vehicle_geometrics_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;   %(x, y)为后桥中心点
AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
V.x = [AX, BX, CX, DX, AX];
V.y = [AY, BY, CY, DY, AY];
end