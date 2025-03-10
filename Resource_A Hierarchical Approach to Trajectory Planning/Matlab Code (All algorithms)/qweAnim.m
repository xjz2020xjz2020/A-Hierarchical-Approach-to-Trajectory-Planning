function qweAnim(xcc, ycc, thetacc)
%close all;
hold on;
global obstacles_ Nobs precise_timeline
Nfe = length(xcc);
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

colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39; 255,192,203;218,112,214;123,104,238;0,0,255;0,0,139;119,136,153;30,144,255;70,130,180;0,191,255;0,139,139;255,102,0;0,250,154;127,255,0;154,205,50;255,215,0;205,133,63;128,0,0;0,255,255;240,128,128;255,0,0;105,105,105;169,169,169;192,192,192;0,0,0] ./ 255;
axis equal
box on
set(gcf,'outerposition',get(0,'screensize'));

fill(road_left_barrier_x, road_left_barrier_y, [0.5, 0.5, 0.5]);
fill(road_right_barrier_x, road_right_barrier_y, [0.5, 0.5, 0.5]);
plot(ref_x, ref_y, 'k--');

axis equal;axis([-2 66 -31 16]);

% for ii = 1 : Nfe
%     for kk = 1 : Nobs
%         cur_obs = obstacles_{1,kk};
%         cur_x = cur_obs.x(ii);
%         cur_y = cur_obs.y(ii);
%         cur_theta = cur_obs.theta(ii);
%         V = CreateVehiclePolygon(cur_x, cur_y, cur_theta);
%         fill(V.x, V.y, colorpool(kk,:));
% 
%     end
%     V = CreateVehiclePolygon(xcc(ii), ycc(ii), thetacc(ii));
%     plot(V.x, V.y, 'k');
% 
% end
% plot(xcc, ycc, 'r');
% xlabel('X / m');
% ylabel('Y / m');

MakeGif('Trajectory Animation.gif',1);
hold on

%  pause(16);
for ii = 1:Nfe
      fill( road_left_barrier_x, road_left_barrier_y, [ 0.5, 0.5, 0.5 ] );
      fill( road_right_barrier_x, road_right_barrier_y, [ 0.5, 0.5, 0.5 ] );
      plot( ref_x, ref_y, 'k--' );   
      for kk = 1:Nobs
          cur_obs = obstacles_{ 1, kk };
          cur_x = cur_obs.x( ii );
          cur_y = cur_obs.y( ii );
          cur_theta = cur_obs.theta( ii );
          V = CreateVehiclePolygon( cur_x, cur_y, cur_theta );
          fill( V.x, V.y, colorpool( kk, : ) );
      end 
      axis equal; axis([-2 66 -31 16]);
      h1 = get( gca, 'children' );

      V = CreateVehiclePolygon( xcc( ii ), ycc( ii ), thetacc( ii ) );
      plot( V.x, V.y, 'color', colorpool( 2, : ) );
     %  plot( xcc( 1:ii ), ycc( 1:ii ), 'k' );
      plot( xcc( 1:ii ), ycc( 1:ii ), '--r' );           %  第一次轨迹规划线   后桥中心  为“红色虚线”
      axis equal; axis([-2 66 -31 16]);
      xlabel('X / m');
      ylabel('Y / m');
      
      stringName = "Trajectory Animation：t =" + num2str(ii);
      title(stringName)
      MakeGif('Trajectory Animation.gif',ii*100+1);
      
      h2 = get( gca, 'children' );
      M( ii ) = getframe;

      if ( ii ~= Nfe )
          delete( h1 );
          delete( h2 );
      end 
end 
%  pause(15);

%    显示规划点【1， 40， 80， 120， 160， 210】的6车状态，ego车的s[5 4 2 4 6]
%    ego车的s为[5 4 2 4 6]，3-4点间距最小， 6-7点间距最大; l为[5 6 3 2 3] 
for ii = 1:40:Nfe
      for kk = 1:Nobs
          cur_obs = obstacles_{ 1, kk };
          cur_x = cur_obs.x( ii );
          cur_y = cur_obs.y( ii );
          cur_theta = cur_obs.theta( ii );
          V = CreateVehiclePolygon( cur_x, cur_y, cur_theta );
          fill( V.x, V.y, colorpool( kk, : ) );
      end

      V = CreateVehiclePolygon( xcc( ii ), ycc( ii ), thetacc( ii ) );
      plot( V.x, V.y, 'color', colorpool( 2, : ) );
      plot( xcc( 1:ii ), ycc( 1:ii ), 'k' );
  end
end

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