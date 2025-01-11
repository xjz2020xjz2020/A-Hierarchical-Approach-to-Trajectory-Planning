function obstacles_ = GenerateObstacles(agv_vel)
global dp_ Nobs precise_timeline
Nfe = length(precise_timeline);
tf = dp_.unit_time * dp_.num_t_grids;
obstacles_ = cell(1,1);
ii = 1;
start_s = 10 + 100 * rand;                       % start_s   范围  [20, 130]
end_s = start_s + agv_vel * rand * tf;           % end_s     范围   start_s+[0, 220]=（ 30, 350]，agv_vel 为车速参数
ds = (end_s - start_s) / (Nfe - 1);              % 障碍车距离   m/每格(或0.05s)       (共200格)
offset = rand * 5 - 4;                           % offset    范围  [-4, 1]
if (offset >= -1)
    offset = -rand * 0.1;                        %  概率小，左车道
else
    offset = 2.95 - rand * 0.1;                  %  概率大，右车道
end
x = []; y = []; t = [];
for jj = 1 : Nfe
    [xr, yr, ~, ~, theta] = ProvideReferenceLineInfo(start_s + ds * (jj-1));
    x = [x, xr - offset * cos(pi/2 + theta)];
    y = [y, yr - offset * sin(pi/2 + theta)];
    t = [t, theta];
end
elem.x = x; elem.y = y; elem.theta = t; elem.s = linspace(start_s, end_s, Nfe); elem.l = ones(1,Nfe) .* offset;
obstacles_{1,ii} = elem;
while (ii < Nobs)
    start_s = 10 + 100 * rand;
    end_s = start_s + agv_vel * rand * tf;
    ds = (end_s - start_s) / (Nfe - 1);
    offset = rand * 5 - 4;
    if (offset >= -1)
        offset = -rand * 0.1;
    else
        offset = 2.95 - rand * 0.1;
    end
    x = []; y = []; t = [];
    for jj = 1 : Nfe
        [xr, yr, ~, ~, theta] = ProvideReferenceLineInfo(start_s + ds * (jj-1));
        x = [x, xr - offset * cos(pi/2 + theta)];
        y = [y, yr - offset * sin(pi/2 + theta)];
        t = [t, theta];
    end
    elem.x = x; elem.y = y; elem.theta = t; elem.s = linspace(start_s, end_s, Nfe); elem.l = ones(1,Nfe) .* offset;
    if (IsCurrentVehicleValid(elem, obstacles_))
        ii = ii + 1;
        obstacles_{1,ii} = elem;
    end
end
end

function is_valid = IsCurrentVehicleValid(elem, obs)
Nalready_stored_obstacles = size(obs,2);
Nfe = length(elem.x);
is_valid = 0;
for ii = 1 : Nalready_stored_obstacles
    cur_obs = obs{1,ii};
    for jj = 1 : Nfe
        x_ego = elem.x(jj); y_ego = elem.y(jj); theta_ego = elem.theta(jj);
        x = cur_obs.x(jj); y = cur_obs.y(jj); theta = cur_obs.theta(jj);
        V1 = CreateVehiclePolygon(x_ego, y_ego, theta_ego);
        V2 = CreateVehiclePolygon(x, y, theta);
        if (any(inpolygon(V2.x,V2.y,V1.x,V1.y)))
            return;
        end
        if (any(inpolygon(V1.x,V1.y,V2.x,V2.y)))
            return;
        end
    end
end
is_valid = 1;
end
  
function V = CreateVehiclePolygon(x, y, theta)
global vehicle_geometrics_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;      %（x，y）为后桥中心
AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;   %前 左
BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;   %前 右
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;     %后 右
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;     %后 左
AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;    %前 左
BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;    %前 右
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;     %后 右
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;     %后 左
V.x = [AX, BX, CX, DX, AX];
V.y = [AY, BY, CY, DY, AY];
end