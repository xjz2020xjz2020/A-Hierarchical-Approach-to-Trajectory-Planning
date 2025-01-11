function [BVr, BVf, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta, thetaFrenet)
NE = length(x);
global vehicle_geometrics_
cos_theta=cos(theta) ;
sin_theta=sin(theta) ;
xr = x;    % （xr，yr)后桥中心
yr = y;    
xf = x + vehicle_geometrics_.vehicle_wheelbase .* cos_theta;    % （xf，yf)前桥中心
yf = y + vehicle_geometrics_.vehicle_wheelbase .* sin_theta;
BVr = zeros(NE,8); BVf = zeros(NE,8);    % xmin, xmax, ymin, ymax
rr =zeros(NE,1);
delete('CC');
fid = fopen('CC', 'w');
for ii = 1 : NE
    x = xr(ii); y = yr(ii); theta_i= theta(ii);  % 后桥中心
    cos_theta_i=cos(theta_i) ;
    sin_theta_i=sin(theta_i) ; 
    
    lb = GetBoxVertexes(x, y, theta_i, ii);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = x + counter * 0.1;
                        y_nudge = y;
                    case 2
                        x_nudge = x - counter * 0.1;
                        y_nudge = y;
                    case 3
                        x_nudge = x;
                        y_nudge = y + counter * 0.1;
                    case 4
                        x_nudge = x;
                        y_nudge = y - counter * 0.1;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge, theta_i,  ii);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    xr(ii) = x; yr(ii) = y;                % 后桥中心（有可能变化偏移找中心（如L20-44），故需重新赋值回去 ）

    rr(ii) = 0.5 * lb(1) +  0.5 * lb(3) ;
    
    BVr(ii,1:2) = [x - lb(2) * cos_theta(ii) - lb(1) * sin_theta(ii), y + lb(1) * cos_theta(ii) - lb(2) * sin_theta(ii)];    % 矩形的 左上方点， lb( 上距，左距， 下距，右距 )
    BVr(ii,3:4) = [x + lb(4) * cos_theta(ii) - lb(1) * sin_theta(ii), y + lb(1) * cos_theta(ii) + lb(4) * sin_theta(ii)];    % 矩形的 右上方点，        A|--------|B
    BVr(ii,5:6) = [x + lb(4) * cos_theta(ii) + lb(3) * sin_theta(ii), y - lb(3) * cos_theta(ii) + lb(4) * sin_theta(ii)];    % 矩形的 右下方点,          |   .(xy)|
    BVr(ii,7:8) = [x - lb(2) * cos_theta(ii) + lb(3) * sin_theta(ii), y - lb(3) * cos_theta(ii) - lb(2) * sin_theta(ii)];    % 矩形的 左下方点，        D|--------|C
    
    BVf(ii,1:2) = [BVr(ii,1) + vehicle_geometrics_.vehicle_wheelbase * cos_theta(ii), BVr(ii,2) + vehicle_geometrics_.vehicle_wheelbase * sin_theta(ii)];    % 矩形的 左上方点， lb( 上距，左距， 下距，右距 )
    BVf(ii,3:4) = [BVr(ii,3) + vehicle_geometrics_.vehicle_wheelbase * cos_theta(ii), BVr(ii,4) + vehicle_geometrics_.vehicle_wheelbase * sin_theta(ii)];    % 矩形的 右上方点，        A|--------|B
    BVf(ii,5:6) = [BVr(ii,5) + vehicle_geometrics_.vehicle_wheelbase * cos_theta(ii), BVr(ii,6) + vehicle_geometrics_.vehicle_wheelbase * sin_theta(ii)];    % 矩形的 右下方点,          |   .(xy)|
    BVf(ii,7:8) = [BVr(ii,7) + vehicle_geometrics_.vehicle_wheelbase * cos_theta(ii), BVr(ii,8) + vehicle_geometrics_.vehicle_wheelbase * sin_theta(ii)];    % 矩形的 左下方点，        D|--------|C

    xf(ii) = xr(ii) + vehicle_geometrics_.vehicle_wheelbase * cos_theta(ii) ;       %前桥中心x坐标
    yf(ii) = yr(ii) + vehicle_geometrics_.vehicle_wheelbase * sin_theta(ii) ;       %前桥中心y坐标

%     DrawBox(BVr(ii,:),1);  DrawBox(BVf(ii,:),2);drawnow;   % 后桥中心隧道 g , 前桥中心隧道 b

    fprintf(fid, '%g 1 %f \r\n', ii,  xr(ii) );       % Data is saved in “CC”
    fprintf(fid, '%g 2 %f \r\n', ii,  yr(ii) );
    fprintf(fid, '%g 3 %f \r\n', ii,  xf(ii) );
    fprintf(fid, '%g 4 %f \r\n', ii,  xf(ii) );
    fprintf(fid, '%g 5 %f \r\n', ii,  theta(ii) );
    fprintf(fid, '%g 6 %f \r\n', ii,  thetaFrenet(ii) );
end
fclose(fid);

delete('DD');
fid1 = fopen('DD', 'w');
    fprintf(fid, '1  %f \r\n', min(rr) );
fclose(fid1);

end

function lb = GetBoxVertexes(x,y, theta, time_index)
global vehicle_geometrics_
half_width = vehicle_geometrics_.vehicle_width * 0.5;
rear_length = vehicle_geometrics_.vehicle_rear_hang;
front_length = vehicle_geometrics_.vehicle_wheelbase + vehicle_geometrics_.vehicle_front_hang;
% up left down right
basic_step = 0.1;
% max_step = 4.0;
max_step = [ 3, 3, 3, 7 ];
% lb = ones(1,4) .* vehicle_geometrics_.radius; 
lb = [ half_width, rear_length ,half_width, front_length ];
if (~IsBoxValid(x, y, theta, time_index, lb))
    lb = zeros(1,4);
    return;
end
is_completed = zeros(1,4);     % 判断4个方向检测是否完成的标志位
while (sum(is_completed) < 4)
    for ind = 1 : 4             % 1上距， 2左距， 3下距， 4右距
        if (is_completed(ind))  % 某个方向检测否完成，则跳过
            continue;
        end
        test = lb;
        if (test(ind) + basic_step > max_step(ind))  % 车身在某个方向上最大扩充间距，先后桥，再前桥
            is_completed(ind) = 1;
            continue;
        end
        test(ind) = test(ind) + basic_step;
        if (IsCurrentEnlargementValid(x, y, theta, test, lb, ind, time_index))
            lb = test;
        else
            is_completed(ind) = 1;
        end
    end
end
% lb = lb - vehicle_geometrics_.radius;
lb = [ lb(1) - half_width, lb(2) - rear_length ,lb(3) - half_width, lb(4) - front_length ];
end

function is_valid = IsCurrentEnlargementValid(x, y, theta, test, lb, ind, time_index)
cos_theta = cos(theta);
sin_theta = sin(theta);                                                                  %      A|--------|B ， 与IsBoxValid（）函数一样
switch ind               % 矩形扩充边,  lb( 上距，左距， 下距，右距 )                    %       |   .(xy)|
    case 1      %扩充变化后，AB上扩（test(1)增加）                                       %      D|--------|C
        A = [x - lb(2) * cos_theta - lb(1) * sin_theta, y + lb(1) * cos_theta - lb(2) * sin_theta];           %变化前的A点， 与IsBoxValid（）函数一样
        B = [x + lb(4) * cos_theta - lb(1) * sin_theta, y + lb(1) * cos_theta + lb(4) * sin_theta];           %变化前的B点
        EA = [x - test(2) * cos_theta - test(1) * sin_theta, y + test(1) * cos_theta - test(2) * sin_theta];      %扩充变化后的A点，test(1)test(2)变化，AB上扩
        EB = [x + test(4) * cos_theta - test(1) * sin_theta, y + test(1) * cos_theta + test(4) * sin_theta];      %扩充变化后的B点，test(1)test(4)变化，AB上扩
        V_check = [A; B; EB; EA];
    case 2    %扩充变化后，AD左扩（test(2)增加） 
        A = [x - lb(2) * cos_theta - lb(1) * sin_theta, y + lb(1) * cos_theta - lb(2) * sin_theta];
        D = [x - lb(2) * cos_theta + lb(3) * sin_theta, y - lb(3) * cos_theta - lb(2) * sin_theta];
        EA = [x - test(2) * cos_theta - test(1) * sin_theta, y + test(1) * cos_theta - test(2) * sin_theta];
        ED = [x - test(2) * cos_theta + test(3) * sin_theta, y - test(3) * cos_theta - test(2) * sin_theta];
        V_check = [A; D; ED; EA];
    case 3   %扩充变化后，CD下扩（test(3)增加） 
        C = [x + lb(4) * cos_theta + lb(3) * sin_theta, y - lb(3) * cos_theta + lb(4) * sin_theta];
        D = [x - lb(2) * cos_theta + lb(3) * sin_theta, y - lb(3) * cos_theta - lb(2) * sin_theta]; 
        EC = [x + test(4) * cos_theta + test(3) * sin_theta, y - test(3) * cos_theta + test(4) * sin_theta];
        ED = [x - test(2) * cos_theta + test(3) * sin_theta, y - test(3) * cos_theta - test(2) * sin_theta]; 
        V_check = [C; D; ED; EC];
    case 4    %扩充变化后，BC右扩（test(4)增加） 
        B = [x + lb(4) * cos_theta - lb(1) * sin_theta, y + lb(1) * cos_theta + lb(4) * sin_theta];
        C = [x + lb(4) * cos_theta + lb(3) * sin_theta, y - lb(3) * cos_theta + lb(4) * sin_theta];
        EB = [x + test(4) * cos_theta - test(1) * sin_theta, y + test(1) * cos_theta + test(4) * sin_theta];
        EC = [x + test(4) * cos_theta + test(3) * sin_theta, y - test(3) * cos_theta + test(4) * sin_theta];
        V_check = [C; B; EB; EC];
    otherwise
        is_valid = 0;
        return;
end
global obstacles_ road_barriers_
obs_x = road_barriers_.x; obs_y = road_barriers_.y;
err_distance = abs(obs_x - x) + abs(obs_y - y);
ind = find(err_distance <= 15);
obs_x = obs_x(ind); obs_y = obs_y(ind);
for ii = 1 : size(obstacles_,2)
    cur_obs_x = obstacles_{1,ii}.x(time_index);
    cur_obs_y = obstacles_{1,ii}.y(time_index);
    if (abs(cur_obs_x - x) + abs(cur_obs_y - y) >= 20)
        continue;
    end
    cur_obs_theta = obstacles_{1,ii}.theta(time_index);
    V = CreateVehiclePolygonGrids(cur_obs_x, cur_obs_y, cur_obs_theta);
    obs_x = [obs_x, V.x]; obs_y = [obs_y, V.y];
end
if (any(inpolygon(obs_x, obs_y, V_check(:,1)', V_check(:,2)')))
    is_valid = 0;
else
    is_valid = 1;
end
end

function V = CreateVehiclePolygonGrids(x, y, theta)
global vehicle_geometrics_
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = vehicle_geometrics_.vehicle_width * 0.5;
AX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * cos_theta + vehicle_half_width * sin_theta;
CX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta + vehicle_half_width * sin_theta;
DX = x - vehicle_geometrics_.vehicle_rear_hang * cos_theta - vehicle_half_width * sin_theta;
AY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (vehicle_geometrics_.vehicle_front_hang + vehicle_geometrics_.vehicle_wheelbase) * sin_theta - vehicle_half_width * cos_theta;
CY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta - vehicle_half_width * cos_theta;
DY = y - vehicle_geometrics_.vehicle_rear_hang * sin_theta + vehicle_half_width * cos_theta;
V.x = [linspace(AX, BX, 4), linspace(BX, CX, 4), linspace(CX, DX, 4), linspace(DX, AX, 4)];
V.y = [linspace(AY, BY, 4), linspace(BY, CY, 4), linspace(CY, DY, 4), linspace(DY, AY, 4)];
end

function is_valid = IsBoxValid(x, y, theta, time_index, lb)   
cos_theta = cos(theta);
sin_theta = sin(theta);
C = [x + lb(4) * cos_theta + lb(3) * sin_theta, y - lb(3) * cos_theta + lb(4) * sin_theta];    % 矩形的 右下方点,  lb( 上距，左距， 下距，右距 )
D = [x - lb(2) * cos_theta + lb(3) * sin_theta, y - lb(3) * cos_theta - lb(2) * sin_theta];    % 矩形的 左下方点，      A|--------|B
A = [x - lb(2) * cos_theta - lb(1) * sin_theta, y + lb(1) * cos_theta - lb(2) * sin_theta];    % 矩形的 左上方点，       |   .(xy)|
B = [x + lb(4) * cos_theta - lb(1) * sin_theta, y + lb(1) * cos_theta + lb(4) * sin_theta];    % 矩形的 右上方点，      D|--------|C
Vx = [A(1), B(1), C(1), D(1), A(1)];
Vy = [A(2), B(2), C(2), D(2), A(2)];
global obstacles_ road_barriers_  
obs_x = road_barriers_.x;      % ds = 1m, 起点d=[-10,200],顺道路中心线每次前进1m测算一次道路上下边界   
obs_y = road_barriers_.y;       
err_distance = abs(obs_x - x) + abs(obs_y - y);  % 后桥中心点(x,y) 与道路边界 点列(obs_x, obs_y)  的曼哈顿距离
ind = find(err_distance <= 15);  % ind = find(err_distance <= 10);  original
obs_x = obs_x(ind);
obs_y = obs_y(ind);
for ii = 1 : size(obstacles_,2)
    cur_obs_x = obstacles_{1,ii}.x(time_index);
    cur_obs_y = obstacles_{1,ii}.y(time_index);
    if (abs(cur_obs_x - x) + abs(cur_obs_y - y) >= 20)
        continue;
    end
    cur_obs_theta = obstacles_{1,ii}.theta(time_index);
    V = CreateVehiclePolygonGrids(cur_obs_x, cur_obs_y, cur_obs_theta);
    obs_x = [obs_x, V.x]; obs_y = [obs_y, V.y];
end
if (any(inpolygon(obs_x, obs_y, Vx, Vy)))
    is_valid = 0;
else
    is_valid = 1;
end
end