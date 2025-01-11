function road_barriers_ = GenerateRoadBarrierGrids()
llx = []; lly = []; uux = []; uuy = [];
ds = 1.0; d = -10;   % 理论道路最大长度为200m=40m*5，% 显示道路最大长度为 [-10, 200] ,画图时还会裁掉一些路段
while (1)             
    [xr, yr, lb, rb, theta] = ProvideReferenceLineInfo(d);
    d = d + ds;        % ds = 1m, 起点d=-10,顺道路中心线每次前进1m测算一次道路上下边界       
    llx = [llx, xr - lb * cos(pi/2 + theta)];     % （llx， lly）道路上边界点，210个坐标点
    lly = [lly, yr - lb * sin(pi/2 + theta)];
    uux = [uux, xr - rb * cos(pi/2 + theta)];     % （uux， uuy）道路下边界点，210个坐标点
    uuy = [uuy, yr - rb * sin(pi/2 + theta)];
    if (d >= 200)
        break;
    end
end
road_barriers_.x = [llx, uux];
road_barriers_.y = [lly, uuy];
end