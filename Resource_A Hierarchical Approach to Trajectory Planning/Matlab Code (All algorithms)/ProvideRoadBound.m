function [lb, rb] = ProvideRoadBound(s)
lb = -2 - cos(0.2 * s + 2.8166138) * 0.2;
rb = 5 + cos(0.78 * s - 0.8166138) * 0.15;
% lb = -2.2;   % 道路边界 左界 -2 ± 0.2    % 2022 0 324
% rb = 5.15;  % 道路边界 右界 5 ± 0.15
end