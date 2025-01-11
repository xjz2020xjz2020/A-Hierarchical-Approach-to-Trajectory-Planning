% saveObstacles 函数
function saveObstacles(obstacles, filename)
    fileID = fopen(filename, 'w');
    if fileID == -1
        error('Failed to open file for writing.');
    end
    
    for i = 1:length(obstacles)
        elem = obstacles{i};
        for j = 1:length(elem.x)
            fprintf(fileID, '%.6f %.6f %.6f %.6f %.6f\n', elem.x(j), elem.y(j), elem.theta(j), elem.s(j), elem.l(j));
        end
        fprintf(fileID, '\n'); % 添加空行分隔不同的ObstacleElement
    end
    
    fclose(fileID);
end