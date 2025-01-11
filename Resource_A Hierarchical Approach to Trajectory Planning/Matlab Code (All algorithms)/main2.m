% ���������߷�:���ڱ��������߹滮�����켣
% ���ߣ�Ally
% ���ڣ�2021/1/30
clc
clear
close all

%% 
% ������Ƶ�
% d = 3.5;
% P0 = [0, -d/2];
% P1 = [25, -d/2];
% P2 = [25, d/2];
% P3 = [50, d/2];
% P = [P0; P1; P2; P3];

%S
%  P0 = [0,     0];
%  P1 = [26.67, 2];
%  P2 = [46.67, 4];
%  P3 = [53.33, 6];
%  P4 = [73.33, 8];
%  P5 = [106.67, 10];
 
 % L
 P0 = [0.78,     0];
 P1 = [1.92, 2];
 P2 = [2.48, 4];
 P3 = [0.29, 6];
 P4 = [-0.32, 8];
 P5 = [0.37, 10];
 
 P = [P0; P1; P2; P3; P4; P5];
 
% ֱ�Ӹ��ݱ��������߶���ʽ�õ�·����
n = length(P)-1;
Path = [];
for t = 0:0.01:1
    if n == 1
        p_t = P;
    elseif n >= 2
        p_t = [0, 0];
        for i = 0:n
            k_C = factorial(n) / (factorial(i) * factorial(n-i));
            k_t = (1-t)^(n-i) * t^i;
            p_t = p_t + k_C * k_t * P(i+1,:);
        end
        Path(end+1,:) = p_t;
    
    else
        disp('���Ƶ�������������������')
    end
end

% 
% %% ��ͼ
% d = 3.5;               % ��·��׼���
% W = 1.8;               % �������
% L = 4.7;               % ����
% figure
% len_line = 50;
% 
% % ����ɫ·��ͼ
% GreyZone = [-5,-d-0.5; -5,d+0.5; len_line,d+0.5; len_line,-d-0.5];
% fill(GreyZone(:,1),GreyZone(:,2),[0.5 0.5 0.5]);
% hold on
% fill([P0(1),P0(1),P0(1)-L,P0(1)-L],[-d/2-W/2,-d/2+W/2,-d/2+W/2,-d/2-W/2],'b')  
% 
% % ���ֽ���
% plot([-5, len_line],[0, 0], 'w--', 'linewidth',2);  %�ֽ���
% plot([-5,len_line],[d,d],'w','linewidth',2);     %��߽���
% plot([-5,len_line],[-d,-d],'w','linewidth',2);  %��߽���
% 
% % ������������ʾ��Χ
% axis equal
% set(gca, 'XLim',[-5 len_line]); 
% set(gca, 'YLim',[-4 4]); 

% ����·��
scatter(P(:,2),P(:,1),'g')
hold on
plot(P(:,2),P(:,1),'r');%·����
% scatter(Path(:,1),Path(:,2),200, '.b');%·����
scatter(Path(:,2),Path(:,1), '.b');%·����
hold off

