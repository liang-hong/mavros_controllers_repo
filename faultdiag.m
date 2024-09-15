clear variables
close all
clc
 
load ('./diag_data.mat', 'data');
whos data

% 数据拆分
data = data';
passage = 10:260;
% passage = 795:990;
time = data(1,passage);
mean = data(2,passage);
variance = data(3,passage);
mean_mean = data(4,passage);
vari_mean = data(5,passage);
faultState = data(6,passage);
faultDiagState = data(7,passage);
% time = data(1,:);
% mean = data(2,:);
% variance = data(3,:);
% mean_mean = data(4,:);
% vari_mean = data(5,:);
% faultState = data(6,:);
% faultDiagState = data(7,:);

% 辅助参考
mean_on = 0.7 * ones(size(time));
vari_1 = 0.25 * ones(size(time));
vari_2 = 0.8 * ones(size(time));
mean_off = 0.4 * ones(size(time));
vari_off = 0.15 * ones(size(time));
zzz = zeros(size(time));

StartTime = time(1);
EndTime = time(end);
VauleMax = 1.8;
VauleMin = -0.2;
FaceAlpha = 0.2;
FaultArea = struct('Color', {}, 'Xtimes', {}, 'Yvalues', {});
DiagArea = struct('Color', {}, 'Xtimes', {}, 'Yvalues', {});
FaultTemp = [];
DiagTemp = [];
for i = 2:length(time)
    if faultState(i-1)==0 && faultState(i)~=0
        FaultTemp(:,1) = [time(i); VauleMin];
        FaultTemp(:,2) = [time(i); 0];
    elseif faultState(i-1)~=0 && faultState(i)==0
        FaultTemp(:,3) = [time(i); 0];
        FaultTemp(:,4) = [time(i); VauleMin];
        FaultArea = [FaultArea, struct('Color', selectColor(faultState(i-1)), ...
            'Xtimes', FaultTemp(1,:), 'Yvalues', FaultTemp(2,:))];
    end

    if faultDiagState(i-1)==0 && faultDiagState(i)~=0
        DiagTemp(:,1) = [time(i); VauleMax];
        DiagTemp(:,2) = [time(i); 0];
    elseif faultDiagState(i-1)~=0 && faultDiagState(i)==0
        DiagTemp(:,3) = [time(i); 0];
        DiagTemp(:,4) = [time(i); VauleMax];
        DiagArea = [DiagArea, struct('Color', selectColor(faultDiagState(i-1)), ...
            'Xtimes', DiagTemp(1,:), 'Yvalues', DiagTemp(2,:))];
    end
end

% 开始绘图
figure; % 创建一个新的图形窗口
set(gcf,'unit','normalized','position', [0.2,0.2,0.36,0.36]);
hold on; % 保持当前图形不变

% % 绘制阴影
% for i = 1:length(FaultArea)
%     fill(FaultArea(i).Xtimes, FaultArea(i).Yvalues, FaultArea(i).Color, ...
%         'EdgeColor','none', 'FaceAlpha',FaceAlpha);
% end
% for i = 1:length(DiagArea)
%     fill(DiagArea(i).Xtimes, DiagArea(i).Yvalues, DiagArea(i).Color, ...
%         'EdgeColor','none', 'FaceAlpha',FaceAlpha);
% end

ckz = plot(time, zzz, 'k-', 'HandleVisibility','off', 'LineWidth', 1);

% 绘制data
% mean0 = plot(time, mean, 'b-', 'DisplayName', 'mean', 'LineWidth', 1.5); 
% vari0 = plot(time, variance, 'g-', 'DisplayName', 'variance', 'LineWidth', 1.5); 
mean1 = plot(time, mean_mean, 'b-', 'DisplayName', '误差', 'LineWidth', 1.5); 
% vari1 = plot(time, vari_mean, 'm-*', 'DisplayName', 'variance', 'LineWidth', 1.5); 
% divi = plot(time, vari_mean./mean_mean, 'k-', 'DisplayName', 'divis', 'LineWidth', 1.5); 
% fault = plot(time, faultState, 'r-', 'DisplayName', 'faultState', 'LineWidth', 1.5);
% diag = plot(time, faultDiagState, 'k-', 'DisplayName', 'faultDiagState', 'LineWidth', 1.5);

% 绘制诊断标准
ckmu = plot(time, mean_on, 'b--', 'DisplayName', '误差阈值', 'LineWidth', 1.5);
% ckv1 = plot(time, vari_1, 'm--', 'DisplayName', 'vari\_1', 'LineWidth', 1.5);
% ckv2 = plot(time, vari_2, 'm--', 'DisplayName', 'vari\_2', 'LineWidth', 1.5);
% ckmd = plot(time, mean_off, 'b--',  'DisplayName', 'mean\_off', 'LineWidth', 1.5);
% ckvd = plot(time, vari_off, 'm--',  'DisplayName', 'vari\_off', 'LineWidth', 1.5);

% 绘制诊断时刻
faultLine = [];
diagLine = [];
for i = 2:length(time)
    if faultState(i-1) == 0 && faultState(i) ~= 0
        faultLine(end+1) = plot([time(i), time(i)], [0, VauleMax], 'k-', ...
            'DisplayName', '故障开始时刻', 'LineWidth', 1.5);
        text(time(i)+0.1, 1.2, sprintf('%.3f', time(i)), ...
            'HorizontalAlignment', 'left', 'Color', 'k');
    end
    if faultDiagState(i-1) == 0 && faultDiagState(i) ~= 0
        diagLine(end+1) = plot([time(i), time(i)], [0, VauleMax], 'r-', ...
            'DisplayName', '诊断时刻', 'LineWidth', 1.5);
        text(time(i)+0.1, 1.2, sprintf('%.3f', time(i)), ...
            'HorizontalAlignment', 'left', 'Color', 'r');
    end
end


legend([ckmu, mean1, faultLine(1), diagLine(1)]);
xlabel('时间/s');
title('故障诊断结果');
ylim([VauleMin VauleMax]); % 设置y轴的范围
grid on;


%%
function result = selectColor(faultstate)
    if faultstate == 1
        result = 'r';
    elseif faultstate == 2
        result = 'b';
    else
        result = 'w';
    end
end

