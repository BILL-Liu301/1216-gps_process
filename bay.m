clear;
clc;
tic

% 参数
path_txt = '2024-03-25-10-10-21.txt';
path_mat = 'way_point.mat';
disp("Aim txt: " + path_txt);

% 加载数据
disp("Loading&Deleting way_point.mat")
load way_point.mat
delete(path_mat)
X = way_points_x;
Y = way_points_y;

% 坐标变换，转为sn坐标系
disp("Transforming: [xy -> sn] ...")
[s2, n, l] = xy2sn(X,Y,X,Y,1);

% 过滤间距小于4米的点
disp("Building s1, s2, X0, Y0 ...")
k = 2;
s2_temp = s2(1);
s(1) = s2(1);
X0(1) = X(1);
Y0(1) = Y(1);
for i=2:1:length(s2)
    if s2(i) ~= s2_temp && s2(i) > (s2_temp + 2)
        s(k) = s2(i);
        X0(k) = X(i);
        Y0(k) = Y(i);
        s2_temp = s2(i);
        k = k+1;
    end
end

% 拟合曲线
disp("Fitting ...")
px = csape(s,X0);
py = csape(s,Y0);

% 求导
disp("DAOing ...")
S_temp = 0: 1 :s2(end);
diffx1 = fnval(fnder(px,1),S_temp); %求X的一阶导
diffx2 = fnval(fnder(px,2),S_temp); %求X的二阶导
diffy1 = fnval(fnder(py,1),S_temp); %求Y的一阶导
diffy2 = fnval(fnder(py,2),S_temp); %求Y的二阶导

% 求曲率
disp("Building curvature ...")
curvature_temp = (diffy2 .* diffx1) ./ (diffx1 .* diffx1 + diffy1 .* diffy1).^1.5;
pc = csape(S_temp,curvature_temp);

% 求路径点
disp("Building S, X1, Y1, curvature ...")
S = 0: 0.2 :s2(end);
X1 = fnval(px,S);
Y1 = fnval(py,S);
curvature = fnval(pc,S);

%航向角
disp("Building th ...")
for i=1:length(Y1)-1
    dY1(i)=Y1(i+1)-Y1(i);
    dX1(i)=X1(i+1)-X1(i);
    dY_X(i)=dY1(i)/dX1(i);
    th0(i)=-atan2(dY1(i),dX1(i))*180/pi+180;
    th1(i)=th0(i)+gps_data(1,4)-th0(1);
    if th1(i) > 360
        th(i)=th1(i)-360;
    elseif th1(i) < 0
        th(i)=th1(i)+360;
    else
        th(i)=th1(i);
    end
end
if th1(end) > 360
    th1(end)=th1(end)-360;
elseif th1(end) < 0
    th1(end)=th1(end)+360;
end
th = [th th1(end)];

% 计算边界
disp("Building boundary ...")
N_L1 = ones(1,length(S))*3.50;
N_R1 = ones(1,length(S))*-3.50;

[ X_L(1:length(S)), Y_L(1:length(S)) ] = sn2xy(S(1:length(S))', N_L1(1:length(S))', X1(1:length(S))', Y1(1:length(S))');
[ X_R(1:length(S)), Y_R(1:length(S)) ] = sn2xy(S(1:length(S))', N_R1(1:length(S))', X1(1:length(S))', Y1(1:length(S))');

% 画图
disp("Ploting")
figure(1);
plot(X,Y)
hold on
plot(X1,Y1,'.')
plot(X_L,Y_L)
plot(X_R,Y_R)
axis equal
hold off

% 保存数据
disp("Saving to " + path_txt + "...")
fid=fopen(path_txt,'w+'); 
for i=1:1:length(X1)
    fprintf(fid, '%d,%f,%f,%f,%f,%g,%f,%f,%f,%f\n',i,X1(i),Y1(i),S(i),th(i),curvature(i),X_L(i),Y_L(i),X_R(i),Y_R(i));
end
fclose(fid);

toc