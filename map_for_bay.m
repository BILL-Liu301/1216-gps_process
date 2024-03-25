clc
clear
tic

% ����
path_bag = '2024-03-25-10-10-21.bag';
init_longitude = 113.4939438; % ����
init_latitude = 23.0423394; % γ��
disp("ROS Bag: " + path_bag);
disp("Init lon&lat: [" + init_latitude + "N, " + init_longitude + "E]")

% ��������
disp("Loading rosbag ...")
bag = rosbag(path_bag);
gps_messages=select(bag,'MessageType','ins_interface_msgs/GpsData');

% ����msg
disp("Loading msgs ...")
gps_data_tmp=readMessages(gps_messages);
gps_data=zeros(length(gps_data_tmp),4);
way_points_x =zeros(length(gps_data_tmp),1);
way_points_y =zeros(length(gps_data_tmp),1);

% ����ת��Ϊgps_data
disp("Building gps_data ...")
for i=1:1:length(gps_data_tmp)
    gps_data(i,1)=gps_data_tmp{i, 1}.TimeStamp;
    gps_data(i,2)=gps_data_tmp{i, 1}.Latitude;%γ��
    gps_data(i,3)=gps_data_tmp{i, 1}.Longitude;%����
    gps_data(i,4)=gps_data_tmp{i, 1}.HeadingAngle;
end

% ����ת��Ϊwan_points_*
disp("Building wan_points_* ...")
for j=1:1:length(gps_data)
     [way_points_x(j),way_points_y(j)] = dataTransform(init_longitude,init_latitude,gps_data(j,3),gps_data(j,2));
end

% ��������
save('way_point.mat','way_points_x','way_points_y','gps_data')
disp("Finish, Saved way_point.mat")
disp("Please run 'bay.m' !")

toc

% ��γ�Ⱥ�xy����任
function [x_local,y_local]=dataTransform(L0,B0,L,B)
a = 6378137;
b = 6356752;
c = tan(B0*pi/180);
d = tan(B*pi/180);
e = 1/c;
f = 1/d;
x0 = a * a / sqrt(a * a + b * b * c * c);
y0 = b * b / sqrt(b * b + a * a * e * e);
x1 = a * a / sqrt(a * a + b * b * d * d);
y1 = b * b / sqrt(b * b + a * a * f * f);
yyy = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
if (B <= B0)
  yyy = -yyy;
end
xxx = (L - L0) * x0 * pi / 180;
x_local = xxx;
y_local = yyy;

end
