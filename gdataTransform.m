function [x_local,y_local]=dataTransform(L0,B0,L,B,heading)
a = 6378137;
b = 6356752;
c = tan(B0*pi/180);
d = tan(B*pi/180);
e = 1/c;
f = 1/d;
temp = 0;
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
% if heading > 180
%     temp= temp - 360;
% else
%     temp = heading;
% end
% x_local = cos(temp / 180 * pi) * yyy + sin(temp / 180 * pi) * xxx;
% y_local = sin(temp / 180 * pi) * yyy - cos(temp / 180 * pi) * xxx;
end