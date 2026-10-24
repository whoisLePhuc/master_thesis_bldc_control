function y = bldc_trap(x)
% BLDC_TRAP  Normalized trapezoidal back-EMF shape F(theta):
%   amplitude 1, 120 deg flat top, 60 deg linear slopes, origin at the middle of the rising slope.
x = mod(x, 2*pi); a = pi/6;
y = zeros(size(x));
k = x < a;                        y(k) = x(k)/a;
k = x >= a & x <= pi - a;         y(k) = 1;
k = x > pi - a & x < pi + a;      y(k) = (pi - x(k))/a;
k = x >= pi + a & x <= 2*pi - a;  y(k) = -1;
k = x > 2*pi - a;                 y(k) = (x(k) - 2*pi)/a;
end
