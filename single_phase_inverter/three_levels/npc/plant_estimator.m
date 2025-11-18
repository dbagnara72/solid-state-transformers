clear all
close all
clc
beep off

T = 20e-3;
tc = 1e-5;
time = [0:tc:2*T-tc];
theta = 2*pi/T*time;
Um = 1;
ualpha = Um*cos(theta);
ubeta = Um*sin(theta);
R = 1;
XL = 2*pi/T*1e-3;
I = Um/(R + 1i*XL);
phi = angle(I)
Im = abs(I)
id = Im * cos(phi)
iq = Im * sin(phi)

ualpha = cos(theta);
ubeta = sin(theta);
ialpha = Im * cos(theta - phi);
ibeta = Im * sin(theta - phi);

udt = ualpha.*cos(theta) + ubeta.*sin(theta);
uqt = -ualpha.*sin(theta) + ubeta.*cos(theta);

idt = ialpha.*cos(theta) + ibeta.*sin(theta);
iqt = -ialpha.*sin(theta) + ibeta.*cos(theta);

idt_mean = mean(idt)
iqt_mean = mean(iqt)
phi_est = atan(iqt_mean/idt_mean)
Im_est = sqrt(idt_mean^2+iqt_mean^2)
Z_est = Um/Im_est;
R_est = Z_est * cos(phi_est)
L_est = Z_est * sin(phi_est) /(2*pi/T)


