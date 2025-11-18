% clear all
% close all
% clc


torque_reference = [100 200 300 400 500 600 700 800 900 1000 1100 1200 ...
    1250];
rpm_base = [10 800 900 1000 1100 1200 1300 1400 1500 1600 1700 1800 1900 ...
    2000 2100 2140];
[m,uit]=size(torque_reference);
[m,uiw]=size(rpm_base);
%% data1
rpm_1_data = [2140 2140 2140 2140 2140 2140 2140];
torque_1_data = [660.1 600.0 500.1 400.1 300.1 200.1 100.0];
consumption_1_data = [220.3 224.2 233.6 247.3 274.3 323.7 476.3];

torque_1_temp = [torque_1_data];
consumption_1_temp = [consumption_1_data];
f=fit(torque_1_temp',consumption_1_temp','spline');

consumption_1 = f(torque_reference);

% figure; 
% plot(torque_1_data, consumption_1_data); grid on
% hold on
% plot(torque_reference, consumption_1); grid on
% hold off
% set(gca,'xlim',[0 1350]);
% 


%% data2
rpm_2_data = [2100 2100 2100 2100 2100 2100 2100 2100];
torque_2_data = [796.0 700.0 600.0 500.0 400.0 300.0 200.0 100.1];
consumption_2_data = [214.6 216.9 222.5 231.7 244.7 266.9 309.9 454.8];
% torque_2_data = fliplr(torque_2_data);
% consumption_2_data = fliplr(consumption_2_data);

torque_2_temp = [1250 1200 1100 1000 900 torque_2_data];
consumption_2_temp = [480 360 290 240 220 consumption_2_data];
f=fit(torque_2_temp',consumption_2_temp','spline');

consumption_2 = f(torque_reference);

% figure; 
% plot(torque_2_data, consumption_2_data); grid on
% hold on
% plot(torque_reference, consumption_2); grid on
% hold off
% set(gca,'xlim',[0 1350]);

%% data3
rpm_3_data = [2000 2000 2000 2000 2000 2000 2000 2000 2000 2000];
torque_3_data = [908.0 900.0 799.9 700.1 600.0 500.0 400.0 300.0 200.2 100.0];
consumption_3_data = [209.0 209.3 210.5 213.2 217.9 225.2 238.0 259.1 300.1 436.1];

torque_3_temp = [1250 1200 1100 1000 torque_3_data];
consumption_3_temp = [280 255 230 215 consumption_3_data];
f=fit(torque_3_temp',consumption_3_temp','spline');

% f=fit(torque_3_data',consumption_3_data','spline');
consumption_3 = f(torque_reference);

% figure; 
% plot(torque_3_data, consumption_3_data); grid on
% hold on
% plot(torque_reference, consumption_3); grid on
% hold off
% set(gca,'xlim',[0 1350]);


%% data4
rpm_4_data = [1900 1900 1900 1900 1900 1900 1900 1900 1900 1900 1900];
torque_4_data = [1020.0 1000.0 900.0 800.0 700.0 600.0 500.0 400.0 300.0 200.2 100.4];
consumption_4_data = [205.3 205.3 205.5 206.8 209.6 214.0 220.6 232.4 250.1 290.9 422.3];
% torque_4_data = fliplr(torque_4_data);
% consumption_4_data = fliplr(consumption_4_data);
% f=fit(torque_4_data',consumption_4_data','spline');
torque_4_temp = [1250 1200 1100 torque_4_data];
consumption_4_temp = [240 224 212 consumption_4_data];
f=fit(torque_4_temp',consumption_4_temp','spline');
consumption_4 = f(torque_reference);

% figure; 
% plot(torque_4_data, consumption_4_data); grid on
% hold on
% plot(torque_reference, consumption_4); grid on
% hold off
% set(gca,'xlim',[0 1350]);


%% data5
rpm_5_data = [1800 1800 1800 1800 1800 1800 1800 1800 1800 1800 1800];
torque_5_data = [1071.0 1000.0 900.0 800.0 700.0 600.0 500.0 400.0 300.0 200.2 100.2];
consumption_5_data = [201.6 202.1 202.7 204.3 206.6 210.7 216.7 226.9 243.3 283.0 417.2];

% f=fit(torque_5_data',consumption_5_data','spline');
torque_5_temp = [1250 1200 1100 torque_5_data];
consumption_5_temp = [206 203 201.4 consumption_5_data];
f=fit(torque_5_temp',consumption_5_temp','spline');

consumption_5 = f(torque_reference);

% figure; 
% plot(torque_5_data, consumption_5_data); grid on
% hold on
% plot(torque_reference, consumption_5); grid on
% hold off
% set(gca,'xlim',[0 1350]);

%% data6
rpm_6_data = [1700 1700 1700 1700 1700 1700 1700 1700 1700 1700 1700 1700];
torque_6_data = [1124 1100 1000 900 800 700 600 500 400 300 200 100];
consumption_6_data = [199 199 199 200 201 204 207 213 222 237 272 391];
% torque_6_data = fliplr(torque_6_data);
% consumption_6_data = fliplr(consumption_6_data);
% f=fit(torque_6_data',consumption_6_data','spline');

torque_6_temp = [1250 1200 torque_6_data];
consumption_6_temp = [201 200 consumption_6_data];
f=fit(torque_6_temp',consumption_6_temp','spline');
consumption_6 = f(torque_reference);
% 
% figure; 
% plot(torque_6_data, consumption_6_data); grid on
% hold on
% plot(torque_reference, consumption_6); grid on
% hold off
% set(gca,'xlim',[0 1350]);


%% data7
rpm_7_data = [1600 1600 1600 1600 1600 1600 1600 1600 1600 1600 1600 1600];
torque_7_data = [1176.0 1099.9 1000.0 900.0 800.0 700.0 600.0 500.0 400.0 300.0 200.1 99.6];
consumption_7_data = [197.0 196.9 196.8 197.4 198.5 200.2 203.3 208.7 217.7 231.7 265.7 400.0];

torque_7_temp = [1250 1200 torque_7_data];
consumption_7_temp = [198 197.4 consumption_7_data];
f=fit(torque_7_temp',consumption_7_temp','spline');
% f=fit(torque_7_data',consumption_7_data','spline');
consumption_7 = f(torque_reference);
% 
% figure; 
% plot(torque_7_data, consumption_7_data); grid on
% hold on
% plot(torque_reference, consumption_7); grid on
% hold off
% set(gca,'xlim',[0 1350]);
% 

%% data8
rpm_8_data = [1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500 1500];
torque_8_data = [1224.0 1199.9 1099.9 1000.0 899.9 800.0 700.0 600.0 500.0 400.0 300.0 200.0 100.0];
consumption_8_data = [195.2 194.8 195.0 194.8 195.3 196.3 197.4 201.0 206.4 213.9 226.4 262.3 395.0];
% torque_8_data = fliplr(torque_8_data);
% consumption_8_data = fliplr(consumption_8_data);
f=fit(torque_8_data',consumption_8_data','spline');
consumption_8 = f(torque_reference);

% figure; 
% plot(torque_8_data, consumption_8_data); grid on
% hold on
% plot(torque_reference, consumption_8); grid on
% hold off
% set(gca,'xlim',[0 1350]);
% 

%% data9
rpm_9_data = [1400 1400 1400 1400 1400 1400 1400 1400 1400 1400 1400 1400 1400];
torque_9_data = [1255.0 1199.9 1100.0 1000.0 900.0 800.0 700.0 600.0 500.0 400.0 300.0 199.7 100.0];
consumption_9_data = [193.6 193.4 193.2 192.9 193.4 194.5 195.6 198.8 204.2 211.1 223.1 254.9 391.5];
f=fit(torque_9_data',consumption_9_data','spline');
consumption_9 = f(torque_reference);

% figure; 
% plot(torque_9_data, consumption_9_data); grid on
% hold on
% plot(torque_reference, consumption_9); grid on
% hold off
% set(gca,'xlim',[0 1350]);

%% data10
rpm_10_data = [1300 1300 1300 1300 1300 1300 1300 1300 1300 1300 1300 1300 1300];
torque_10_data = [1251.0 1200.0 1100.0 1000.0 900.0 800.0 700.0 600.0 500.0 400.0 300.0 200.0 99.2];
consumption_10_data = [191.9 191.8 191.5 192.0 192.7 193.9 195.6 198.1 201.9 208.6 219.8 242.8 381.5];
% torque_10_data = fliplr(torque_10_data);
% consumption_10_data = fliplr(consumption_10_data);
f=fit(torque_10_data',consumption_10_data','spline');
consumption_10 = f(torque_reference);

% figure; 
% plot(torque_10_data, consumption_10_data); grid on
% hold on
% plot(torque_reference, consumption_10); grid on
% hold off
% set(gca,'xlim',[0 1350]);

%% data11
rpm_11_data = [1200 1200 1200 1200 1200 1200 1200 1200 1200 1200 1200 1200 1200];
torque_11_data = [1251.0 1200.0 1100.0 1000.0 900.0 800.0 700.0 600.0 500.0 400.0 300.0 200.0 100.0];
consumption_11_data = [191.3 191.3 191.5 191.9 192.6 193.7 195.5 197.9 200.5 206.6 217.5 239.6 398.9];
f=fit(torque_11_data',consumption_11_data','spline');
consumption_11 = f(torque_reference);

% figure; 
% plot(torque_11_data, consumption_11_data); grid on
% hold on
% plot(torque_reference, consumption_11); grid on
% hold off
% set(gca,'xlim',[0 1350]);

%% data12
rpm_12_data = [1100 1100 1100 1100 1100 1100 1100 1100 1100 1100 1100 1100 1100];
torque_12_data = [1251.0 1200.0 1100.0 1000.0 900.0 800.0 700.0 600.0 500.0 400.0 300.0 200.0 100.0];
consumption_12_data = [191.7 192.0 192.4 192.9 193.8 195.1 196.0 198.2 201.8 206.5 214.3 233.9 381.4];
% torque_12_data = fliplr(torque_12_data);
% consumption_12_data = fliplr(consumption_12_data);
f=fit(torque_12_data',consumption_12_data','spline');
consumption_12 = f(torque_reference);

% figure; 
% plot(torque_12_data, consumption_12_data); grid on
% hold on
% plot(torque_reference, consumption_12); grid on
% hold off
% set(gca,'xlim',[0 1350]);


%% data13
rpm_13_data = [1000 1000 1000 1000 1000 1000 1000 1000 1000 1000 1000];
torque_13_data = [1050.0 999.9 900.0 799.9 700.0 600.0 500.0 400.0 300.0 200.0 100.0];
consumption_13_data = [195.1 194.5 195.8 196.7 198.3 200.2 202.1 204.9 212.2 232.9 294.5];
torque_13_temp = [torque_13_data];
consumption_13_temp = [consumption_13_data];
f=fit(torque_13_temp',consumption_13_temp','spline');
consumption_13 = f(torque_reference);

% figure; 
% plot(torque_13_data, consumption_13_data); grid on
% hold on
% plot(torque_reference, consumption_13); grid on
% hold off
% set(gca,'xlim',[0 1350]);


%% data14
rpm_14_data = [900 900 900 900 900 900 900 900 900 900];
torque_14_data = [934.1 900.0 800.0 700.0 600.0 500.0 400.0 300.0 200.0 100.0];
consumption_14_data = [203.0 202.2 201.5 201.4 202.5 203.7 206.3 211.6 228.7 285.6];
torque_14_temp = [torque_14_data];
consumption_14_temp = [consumption_14_data];

% torque_14_temp = fliplr(torque_14_temp);
% consumption_14_temp = fliplr(consumption_14_temp);
f=fit(torque_14_temp',consumption_14_temp','spline');
consumption_14 = f(torque_reference);

% figure; 
% plot(torque_14_data, consumption_14_data); grid on    
% hold on
% plot(torque_reference, consumption_14); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data15
rpm_15_data = [800 800 800 800 800 800 800 800];
torque_15_data = [780.1 700.0 600.0 500.0 400.0 300.0 200.0 100.0];
consumption_15_data = [211.3 206.5 205.3 206.2 208.2 213.5 229.3 280.3];
f=fit(torque_15_data',consumption_15_data','spline');
consumption_15 = f(torque_reference);

% figure; 
% plot(torque_15_data, consumption_15_data); grid on    
% hold on
% plot(torque_reference, consumption_15); grid on
% hold off
% set(gca,'xlim',[0 1350]);

%% torque and power curve

torque_friction_base = -(rpm_base./60*2*pi)*0.15-(rpm_base./60*2*pi).^2*2.5e-4; %torque friction at shaft

torque_base = [50 max(torque_15_data) max(torque_14_data) ...
    max(torque_13_data) max(torque_12_data) max(torque_11_data) ...
    max(torque_10_data) max(torque_9_data) max(torque_8_data) ...
    max(torque_7_data) max(torque_6_data) max(torque_5_data) ...
    max(torque_4_data) max(torque_3_data) max(torque_2_data) ...
    max(torque_1_data)]; % torque available at shaft after the friction

torque_e = torque_base-torque_friction_base; % torque generated by the engine before the friction

% figure; 
% plot(rpm_base, torque_e,'--k','LineWidth',1.8);
% hold on
% plot(rpm_base, torque_base,'-k','LineWidth',1.8);
% hold on
% plot(rpm_base, torque_friction_base,'-','LineWidth',1.8,'Color',[0.5 0.5 0.5]);
% hold off
% legend('$\hat{\tau}_{e}$','$\hat{\tau}_{d}$','$\tau_{b}$','Interpreter','latex','Location','best');
% set(gca,'xlim',[rpm_base(1) rpm_base(end)]); 
% hchild = get(gca,'Children');
% set(gca,'Children',[hchild(3) hchild(2) hchild(1)])
% ylabel('$\tau/Nm$','Interpreter','latex')
% xlabel('$n/rpm$','Interpreter','latex')
% title('\textbf{Liebherr D934 Engine - Torque and friction curves}','Interpreter','latex')
% grid on
% print -dpdf engine_D934_torque
% print -depsc engine_D934_torque

% figure; 
% plot(rpm_base, torque_e.*rpm_base/60*2*pi*1e-3,'k-','LineWidth',1.8);
% hold on
% plot(rpm_base, torque_base.*rpm_base/60*2*pi*1e-3,'-','LineWidth',1.8,'Color',[0.5 0.5 0.5]);
% hold off
% legend('$P_e$','$P_{d}$','Interpreter','latex','Location','best');
% set(gca,'xlim',[rpm_base(1) rpm_base(end)]); 
% ylabel('$P/kW$','Interpreter','latex')
% xlabel('$n/rpm$','Interpreter','latex')
% title('\textbf{Liebherr D934 Engine - Power curves}','Interpreter','latex')
% grid on
% print -dpdf engine_D934_power
% print -depsc engine_D934_power


%% consumption 3D map
rpm_consumption = rpm_base(2:end); % input speed for engine consumption
torque_consumption = torque_reference;
consumption = [consumption_1, consumption_2, consumption_3, consumption_4, ...
        consumption_5, consumption_6, consumption_7, consumption_8, ...
        consumption_9, consumption_10, consumption_11, consumption_12, ...
        consumption_13, consumption_14, consumption_15 ...
        ]; % Engine consumption as function of speed and power

V = [120 110 105 102 100 98 96 94 92 90 88]./100*220;

% figure;
% [M,c] = contour(rpm_consumption,torque_reference,consumption, V...
%     ,'ShowText','on');
% c.LineWidth = 2;
% colormap(gray(256));
% grid on
% ylabel('$\tau/Nm$','Interpreter','latex')
% xlabel('$n/rpm$','Interpreter','latex')
% title('\textbf{Liebherr D934 Engine - Consumption $\mathbf{[g/kWh]}$}','Interpreter','latex')
% print -dpdf engine_D934_consumption
% print -depsc engine_D934_consumption





