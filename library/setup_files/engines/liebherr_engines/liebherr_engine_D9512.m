% clear all
% close all
% clc


torque_reference = [300 500 700 833 900 1100 1300 1500 1665 ...
    1700 1900 2100 2300 2500 2700 2900 3100 3300];
rpm_base = [620 700 800 900 1000 1100 1200 1300 1400 1500 1600 1700 1800 ...
    1900 2000 2100];
[m,uit]=size(torque_reference);
[m,uiw]=size(rpm_base);

%% data1
rpm_1_data = [2100 2100 2100 2100 2100 2100];
torque_1_data = [200 400 600 800 1000 1167];
consumption_1_data = [544.9	362.7 304.6	279.2 268.2 262.3];

torque_1_temp = [200 400 600 800 1000 1167 1300 1500 1665 ... 
    1700 1900 2100 2300 2500 2700 2900 3100 3300];
consumption_1_temp = [544.9	362.7	304.6	279.2	268.2 ...
    262.3 268 280 304 324 345 420 540 620 850 1090 1540 2580];
f=fit(torque_1_temp',consumption_1_temp','spline');

consumption_1 = f(torque_reference);

% figure; 
% plot(torque_1_data, consumption_1_data); grid on
% hold on
% plot(torque_reference, consumption_1); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data2
rpm_2_data = [2000 2000 2000 2000 2000 2000 2000 2000 ...
    2000 2000 2000 2000 2000 2000 2000 2000 2000];
torque_2_data = [2700 2500 2300 2100 2024 1900 1700 ...
    1500 1300 1349 1100 900 700 675 500 300 270];
consumption_2_data = [236.9	232.6	233.8	236.7	238.1	240.7	244.7 ...
    246.1	249.2	248.2	254.6	259.6	275.4	278.2	310.5 ...
    398.4	423.0];
torque_2_data = fliplr(torque_2_data);
consumption_2_data = fliplr(consumption_2_data);

f=fit(torque_2_data',consumption_2_data','spline');

consumption_2 = f(torque_reference);

% figure; 
% plot(torque_2_data, consumption_2_data); grid on
% hold on
% plot(torque_reference, consumption_2); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data3
rpm_3_data = [1900	1900	1900	1900	1900	1900	1900	1900 ...
    1900	1900	1900	1900	1900	1900];
torque_3_data = [300	500	700	900	1100	1300	1500	1700	1900 ...
    2100	2300	2500	2700	2835];
consumption_3_data = [369.8 289.5 257.0 240.5 230.9 224.7 220.7 ...
    218.2 217.0 215.5 214.6 214.1 215.4 217.1];
f=fit(torque_3_data',consumption_3_data','spline');
consumption_3 = f(torque_reference);
% 
% figure; 
% plot(torque_3_data, consumption_3_data); grid on
% hold on
% plot(torque_reference, consumption_3); grid on
% hold off
% set(gca,'xlim',[0 3500]);
% 

%% data4
rpm_4_data = [1800 1800 1800 1800 1800 1800 1800 1800 ...
    1800 1800 1800 1800 1800 1800 1800];
torque_4_data = [2976 2900 2700 2500 2300 2100 1900 ...
    1700 1500 1300 1100 900 700 500 300];
consumption_4_data = [208.5 207.4 205.7 206.3 207.4 209.0 210.6 ...
    212.0 214.7 218.9 224.7 233.8 249.1 279.4 354.6];
torque_4_data = fliplr(torque_4_data);
consumption_4_data = fliplr(consumption_4_data);
f=fit(torque_4_data',consumption_4_data','spline');
consumption_4 = f(torque_reference);

% figure; 
% plot(torque_4_data, consumption_4_data); grid on
% hold on
% plot(torque_reference, consumption_4); grid on
% hold off
% set(gca,'xlim',[0 3500]);


%% data5
rpm_5_data = [1700 1700 1700 1700 1700 1700 1700 1700 ... 
    1700 1700 1700 1700 1700 1700 1700 1700 ];
torque_5_data = [300 500 700 900 1100 1300 1500 1700 1900 ...
    2100 2300 2500 2700 2900 3100 3156];
consumption_5_data = [343.8 271.8 242.4 227.4 218.3 213.8 209.8 ...
    207.0 205.2 204.5 204.2 204.1 203.7 204.8 206.6 207.5];
f=fit(torque_5_data',consumption_5_data','spline');
consumption_5 = f(torque_reference);

% figure; 
% plot(torque_5_data, consumption_5_data); grid on
% hold on
% plot(torque_reference, consumption_5); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data6
rpm_6_data = [1600 1600 1600 1600 1600 1600 1600 1600 ...
    1600 1600 1600 1600 1600 1600 1600 1600];
torque_6_data = [3300 3100 2900 2700 2500 2300 2100 1900 ...
    1700 1500 1300 1100 900 700 500 300];
consumption_6_data = [203.4 201.7 202.0 202.2 202.0 201.9 202.3 ...
    203.3 204.8 207.3 210.9 215.4 223.1 237.0 264.0 330.6];
torque_6_data = fliplr(torque_6_data);
consumption_6_data = fliplr(consumption_6_data);
f=fit(torque_6_data',consumption_6_data','spline');
consumption_6 = f(torque_reference);

% figure; 
% plot(torque_6_data, consumption_6_data); grid on
% hold on
% plot(torque_reference, consumption_6); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data7
rpm_7_data = [1500 1500 1500	1500 1500 1500 1500 1500 1500 1500 1500 ...
    1500 1500 1500 1500 1500 1500 1500 1500 1500];
torque_7_data = [300 500 700 833 900 1100 1300 1500 1665 ...
    1700 1900 2100 2300 2498 2500 2700 2900 3100 3300];
consumption_7_data = [323.7 259.3 233.2 224.1 221.1 217.2 ...
    216.1 213.2 211.8 211.7 210.7 209.6 208.7 208.5 ...
    208.0 207.6 206.4 204.8 204.0];
f=fit(torque_7_data',consumption_7_data','spline');
consumption_7 = f(torque_reference);

% figure; 
% plot(torque_7_data, consumption_7_data); grid on
% hold on
% plot(torque_reference, consumption_7); grid on
% hold off
% set(gca,'xlim',[0 3500]);


%% data8
rpm_8_data = [1400 1400 1400 1400 1400 1400 1400 1400 ...
    1400 1400 1400 1400 1400 1400 1400 1400];
torque_8_data = [3300 3100 2900 2700 2500 2300 2100 ...
    1900 1700 1500 1300 1100 900 700 500 300];
consumption_8_data = [191.0 190.9 191.1 191.5 192.1 193.0 194.0 ...
    195.4 197.2 199.6 203.1 207.7 215.4 229.0 254.9 315.4];
torque_8_data = fliplr(torque_8_data);
consumption_8_data = fliplr(consumption_8_data);
f=fit(torque_8_data',consumption_8_data','spline');
consumption_8 = f(torque_reference);

% figure; 
% plot(torque_8_data, consumption_8_data); grid on
% hold on
% plot(torque_reference, consumption_8); grid on
% hold off
% set(gca,'xlim',[0 3500]);


%% data9
rpm_9_data = [1300 1300 1300 1300 1300 1300 1300 1300 1300 1300 1300 1300 ...
    1300 1300];
torque_9_data = [300 500 700 900 1100 1300 1500 1700 1900 2100 2300 2500 ...
    2700 2900];
consumption_9_data = [309.9 251.5 226.8 213.8 206.5 202.0 198.9 ...
    196.8 194.2 193.9 192.9 192.5 191.3 191.8];
f=fit(torque_9_data',consumption_9_data','spline');
consumption_9 = f(torque_reference);

% figure; 
% plot(torque_9_data, consumption_9_data); grid on
% hold on
% plot(torque_reference, consumption_9); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data10
rpm_10_data = [1200 1200 1200 1200 1200 1200 1200 1200 1200 1200 1200 1200];
torque_10_data = [2499 2300 2100 1900 1700 1500 1300 1100 900 700 500 300];
consumption_10_data = [193.4 193.7 194.1 194.6 195.8 197.6 200.3 204.8 ...
    211.6 223.2 245.9 301.0];
torque_10_data = fliplr(torque_10_data);
consumption_10_data = fliplr(consumption_10_data);
f=fit(torque_10_data',consumption_10_data','spline');
consumption_10 = f(torque_reference);

% figure; 
% plot(torque_10_data, consumption_10_data); grid on
% hold on
% plot(torque_reference, consumption_10); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data11
rpm_11_data = [1100 1100 1100 1100 1100 1100 1100 1100 1100 1100 1100];
torque_11_data = [300 500 700 900 1100 1300 1500 1700 1900 2100 2300];
consumption_11_data = [298.5 244.2 222.2 210.7 204.5 200.8 198.8 197.3 196.5 ...
    195.8 195.8];
f=fit(torque_11_data',consumption_11_data','spline');
consumption_11 = f(torque_reference);

% figure; 
% plot(torque_11_data, consumption_11_data); grid on
% hold on
% plot(torque_reference, consumption_11); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data12
rpm_12_data = [1000 1000 1000 1000 1000 1000 1000 1000 1000 1000];
torque_12_data = [2100 1900 1700 1500 1300 1100 900 700 500 300];
consumption_12_data = [198.3 197.6 197.6 198.2 200.0 203.1 209.0 ...
    219.2 239.9 293.1];
torque_12_data = fliplr(torque_12_data);
consumption_12_data = fliplr(consumption_12_data);
f=fit(torque_12_data',consumption_12_data','spline');
consumption_12 = f(torque_reference);

% figure; 
% plot(torque_12_data, consumption_12_data); grid on
% hold on
% plot(torque_reference, consumption_12); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data13
rpm_13_data = [900 900 900 900 900 900 900 900 900 900];
torque_13_data = [300 500 700 900 1100 1300 1500 1700 1900 2100];
consumption_13_data = [285.5 237.4 217.7 208.5 203.7 200.4 199.2 ...
    198.9 200.2 202.6];
torque_13_temp = [300 500 700 900 1100 1300 1500 1700 1900 2100 2300 ...
    2500 2700 2900 3100 3300];
consumption_13_temp = [285.5 237.4 217.7 208.5 203.7 200.4 199.2 ...
    198.9 200.2 202.6 208 212 220 236 248 272];
f=fit(torque_13_temp',consumption_13_temp','spline');
consumption_13 = f(torque_reference);

% figure; 
% plot(torque_13_data, consumption_13_data); grid on
% hold on
% plot(torque_reference, consumption_13); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data14
rpm_14_data = [800 800 800 800 800 800 800 800 800];
torque_14_data = [1900 1700 1500 1300 1100 900 700 500 300];
consumption_14_data = [203.8 200.6 199.2 199.5 201.5 206.1 214.9 233.1 276.6];
torque_14_temp = [3300 3100 2900 2700 2500 2300 2100 1900 1700 1500 1300 ...
    1100 900 700 500 300];
consumption_14_temp = [280 250 225 215 210 208 205 203.8 200.6 199.2 ...
    199.5 201.5 206.1 214.9 233.1 276.6];

torque_14_temp = fliplr(torque_14_temp);
consumption_14_temp = fliplr(consumption_14_temp);
f=fit(torque_14_temp',consumption_14_temp','spline');
consumption_14 = f(torque_reference);

% figure; 
% plot(torque_14_data, consumption_14_data); grid on    
% hold on
% plot(torque_reference, consumption_14); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data15
rpm_15_data = [700 700 700 700 700 700 700];
torque_15_data = [300 500 700 900 1100 1300 1500];
consumption_15_data = [272.4 231.7 214.7 206.8 203.3 201.5 201.7];
f=fit(torque_15_data',consumption_15_data','spline');
consumption_15 = f(torque_reference);

% figure; 
% plot(torque_15_data, consumption_15_data); grid on    
% hold on
% plot(torque_reference, consumption_15); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% data16
rpm_16_data = [620 620 620 620 620 620];
torque_16_data = [1300 1100 900 700 500 300];
consumption_16_data = [202.5 203.1 206.9 215.1 231.5 269.1];
torque_16_temp = [3300 3100 2900 2700 2500 2300 2100 1900 1700 1500 1300 1100 900 700 500 300];
consumption_16_temp = [665 520 420 350 290 250 236 222  215 208 202.5 203.1 206.9 215.1 231.5 269.1];
torque_16_temp = fliplr(torque_16_temp);
consumption_16_temp = fliplr(consumption_16_temp);
f=fit(torque_16_temp',consumption_16_temp','spline');
consumption_16 = f(torque_reference);

% figure; 
% plot(torque_16_data, consumption_16_data); grid on    
% hold on
% plot(torque_reference, consumption_16); grid on
% hold off
% set(gca,'xlim',[0 3500]);

%% torque and power curve

% torque_friction_base = -(rpm_base./60*2*pi)*0.5-(rpm_base./60*2*pi)*5e-3; %torque friction at shaft
torque_friction_base = -(rpm_base./60*2*pi)*0.25-(rpm_base./60*2*pi).^2*5.2e-4; %torque friction at shaft

torque_base = [max(torque_16_data) max(torque_15_data) max(torque_14_data) ...
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
% set(gca,'xlim',[620 rpm_base(end)]); 
% hchild = get(gca,'Children');
% set(gca,'Children',[hchild(3) hchild(2) hchild(1)])
% ylabel('$\tau/Nm$','Interpreter','latex')
% xlabel('$n/rpm$','Interpreter','latex')
% title('\textbf{Liebherr D9512 Engine - Torque and friction curves}','Interpreter','latex')
% grid on
% print -dpdf engine_D9512_torque
% print -depsc engine_D9512_torque
% 
% figure; 
% plot(rpm_base, torque_e.*rpm_base/60*2*pi*1e-3,'k-','LineWidth',1.8);
% hold on
% plot(rpm_base, torque_base.*rpm_base/60*2*pi*1e-3,'-','LineWidth',1.8,'Color',[0.5 0.5 0.5]);
% hold off
% legend('$P_e$','$P_{d}$','Interpreter','latex','Location','best');
% set(gca,'xlim',[620 rpm_base(end)]); 
% ylabel('$P/kW$','Interpreter','latex')
% xlabel('$n/rpm$','Interpreter','latex')
% title('\textbf{Liebherr D9512 Engine - Power curves}','Interpreter','latex')
% grid on
% print -dpdf engine_D9512_power
% print -depsc engine_D9512_power
% % 

%% consumption 3D map
rpm_consumption = rpm_base; % input speed for engine consumption
torque_consumption = torque_reference;
consumption = [consumption_1, consumption_2, consumption_3, consumption_4, ...
        consumption_5, consumption_6, consumption_7, consumption_8, ...
        consumption_9, consumption_10, consumption_11, consumption_12, ...
        consumption_13, consumption_14, consumption_15, consumption_16 ...
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
% title('\textbf{Liebherr D9512 Engine - Consumption $\mathbf{[g/kWh]}$}','Interpreter','latex')
% print -dpdf engine_D9512_consumption
% print -depsc engine_D9512_consumption





