%[text] ## General Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

rpi_enable = 0; % use RPI otherwise DQ PI
system_identification_enable = 0;
%[text] ### Voltage application
application400 = 1;
application690 = 0;
application480 = 0;
%[text] ### PWM and sampling time and data length storage
fPWM = 10e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

double_sampling = 0;

if double_sampling 
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_dab = 1/fPWM_DAB/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_dab = 1/fPWM_DAB;
end

ts_battery = ts_dab;
tc = ts_inv/100;

z_dab = tf('z',ts_dab);
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);

t_misura = simlength - 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

dead_time_INV = 1e-6;
delay_pwm = 0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ## Grid Voltage Rectifier
%[text] ### Nominal DClink voltage seting
if (application690 == 1)
    Vdc_nom = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_nom = 750; % DClink voltage reference
else
    Vdc_nom = 660; % DClink voltage reference
end
%[text] ### 
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = Vdc_nom; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 1400e-6*4;
RCFi_dc1_internal = 1e-3;
%[text] ### Load Transformer Parameters
m1_load_trafo = 50;
m2_load_trafo = 1;
m12_load_trafo = m1_load_trafo/m2_load_trafo;

ftr_nom_load_trafo = 50;
I0rms_load_trafo = 5;
V1rms_load_trafo = 330;
I1rms_load_trafo = 600;
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:4e4d9ab0]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:617b30e4]
lm_load_trafo = V1rms_load_trafo/I0rms_load_trafo/2/pi/ftr_nom_load_trafo;
rfe_load_trafo = 2e3;
rd1_load_trafo = 1e-3;
ld1_load_trafo = 400e-6;
rd2_load_trafo = rd1_load_trafo/m12_load_trafo^2;
ld2_load_trafo = ld1_load_trafo/m12_load_trafo^2;
%[text] ### Current sensor endscale, and quantization
Pnom = 200e3;
margin_factor = 1.25;
adc12_quantization = 1/2^12;
adc11_quantization = 1/2^11;
adc16_quantization = 1/2^16;
adc15_quantization = 1/2^15;
Vdc_FS = Vdc_nom * margin_factor;
Idc_FS = Pnom/Vdc_nom * margin_factor;
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:45db0fb3]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:19be7e26]
%[text] ### Single phase inverter control
iph_grid_pu_ref = 1;
%[text] ### DClink Lstray model
Lstray_dclink = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_dclink + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi_dc1;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ### INV current control parameters
%[text] #### DQ PI
kp_inv = 0.25;
ki_inv = 45;
%[text] #### Phase shift filter for Q component derivation at 50Hz and 80Hz
if system_identification_enable
    freq = 300;
else
    freq = 80;
end
omega_set = 2*pi*freq;

flt_dq = 2/(s/omega_set+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
figure; bode(flt_dq_d, options); grid on %[output:700f1fc7]
[num, den]=tfdata(flt_dq_d,'v') %[output:97e357dd] %[output:9d1224cc]

%[text] #### Resonant PI
kp_rpi = 2;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:26f707e8]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:2214b3d6]
a11d = 1 %[output:6e27c0a7]
a12d = ts_inv %[output:7996ad92]
a21d = -omega_set^2*ts_inv %[output:9ad8d2a8]
a22d = 1 -2*delta*omega_set*ts_inv %[output:637faca4]

Bres = [0; 1];
Cres = [0 1];
Bresd = Bres*ts_inv;
Cresd = Cres;
%%
%[text] ### Grid Normalization Factors
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;
Igrid_phase_normalization_factor = 250e3/Vphase2/3/0.9*sqrt(2);
ixi_pos_ref_lim = 1.6;
ieta_pos_ref_lim = 1.0;
ieta_neg_ref_lim = 0.5;
%%
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:61f9eea3]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:306c88de]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:23fb1bfd]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:11c23dfc]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:4b099645]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;

kp_rc_pos_grid = 0.35;
ki_rc_pos_grid = 35;
kp_rc_neg_grid = 0.35;
ki_rc_neg_grid = 35;
%%
%[text] ## Settings for user functions: filters, moving average, rms
%[text] ### Low Pass Filters
%[text] #### LPF 50Hz in state space (for initialization)
fcut = 50;
fof = 1/(s/(2*pi*fcut)+1);
[nfof, dfof] = tfdata(fof,'v');
[nfofd, dfofd]=tfdata(c2d(fof,ts_afe),'v');
fof_z = tf(nfofd,dfofd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(nfofd,dfofd);
LVRT_flt_ss = ss(A,B,C,D,ts_afe);
[A,B,C,D] = tf2ss(nfof,dfof);
LVRT_flt_ss_c = ss(A,B,C,D);
%[text] #### LPF 161Hz
fcut_161Hz_flt = 161;
g0_161Hz = fcut_161Hz_flt * ts_afe * 2*pi;
g1_161Hz = 1 - g0_161Hz;
%%
%[text] #### LPF 500Hz
fcut_500Hz_flt = 500;
g0_500Hz = fcut_500Hz_flt * ts_afe * 2*pi;
g1_500Hz = 1 - g0_500Hz;
%%
%[text] #### LPF 75Hz
fcut_75Hz_flt = 75;
g0_75Hz = fcut_75Hz_flt * ts_afe * 2*pi;
g1_75Hz = 1 - g0_75Hz;
%%
%[text] #### LPF 50Hz
fcut_50Hz_flt = 50;
g0_50Hz = fcut_50Hz_flt * ts_afe * 2*pi;
g1_50Hz = 1 - g0_50Hz;
%%
%[text] #### LPF 10Hz
fcut_10Hz_flt = 10;
g0_10Hz = fcut_10Hz_flt * ts_afe * 2*pi;
g1_10Hz = 1 - g0_10Hz;
%%
%[text] #### LPF 4Hz
fcut_4Hz_flt = 4;
g0_4Hz = fcut_4Hz_flt * ts_afe * 2*pi;
g1_4Hz = 1 - g0_4Hz;
%%
%[text] #### LPF 1Hz
fcut_1Hz_flt = 1;
g0_1Hz = fcut_1Hz_flt * ts_afe * 2*pi;
g1_1Hz = 1 - g0_1Hz;
%%
%[text] #### LPF 0.2Hz
fcut_0Hz2_flt = 0.2;
g0_0Hz2 = fcut_0Hz2_flt * ts_afe * 2*pi;
g1_0Hz2 = 1 - g0_0Hz2;
%[text] #### 
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = 2*pi*f_grid;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ### Single phase pll
freq_pll = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:00aa870f]

freq_filter = freq_pll;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Lithium Ion Battery
ubattery = 600;
Pbattery_nom = 750e3;
typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Ibattery_nom = Pbattery_nom/Vbattery_nom;
Rmax = Vbattery_nom^2/(Pbattery_nom*0.1);
Rmin = Vbattery_nom^2/(Pbattery_nom);

E_1 = -1.031;
E0 = 3.485;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.0035;
R1 = 0.0035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:2e5b507a]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:2e5b507a]
xlabel('state of charge [p.u.]'); %[output:2e5b507a]
ylabel('open circuit voltage [V]'); %[output:2e5b507a]
title('open circuit voltage(state of charge)'); %[output:2e5b507a]
grid on %[output:2e5b507a]

%[text] ## System Identification
%[text] ### Normalization
Ibez = I1rms_load_trafo*sqrt(2);
Ubez = V1rms_load_trafo*sqrt(2);
Xbez = Ubez/Ibez;
%[text] ## IGBT, MOSFET, DIODE and snubber data
%[text] ### Diode rectifier
Vf_diode_rectifier = 0.7;
Rdon_diode_rectifier = 1.85e-3;
%[text] ### HeatSink
weigth = 0.050; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K %[output:6f3c261a]
thermal_conducibility = 204; % W/(m K)
Rth_mosfet_HA = 30/1000 % K/W %[output:7d5e9062]
Tambient = 40;
DThs_init = 0;
%[text] ### SKM1700MB20R4S2I4
Rds_on = 1.04e-3; % Rds [Ohm]
Vdon_diode = 4; % V
Vgamma = 4; % V
Rdon_diode = 1.85e-3; % V
Eon = 77e-3; % J @ Tj = 125°C
Eoff = 108e-3; % J @ Tj = 125°C
Eerr = 9.7e-3; % J @ Tj = 125°C
Voff_sw_losses = 1300; % V
Ion_sw_losses = 1000; % A

JunctionTermalMass = 2; % J/K
Rtim = 0.05;
Rth_mosfet_JC = 19/1000; % K/W
Rth_mosfet_CH = 6/1000; % K/W
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W %[output:71a97270]
Lstray_module = 12e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdc_nom^2 %[output:5cd9d49f]
Rsnubber = 1/(Csnubber*fPWM_INV)/5 %[output:60ed39a0]
%[text] ### Load
uload = 3;
rload = uload/I2rms_load_trafo;
%[text] ### FF1000UXTR23T2M1

% Ron = 2.13e-3; % Rds [Ohm]
% Vgamma = 5.5; % V
% Eon = 540e-3; % J @ Tj = 125°C
% Eoff = 370e-3; % J @ Tj = 125°C
% Eerr = 0.075e-3; % J @ Tj = 125°C
% Voff_sw_losses = 1500; % V
% Ion_sw_losses = 2000; % A
% 
% JunctionTermalMass = 0.025; % J/K
% Rtim = 0.05;
% Rth_mosfet_JC = 20/1000; % K/W
% Rth_mosfet_CH = 5.8/1000; % K/W
% Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W
%[text] ## C-Caller Settings
model = 'single_phase_inverter';
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});

%[text] ## Remove Scopes Opening Automatically
% open_scopes = find_system(model, 'BlockType', 'Scope');
% for i = 1:length(open_scopes)
%     set_param(open_scopes{i}, 'Open', 'off');
% end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":51.6}
%---
%[output:4e4d9ab0]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"     6.600000000000000e+00"}}
%---
%[output:617b30e4]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:45db0fb3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:19be7e26]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:700f1fc7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGlCAYAAAAYiyWNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tvQ9wVtd993nc0W5oK7LYm04EwoEYtRO\/TKp6GkMUe5i4vIPZNO0ydoOxtiuywNREUe1alZNdhnESD2HGSFajiaKSVjCG3QGMxx7mJWFt3jDdKDiJcDOu\/OZd2kY0JFFkGPKHBAhmX7+vdn5XPk\/uc3n+3HOfc+859z6fO+OxpOfc3zn38\/1dzldHv3vuLXNzc3OKAwIQgAAEIAABCEAAAgUlcAuGt6DKclkQgAAEIAABCEAAAgEBDC+JAAEIQAACEIAABCBQaAIY3kLLy8VBAAIQgAAEIAABCGB4yQEIQAACEIAABCAAgUITwPAWWl4uDgIQgAAEIAABCEAAw0sOQAACEIAABCAAAQgUmgCGt9DycnEQgAAEIAABCEAAAhhecgACEIAABCAAAQhAoNAEMLyFlpeLgwAEIAABCEAAAhDA8JIDEIAABCAAAQhAAAKFJoDhLbS8XBwEIAABCEAAAhCAAIaXHIAABCAAAQhAAAIQKDQBDG+h5eXiIAABCEAAAhCAAAQwvOQABCAAAQhAAAIQgEChCWB4Cy0vFwcBCNQiMDY2poaGhsqaLFy4UB08eFB1dnYaw7tw4YLauHGj2rlzp1q3bp3R+f39\/erYsWM1x3Ly5Em1a9cudfToUdXW1mYUn8YQgAAEmpkAhreZ1efaIdDkBMTwTkxMqPHxcdXa2hrQkJ8dOXIkkals1PBK\/8PDwyVVZCxf\/vKXSwYcw9vkCcvlQwACiQlgeBOj40QIQCDvBCoZ3qmpKdXX16dGR0eNV3ltG17hKyu\/USOcd+6MHwIQgEDWBDC8WROnPwhAwBsC1VZ4o6u+4XKDDRs2lK3Cyqrr9u3bg2tatWqVmp2dLZU0aAM8MzMTfL53796qpQ7VjG14jN\/85jfLShrCfUv8gYEB1dvbG\/R19epVtW3bNnXmzBklZRoytitXrgSr2RJnZGQkaCdjkxIOOXp6eoI2coSvU8YmP5f\/JN7SpUvVF7\/4RfWXf\/mXwfnyPWUW3qQ1A4EABCoQwPCSFhCAQNMSqFTDGzVvYcMpJlJqdDdt2hQYy+iKro4nxvZDH\/pQYDjXrFkTtK23clzN8IbLGF5\/\/fWS4RXRtm7dqnbv3h2sREfLHSqNe8mSJSXDKyZdG3Btjrds2RIY8uhYZWynTp0KjPGKFSuC6xJjLyZXjjCTpk0mLhwCEPCaAIbXa3kYHAQgkCaBSiu8YhyfeOKJMnOnTauMpdaKa9gAv\/vd71Y7duxQ+\/btKz1gJsaxo6OjtAobvjZTwxt9aC1ceyz1yGGzXW\/cUcZhBmKmo2MLf6\/NcphRmpoRGwIQgEASAhjeJNQ4BwIQKASBSoY3bOAeeOCBm3ZdCJ8jK57h8ofwSqkA0qUOYVjRkgj9mWlJgza1UmIgx\/ve976gjCG86hreLaKWUZfzw2Ub7e3tQUxdx4zhLUS6cxEQaGoCGN6mlp+Lh0BzE6hneKWm1WSltN4Kby3alQyvNtBSiiC7N1Qrb5DV3vBnpiu80RKGSiUNMna9gwQrvM1933D1EMgjAQxvHlVjzBCAgBUC9Uoa5M\/5cWp4dU2vfoisUg2vNsO6bfQCKhneWtuSST1vtPRC19WKAa5Xwxvez1cMrpj7wcHBoIY3XLNLSYOVVCMIBCDgmACG17EAdA8BCLgjUOmhNRlNdDeFWrs0aLMoOxjceeedwcU89thjgXGM7tJQrZxBzqn04onoA3SVVnH1Lgx\/\/dd\/rf7+7\/++VIYQ3qVB4tx3333qX\/7lX0oPrUVfYBHu\/7Of\/WzQTpdEUNLgLkfpGQIQsEMAw2uHI1EgAAEIeE1AzP309HTZlmpeD5jBQQACELBIAMNrESahIAABCPhCILwjRL1yCl\/GzDggAAEIpEUAw5sWWeJCAAIQcEggXGohw6hVTuFwmHQNAQhAIBMCGN5MMNMJBCAAAQhAAAIQgIArAhheV+TpFwIQgAAEIAABCEAgEwIY3kww0wkEIAABCEAAAhCAgCsCGF5X5OkXAhCAAAQgAAEIQCATAhjeTDDTCQQgAAEIQAACEICAKwIYXlfk6RcCEIAABCAAAQhAIBMCGN5MMNMJBCAAAQhAAAIQgIArAhheV+TpFwIQgAAEIAABCEAgEwIY3kww0wkEIAABCEAAAhCAgCsCGF5X5OkXAhCAAAQgAAEIQCATAhjeTDDTCQQgAAEIQAACEICAKwIYXlfk6RcCEIAABCAAAQhAIBMCGN5MMNMJBCAAAQhAAAIQgIArAhheV+TpFwIQgAAEIAABCEAgEwIY3kww0wkEIAABCEAAAhCAgCsCGF5X5OkXAhCAAAQgAAEIQCATAhjeTDDTCQQgAAEIQAACEICAKwIYXlfk6RcCEIAABCAAAQhAIBMCGN5MMNMJBCAAAQhAAAIQgIArAhheV+TpFwIQgAAEIAABCEAgEwIY3kww0wkEIAABCEAAAhCAgCsCGF5X5OkXAhCAAAQgAAEIQCATAhjeTDDTCQQgAAEIQAACEICAKwIYXlfk6RcCEIAABCAAAQhAIBMCGN5MMNMJBCAAAQhAAAIQgIArAhjeDMnPzMxk2BtdQQACEIAABCDQzASWLl3azJdfdu0Y3oxSQczuE088oSYnJzPqkW4gAAEIQAAC7gh0dXWpBQsWqAsXLqizZ8+6G0gT97x69Wo1ODioML5KYXgzuhG+\/e1vq+7u7iDx2tvbM+q1ejdivEdGRqyOp5GYpufGaV+vTa3PTT+r11fWgqc1nkbimp4bp329NqY6ik7VzqnXVxE0buQaTc+N275eO1sa1+sna31r5WIjY2nkOk3P\/da3vqW++93vqg9+8IPq\/e9\/f8Vh14uZRN9q7Or11QjXJOemNR4d98EHH1QvvPCCmpiYwPAqDG+SHE10jja8viSerDjLjfDYY48lup5KJzUS0\/TcOO3rtan1uelnzaCvaF6Paa1kMj03Tvt6bUx1rHWNzaBxPZ5Z6xsn52xp7Ju+ca49yT\/eWWp85MgRNT09rdauXatktTfJvJFE32rsfNO4ES3i3IuyuisLbb74jiT5avMcVnht0qwRy7cbLaPLbppu0Lf4UqNxsTVGX\/v6Hjt2TF27dk3dcccdVQ2v\/V6rR2w2jZvteuvlEoa3HiFLn0viPfDUc+qxxx61FJEwWRF4z22\/Wbcr+U1dSkRkxbxWrdTtty6oG0s3eM9ttdvW+zx2RzSMRYDJIxam3DZCX\/vSYXjtMzWJSE6X08LwmmRPA20l8T767I+oo2mAoYtTf\/izN11021CflYxw1GhH29z8\/bzJD5+n2zSr0T5\/\/rx69tln1c6dO1VLS0tDGnGyfwTQ174mvhneZtMYw4vhtX9Xx4hI4sWAlOMmb775pnrjjTfU7bffXtMMmRjoem1\/9PPKZvyHP7t+E8lorOj34Vj1+g0H1+ZXG2P5PmyIZXVcf3Zvx6IcK6xUXI1zfZFNPHj0tS++b4a32TTGd2B47d\/VMSKSeDEg5bhJUf8hDZtf\/bU2x2FjLZ+F20qbasY5bJLDBlmb46hp9iUtiqqxL3xdjwN97SuA4bXP1CQivgPDa5IvsduOjY2poaGhoP3AwIDq7e0tO5fEi40ylw2ZLKvLpo2v\/H\/eCM+vQIdN8ivnLt8UQBvje1YsKq0a37Pi1uBrF2UVaJzLWzP2oNE3NqrYDTG8sVGl0hDfgeG1nlhTU1Oqr69PjY6OBrH1152dnaW+SDzr2L0KyGTZuBzVjPHp6ctvG+XyEg4xvVIuoUslxAynWTaBxo1r7HME9LWvDobXPlOTiPgODK9JvsRqK6u7ss\/d+Pi4am1tVf39\/aqjo6NslZfEi4Uyt42YLLORTq8Kv3Lu50GHlcxwWkYYjbPR2FUv6GufPIbXPlOTiPgODK9JvsRqKwZXjuHh4eD\/0e\/lZzrxDh06VLZTQ1tbW6w+aOQ3ASZL9\/qIGRYjrE2x\/H\/yB1dLAxMj\/PDd8\/ebrAZ\/cHmr0aDR2AhX7hqjr33Jjh8\/HuzDu2zZMi\/24S26xvIK5\/Ah22Xy4olfE2FbMgv3eHRFV1Z85e0y2gCHDW+0u56eHrV582YLoyCESwI3btxQly5dUvILDFtWuVTi5r5nf\/mWOn523vh+58dvBv\/pY8k7W9RH3zdvfB9ZXXsXCTT2S1fbo0Ff20SVOn36dLC7yeLFi9XKlSvtd2AYsegaHzhwQB08ePAmKrxpbR4JhtfwhqnU3MTwDg4Oqvb29lIYMUis8loQwXGI69evq4sXLwar9xhex2LE6F5M8POvXQpaSlmEfmhODPDihS1BLbA8LHdvx62laGgcA2yOm6CvffFOnDihrl69qpYvX65WrVplvwPDiEXXWFZ4w6u8k5OTwQuRMLwYXsNbpXpzk5IGEs8adq8CFf1PZV7BTmEw4drgsAGWrnQpxOplC9Wyd1ypu9dyCsMjZAYEuIftQ6aG1z5Tk4jU8JbTYoXXJHuqtI2WMPDQmgWoOQvBZJkzwWIMV0zw4VffuGkF+L2\/0xqsAD9892In26PFGDpNEhDgHk4Arc4pGF77TE0iYngxvCb5Eqst25LFwlToRkyWhZY3uLh\/nb2s9k98X\/3nn6pSCYRe\/ZWXZugH4opPophXyD1sX1cMr32mJhExvBhek3yJ3ZYXT8RGVciGTJaFlLXsoqIa6xXgw69eKL1VThtgVn\/zlw\/cw\/Y1w\/DaZ2oSEcOL4TXJF2ttSTxrKL0MxGTppSxWB1VL40rlD9r8pv1CDKsX2cTBuIfti4\/htc\/UJCK+A8Nrki\/W2pJ41lB6GYjJ0ktZrA7KROP5B99+rp5++XwwBkofrEqRSjATfVMZQAGDYnjdiorvwPA6yUASzwn2zDplsswMtbOOkmocLX0Q8ytbnsn\/P33\/e51dDx2XE0iqLxyrE8Dwus0OfAeG10kGknhOsGfWKZNlZqiddWRD40qlD\/P7\/S7C\/DpTdr5jG\/o6vgTvusfwupUE34HhdZKBJJ4T7Jl1ymSZGWpnHdnWuJr57V61mB0fHKhsW18Hl+Bdlxhet5LgOzC8TjKQxHOCPbNOmSwzQ+2sozQ1jppfan6zlzlNfbO\/Gj96xPC61QHfgeF1koEknhPsmXXKZJkZamcdZaVxpZpf2eOX3R7SlT4rfdO9Cr+iY3jd6oHvwPA6yUASzwn2zDplsswMtbOOXGhczfyyz6\/9NHChr\/2r8CsihtetHvgODK+TDCTxnGDPrFMmy8xQO+vItcba\/Ea3OmOnBzsp4VpfO1fhVxQMr1s98B0FNLxXr15V27ZtU2fOnLkpu1atWqXGx8dVa2ur08wj8ZziT71zJsvUETvvwCeNZZ\/fw6++oeQtb3LITg887NZYivikb2NX4s\/ZGF63WuA7CmR4p6amVE9PT3BFBw8eVJ2dnTdl18mTJ9X27dvVwoULq7bJIiVJvCwou+uDydId+6x69lFjSh7sqe+jvvauzk0kDK8b7rpXfEdBDO+FCxfUnj171FNPPRVr9VZWgZ988kk1PDzsJANJPCfYM+uUyTIz1M468l1jSh4aSw3f9W3s6tycjeF1wx3DW5n7LXNzc3NuJWmO3jG8xdaZybLY+srV5UljMb9Pv\/z9UsnDp+9frt5z22+yv2+NNM2Tvnm52zC8bpXCdxRkhVcuI1q7u3fv3uDqpIRBDl\/qd2UsJJ7bGz\/t3pks0ybsPn4eNa5W8sCDbjfnUx71dX9X1B4BhtetQviOAhne\/v7+4GqkTEHX6m7YsCH4XpvhJUuWOCtjCKMm8dze+Gn3zmSZNmH38fOucaWSBzG+sscvR75W8POiF4bXrVL4joIYXqnh3bp1q9q9e3fwsJo2uGvWrFG9vb3BVcpDbTt27FD79u1TbW1u\/1En8dze+Gn3nnczlDafIsQvisY86FY5G4uir0\/3GobXrRr4joIb3i1btqh169ZZN7x6R4grV64EsfVKssY5NjamhoaGgm8HBgZKplt\/TuK5vfHT7p3JMm3C7uMXUWNWfX+dV0XU1\/Vdg+F1qwC+A8NrnIGymrxx40a1c+fOwEzr7zdt2hQYWzHDfX19anR0NIitvw5vk0biGWPP1QlMlrmSK9Fgi66xPOQm+\/u+cu6yes9tC4JSh2Z6o1vR9U2U9A2ehOFtEGCDp+M7MLwNptD86eH6YVndnZiYKL3gQj7r6OgoW+Ul8axg9zYIk6W30lgbWLNo3Kyrvs2ir7UbIkYgDG8MSCk2wXcUyPDKquvMzEzNdFm6dKk6evSo9RresOENfx01w3pwOvEOHTqkZEz6cF1bnOK91lShmSyLL3czaqxXfSd\/cLW06vvXa28vpNjNqG\/aQh4\/flxdu3ZNLVu2THV1daXdXd34RddY\/vocPsQfdXd3BwtyYd9RF1RBG7APbwJhdT3v4OBgUOIQXdGVFd\/p6emy3SG04Y12J2+K27x5c4JRcIpPBG7cuKEuXboU\/GLV0tLi09AYiyUCzazx7C\/fUsfPXlV\/d+ZyQHPJO1vUX6xapP7kTrevbLckbRCmmfW1yTEc6\/Tp08H+1YsXL1YrV65Mq5vYcYuu8YEDB4I3ykYPDO88EQxv7FtlvqGu3\/3ABz5QMrQmhldMcnt7e6lXMUis8hqK4GHz69evq4sXLwa\/RWN4PRTIwpDQeB7iyNffCF5oIaUPYnw\/dte7VBH29UVfCzdJJMSJEyeCHZSWL18e7Ivv+ii6xuJPwqu8k5OTamRkhBXetxMvt4ZXG880ShrEwErtkRzhl1dUMrvSxqSkgd+0XP+Tl07\/Rf9TWTrU8hUVjcv1itb6ytvc8vyQG\/ravx+p4bXP1CQiNbzltHJreKOiV3pQLPowmUmiRNtGd2YIfx4tYeChtUZI5\/NcJst86mYyajSuTKsob3NDX5O7IV5bDG88Tmm1wvAW0PBGX0KhL9HWiyfqvbWNbcnSul3zE5fJMj9aJR0pGtcnF171zdvWZuhbX1\/TFhheU2J222N4C2h45ZJ0GcLevXuDB8mirxpuJI2iL53QscLlDrx4ohHC+T+XyTL\/Gta7AjSuR+jXn1fa2kz29fW51hd94+sbtyWGNy6pdNpheAtqeOWywsZ04cKFwdOK4Zc\/pJNS8aKSePE45bUVk2VelYs\/bjSOzyrcUl5mcfjV+Qfd9Kqvj8YXfZPpW+ssDK99piYR8R0FNrwmiZB1WxIva+LZ9sdkmS1vF72hcWPUo6u++k1u93YsaiywpbPR1xLIUBgMr32mJhHxHRhek3yx1pbEs4bSy0BMll7KYnVQaGwPp7zQQm9t5suqL\/ra01dHwvDaZ2oSEd9REMMrD6rt2bNHPfXUU6q1tf7m5\/Lg2ZNPPln2MgiTxGm0LYnXKEG\/z2ey9FsfG6NDYxsUy2P49JAb+trXF8Nrn6lJRHxHQQyvXIau2ZWvq9Xr6ofXXNf0kngmt2n+2jJZ5k8z0xGjsSmx+O0rPeQmdb5S9pDVgb72SWN47TM1iYjvKJDh1Zeitw07c+bMTbkQ3knBJFFstyXxbBP1Kx6TpV96pDEaNE6D6s0xXZU7oK99fTG89pmaRMR3FNDwmiSAq7Yknivy2fTLZJkNZ5e9oHG29LN+kxv62tcXw2ufqUlEfAeG1yRfrLUl8ayh9DIQk6WXslgdFBpbxRk7WFZvckPf2JLEbojhjY0qlYb4DgxvKolVLyiJV49Qvj9nssy3fnFGj8ZxKKXbRsyvLnmwvbsD+trXDsNrn6lJRHwHhtckX6y1JfGsofQyEJOll7JYHRQaW8XZULA0yh3QtyFJKp6M4bXP1CQivgPDa5Iv1tqSeNZQehmIydJLWawOCo2t4rQSzGa5A\/pakaQsCIbXPlOTiPgODK9JvlhrS+JZQ+llICZLL2WxOig0torTerBGyx3Q17okCsNrn6lJRHxHQQ1veGsy2Yrs0UcfVZ\/\/\/OfVvn37VFtbdns5VktGEs\/kNs1fWybL\/GlmOmI0NiXmpn3Scgf0ta8Xhtc+U5OI+I4CGl5tdtesWaM6OjrU\/v371fj4ePAyiomJieDrOG9jM0kk07YknimxfLVnssyXXklGi8ZJqLk7x7TcAX3ta4Xhtc\/UJCK+o4CGV14zvHXrVrV792518eLFkuE9d+6c2rFjhxervCSeyW2av7ZMlvnTzHTEaGxKzJ\/2ccod0Ne+Xhhe+0xNIuI7Cmh45ZL6+\/vV7Oyseuihh9Rzzz0XlDR84hOfUGvXrlXDw8MmOZJKWxIvFazeBGWy9EaK1AaCxqmhzSxwrXIH9LUvA4bXPlOTiPiOghpeuayTJ0+q7du3l65wYGBA9fb2muRHrLbSz65du9TRo0dL9cFjY2NqaGgoOL9SvyReLLS5bcRkmVvpYg8cjWOj8r5hpXKHj931LtX971rU7bffrlpaWry\/hjwMEMPrViV8R4ENbxapJeUTGzduDLrShndqakr19fWp0dHR4Of6687OztKQSLws1HHXB2bIHfusekbjrEhn20+43GHJO1vU\/9q1VH36\/vdmO4iC9obhdSssvgPD21AGykruV77yFSUPymnDKz8LPxwn5RXy8Fx4dZnEawi79ydjhryXqOEBonHDCL0O8K+zl9X+ie+rvztzORjnp+9frh6+e7GSN7pxJCOA4U3GzdZZ+I6CGF690jozM1MzN5YuXVpWetBIIslKrqzifuQjH1F\/8zd\/U4orBlcOXSsc\/V4+04l36NAhJWPShw9bpjXChHPnCWCGip8JaFxsjbW+c7\/9O+r51y6p51\/7iZLVXzG8suIrJQ8cZgSOHz+url27ppYtW6a6urrMTk6hddHvYfFF4UP8UXd3d7AgF\/YdKaDNRchb5ubm5nIx0jqDrLSqGl15bfQ6pY\/169cHYcI1vNG+pd\/p6emyh+W04Y2OoaenR23evLnRoXG+YwI3btxQly5dCmq6qf9zLEZK3aNxSmA9CVtJ3+\/8+E11\/OzV4D8pd\/jo+1rVI6sXeTJi\/4dx+vTpYDFg8eLFauXKlc4HXPR7+MCBA8F2rNEDwztPpBCGN7wtWbhuVlZkbW1LJg+qvfTSS4GJjT60ZmJ4BwcHVXt7eykfxSCxyuv838GGB3D9+vVgSzz5LRrD2zBOLwOgsZeyWBtULX1nf\/lWsOr79MvnA+Mrq72UO9RHf+LEiaD8b\/ny5UpeCOX6KPo9LF4ovMo7OTmpRkZGWOF9O\/EKYXjlWsR0Sr3Q3r171bp160o7NmzYsMF4WzIdS+LKTSrlC08++WTwMJoY6kqGV9rGKWngNy3X\/+Sl03\/R\/1SWDrV8RUXjfOllOto4+ka3NbtnxSLVvWqxevhu92\/zNL3eLNpTw5sF5ep9UMNbzqYwhlcuS1Z0pUTgypUrauHChcHSfnjFN2nqheOGY+g+XnnllbISBh5aS0o6v+fFmSzze3WMXAigcbHzwFTfp1\/+vjr86oVSna+YXnZ3KM8RDK\/bewbDW2DDm1VqRVd42ZYsK\/L+9mM6Wfp7JYysGgE0LnZuJNU3zlvcik2u+tVheN0qj+HF8Dacgbx4omGEhQuQdLIsHIgCXxAaF1hcCyv4td7iVmxyGF5f9cXwYnid5CaJ5wR7Zp1ihjJD7awjNHaGPpOObekbNb56W7NmrPNlhTeT1K3aCb6jgIa31p68NvfhbSR1SbxG6Pl\/rq3J0v8rbd4RonGxtU9D32av88Xwur1n8B0FNLzVUkrvmyu7Nrg+SDzXCqTbfxqTZbojJropATQ2JZav9mnq26x1vhhet\/cAvqOJDK\/NfXgbTVsSr1GCfp+f5mTp95U3z+jQuNhaZ6Fvs9X5Ynjd3jP4jiYyvPLGsyNHjlh7tXAjqUviNULP\/3OzmCz9p1DsEaIx+toiUGk\/3y89fGfwGuMiHRhet2riOwpoeGvV8OoXUbhNO6VIPNcKpNs\/Zihdvj5ER2MfVEhvDK70jdb5yl6+RXnADcObXr7GiYzvKKDhjSO86zYknmsF0u3f1WSZ7lURPUwAjYudD671lVXfTx4+q145dzlY6S3CiywwvG7vGXxHAQ2vrPBu3bpV7d69u+zNatTwur3Zmql315NlM7F2da1o7Ip8Nv36om+RHnDD8GaTu9V6wfAWyPBWe+Vv+BJXrVqlxsfHVWtrq9PMI\/Gc4k+9c18my9QvtIk7QONii++bvuE637yu+GJ43d4z+I4CGV59KdVWeN2mWnnvJJ5Patgfi2+Tpf0rJCIaFzsHfNW30s4OUuebhwPD61YlfEcBDa\/blIrXO4kXj1NeW\/k6WeaVp4\/jRmMfVbE3Jt\/1zeOWZhhee\/mZJBK+oyCGV+\/M8Itf\/EINDg6qXbt2qZmZmZtygjetJblNOMeUgO+Tpen10P5mAmhc7KzIk76ys8PTL58PBPn0\/cvVw3cv9nJLMwyv23sGw1sQw+s2jcx7J\/HMmeXpjDxNlnni6tNY0dgnNeyPJY\/6+m58Mbz289QkIr4Dw2uSL9baknjWUHoZKI+TpZcgPR4UGnssjoWh5Vnf8F6+96xYpHx5iQWG10JiNhAC34HhbSB9kp9K4iVnl4cz8zxZ5oGvD2NEYx9USG8MRdA3bHyl1MH1w20Y3vTyNU5kfEdBDe\/JkyfV9u3bb8oBmzW88qrioaGhoI\/odmfhzwYGBlRvb2\/ZWEi8OLdnftsUYbLML\/1sRo7G2XB21UuR9I2WOrgyvhheV9k83y++o4CGVz\/AtmnTppuMpq10E0P9xBNPqIMHDwYvt+jv7w9CDw8PK9kPuK+vT42OjgY\/019LO32QeLaU8DNOkSZLPwm7HxUau9cgzREUUV\/XxhfDm2bG1o+N7yio4a30prX66RC\/hRjcjo6OioZaVncnJiZKL7io1JbEi886jy2LOFnmUYc0x4zGadJ1H7uo+rp8gQWG121e4zsKaHjlksR0Tk9PByuutg+9grxz5061bt26m8KHV3vlw+j38jMSz7YqfsUr6mTpF2W3o0Fjt\/zT7r3o+kaN7+imO9W9HYtSxYrY+OunAAAgAElEQVThTRVv3eD4jgIaXm1I09qHV7\/J7aGHHlLPPPOMunLlSlkNb3RFt5L51ol36NAhJXXF+mhra6ubtDTwn0DRJ0v\/FUh\/hGicPmOXPTSLvmJ8H3v+e+qVc5eV7Ogw8rHfTW0P3+PHj6tr166pZcuWqa6uLpfyBn0XXWPxKuFDPFF3d3fwF+iw73AuhKMB3DI3NzfnqO\/cdKsN9ZIlS4KyBTm2bdum5HtZUTYxvNGL7unpUZs3b84NCwZamcCNGzfUpUuXlPwC09LSAqYCEkDjAooauqRm0\/c7P35TffZrP1Gzv3xL\/cWqReqR1fZXe0+fPh2YzMWLF6uVK1c6T6Cia3zgwIHgOaPogeGdJ4LhrXALioGVP8XIIbsx7N69W3384x9X4ZIGeYhN3u529OhRtWfPnqCtLqeoVdIgb4Vrb28v9SoGiVVe5\/8ONjyA69evq4sXLwa\/RWN4G8bpZQA09lIWa4NqVn1Hvv5G8Na2Je9sUV\/4sw51b8et1pieOHFCXb16VS1fvjyYS10fRddYFufCq7yTk5NqZGSEFd63E68QhrdWSYO+wRYuXFjaYcH0ppMbVlZ0t2zZUqrhFcO7f\/\/+YMVXfqMK1w\/z0Jop4fy3L\/qfyvKvUONXgMaNM\/Q5QjPrK2UOnzx8NihzsLl\/LzW8bjOeGt5y\/oUwvHJJlUxmuJZW76Rw5MiRRBkY3olBAogBXrNmTbBrA9uSJUJaqJOaebIslJA1LgaNi600+iqltzF7z20LlI2H2jC8bu8ZDG8BDa9+qExKD8J734oR3bFjh9q3b1\/w52b5+qtf\/WriDAy\/XGLDhg1lO0Lw4onEWAtxIpNlIWSseRFoXGyN0XdeX5urvRhet\/cMhreAhlev8MrNtXfv3qDsQL95TRvTRld4G01bEq9Rgn6fz2Tptz42RofGNij6GwN9y7XRq72yk8PxT96VSDgMbyJs1k7CdxTU8MplyYqu7Hog24aFa3bDK72uHhAj8azdw14GYrL0Uharg0Jjqzi9C4a+N0tyevqy+tOx14Jty5KUOGB43aY5vqPAhtdtatXuncTzWZ3Gx8Zk2ThD3yOgse8KNTY+9K3MT5c4\/OjnbxqbXgxvYznZ6Nn4DgxvozmU6HwSLxG23JzEZJkbqRIPFI0To8vFiehbW6Y\/+dJrwS4O\/6H3rthvaMPwuk19fAeG10kGknhOsGfWKZNlZqiddYTGztBn0jH61scsW5cdfvVCbNOL4a3PNM0W+I6CGl79kFo0eeRFAPJyCFe1u3o8JF6at7X72EyW7jVIewRonDZht\/HRNx5\/k5VeDG88pmm1wncU0PDqF088\/vjjSt7s0tfXp1asWFG2V25aCRU3LokXl1Q+2zFZ5lM3k1GjsQmt\/LVF3\/iaxTW9GN74TNNoie8oqOHdunVr8ApgeZd0R0dH6YUQeh9eVnjTuJ2IqQkwWRY\/F9C42Bqjr5m+YnrlQbZ\/2tlV9UQMrxlT260xvAU0vPrVv0uWLFHr169X27dvV88884x67rnn1OzsLCUNtu8i4t1EgMmy+EmBxsXWGH3N9JXdG\/5g17dUrX16MbxmTG23xvAW0PDqS\/rMZz6jHnjggeCtamJ6V61apcbHx1Vra6vtPDKOR+IZI8vVCUyWuZIr0WDROBG23JyEvuZS6X16v\/Twnerhu9tuCoDhNWdq8wx8R4ENr81EsR2LxLNN1K94TJZ+6ZHGaNA4Dar+xETfZFronRuktEFeUBE+MLzJmNo6C9+B4bWVS0ZxSDwjXLlrzGSZO8mMB4zGxshydQL6JpdLlzbISi+GNzlH22fiOwpiePXODDMzMzVzhG3JbN9CxKtEgMmy+HmBxsXWGH2T6yt788pKb\/SlFKzwJmdq40wMbwENry+mtlaCkng2bl9\/YzBZ+quNrZGhsS2SfsZB38Z0qbTKi+FtjGmjZ+M7CmJ4w5cRfumEr+aXxGv01vX7fCZLv\/WxMTo0tkHR3xjo25g2epU3XMuL4W2MaaNn4zsKaHjDl9Tf36\/kJpODXRoavV04Py4BJsu4pPLbDo3zq12ckaNvHEq120RXeTG8jTNtJAKGt+CGV1+ervGV7229Wjhspjds2KCGh4dLNMfGxtTQ0FDw\/cDAQPDii\/BB4jVy2\/p\/LpOl\/xo1OkI0bpSg3+ejb+P6PP3y99XTL59XPxu+LwiG4W2caSMR8B0FNrz6BRRnzpwJrjJqShtJHDG0ExMTwb6+0s\/GjRvVpk2bSm90k9cZj46OBl3orzs7O0tdkniN0Pf\/XCZL\/zVqdIRo3ChBv89HXzv63Nb\/D0rvy4vhtcM0aRR8RwENb62V16SJEj1P+pBDr+qGvw+bYXnJhXymX2+s45B4tpTwMw6TpZ+62BwVGtuk6V8s9LWjSbisAcNrh2nSKPiOghje8LZkNldyqyVWpRXenTt3qnXr1gUGt5oZxvAmvVXzdR6TZb70SjJaNE5CLT\/noK8drfTDa1LWgOG1wzRpFAxvAQ1vrWSwuWuD3g1i4cKF6uDBg0qXLERXdMUcT09Pl9X46sQ7dOiQkjHpo63t5tcxJk1uznNHgMnSHfusekbjrEi76Qd97XB\/\/rWfBHvy\/uP\/freaeuU\/qmvXrqlly5aprq4uOx00EKXoGstCYPiQ9xR0d3cH5Zhh39EAwlyfesvc3Nxcrq8go8GLqZ2dnQ1qeOXYtm2bWrNmTVDDa2J4o8Pt6elRmzdvzugq6CYtAjdu3FCXLl1S8gtMS0tLWt0Q1yEBNHYIP4Ou0dce5D\/84nn12X\/\/LnXrT\/9JiclcvHixWrlypb0OEkYqusYHDhwIFuOiB4Z3ngiGt8KNE93abPfu3erjH\/+40iUMcoqs9u7atSvYAWLPnj1BlEr1vTq8XuEdHBxU7e3tpV7FILHKm\/BfL49Ou379urp48WLwWzSG1yNhLA4FjS3C9DAU+toT5bHnvxcEu\/+3vxc85L18+fJgm1DXR9E1lhXe8Crv5OSkGhkZYYX37cTD8Ma4A3W9cDXD++KLL5aVMPDQWgyoBWtS9D+VFUyuRJeDxomw5eYk9LUnla7j3b\/mF0FJwx133EFJgz28sSNRw1uOCsMbM3UqlTQsWbIkWNWdmpoqbUUm4diWLCbUAjVjsiyQmFUuBY2LrTH62tNXG97Pvf+iuu2\/\/68YXntojSJheDG8RgmjG9fb45cXTyTCWpiTmCwLI2XVC0HjYmuMvnb1lf14\/3z5ZbX6f\/wVhtcu2tjRMLwY3tjJYrMhiWeTpn+xmCz908T2iNDYNlG\/4qGvXT0wvHZ5JomG78DwJsmbhs8h8RpG6HUAJkuv5bEyODS2gtHbIOhrVxp5AcXiW34WrPJSw2uXbdxo+A4Mb9xcsdqOxLOK07tgTJbeSWJ9QGhsHalXAdHXrhzBXrz\/\/AP16O\/9FMNrF23saPgODG\/sZLHZkMSzSdO\/WEyW\/mlie0RobJuoX\/HQ164eT7\/8ffX3\/3BOyYNrrPDaZRs3Gr4Dwxs3V6y2I\/Gs4vQuGJOld5JYHxAaW0fqVUD0tSuH3qnhi384i+G1izZ2NHwHhjd2sthsSOLZpOlfLCZL\/zSxPSI0tk3Ur3joa1eP8NZkH3gfrxa2SzdeNHwHhjdeplhuReJZBupZOCZLzwRJYThonAJUj0Kir10xfvizN5U8uCYlDRheu2zjRsN3YHjj5orVdiSeVZzeBWOy9E4S6wNCY+tIvQqIvvbl0FuTPXx3G29as4+3bkR8B4a3bpKk0YDES4OqPzGZLP3RIq2RoHFaZP2Ii772dZAV3jXvvKAwvPbZxomI78DwxskT621IPOtIvQrIZOmVHKkMBo1TwepNUPS1L4Xei\/fJP7qNFV77eOtGxHdgeOsmSRoNSLw0qPoTk8nSHy3SGgkap0XWj7joa1+HP\/vC19W1a1cVhtc+2zgR8R0Y3jh5Yr0NiWcdqVcBmSy9kiOVwaBxKli9CYq+9qV45G9Pqee\/9xvqqx9bwAqvfbx1I+I7MLx1kySNBiReGlT9iclk6Y8WaY0EjdMi60dc9LWvw9D\/9X+rUz\/4b6zw2kcbKyK+A8MbK1FsNyLxbBP1Kx6TpV96pDEaNE6Dqj8x0de+FseOHVPXrl3jxRP20caKiO\/A8MZKFNuNSDzbRP2Kx2Tplx5pjAaN06DqT0z0ta8Fhtc+U5OI+A4Mr0m+WGtL4llD6WWg8+fPq2effVZt27ZNLV261MsxMqjGCKBxY\/x8Pxt97Svkm+FtNo3xHRjeunf1yZMn1f79+9X4+LhqbW0tte\/v71dyA8uxd+9etW7dutJnY2NjamhoKPh+YGBA9fb2lvVD4tXFnusG6Jtr+WINHo1jYcptI\/S1L51vhrfZNG62662XwbfMzc3N1WvUTJ+L2d2+fbtatWpVmeGVn+\/atUsdPXpUvf7666Wv29ra1NTUlOrr61Ojo6MBKv11Z2dnCR2JV+wsQt9i6ytXh8bF1hh97euL4bXP1CQiOc0Kb9V8kRXcU6dOBWb3ypUrZYZXPpNjeHhYXb16NfjT9ZYtW4JVXlndnZiYKLWXth0dHWWrvL4l3szMjHrhhRfUgw8+aO1P8I3END03Tvt6bWp9bvpZM+gr+V+Paa1\/jE3PjdO+XhtTHWtdYzNoXI9n1vrGyTlbGvumb5xrNzE\/um2WGh85ckRNT0+re+65R913330Vh1tvPEn0rcbON43rXXsSfcPXLuV1TzzxROBPKLVTihXeUEbJKm4lA6sN7po1awITG\/0+bIYlXPT78OrQoUOHvEg8udG6u7uVzfE0EtP03Djt67Wp9bnpZ\/X6SvoPV9Lz0hpPI3FNz43Tvl4bUx31ZFHp3qjXV1Ktkp6XxngaiWl6btz29drZ0rheP0l1auS8NMbUSEzTc7\/2ta+pb3zjG+rDH\/5wTcNbay5Kom+1+9h0\/I1oF+fctMaj4w4ODmJ4Q0JgeCtkZXTFNrqiq02tXsWNrujK+fJbrawGh3+rlt+0Jicn49wHtIEABCAAAQjkmkBXV5dasGCBunDhgjp79myuryWvg1+9erU6fPhwXodvddwY3owMr\/6NU37z4oAABCAAAQhAAAJpE5BSBsoZ5ik3peGVh8x6enqCOl05Ku24EK7JtVHSkHZSEx8CEIAABCAAAQhAoDKBpjS89ZIhWtIg7cNlC5UeWguXMFR6aK1en3wOAQhAAAIQgAAEIJAOAQxvBa6VDG+j25KlIx9RIQABCEAAAhCAAATqEcDwxjS8epU36Ysn6gnB5xCAAAQgAAEIQAAC6RDA8KbDlagQgAAEIAABCEAAAp4QwPB6IgTDgAAEIAABCEAAAhBIhwCGNx2uRIUABCAAAQhAAAIQ8IQAhtcTIRgGBCAAAQhAAAIQgEA6BDC86XAlKgQgAAEIQAACEICAJwQwvJ4IwTAgAAEIQAACEIAABNIhgOFNhytRIQABCEAAAhCAAAQ8IYDh9UQIhgEBCEAAAhCAAAQgkA4BDG86XIkKAQhAAAIQgAAEIOAJAQyvJ0IwDAhAAAIQgAAEIACBdAhgeNPhSlQIQAACEIAABCAAAU8IYHg9EYJhQAACEIAABCAAAQikQwDDmw5XokIAAhCAAAQgAAEIeEIAw+uJEAwDAhCAAAQgAAEIQCAdAhjedLgSFQIQgAAEIAABCEDAEwIYXk+EYBgQgAAEIAABCEAAAukQwPCmw5WoEIAABCAAAQhAAAKeEMDweiIEw4AABCAAAQhAAAIQSIcAhjcdrkSFAAQgAAEIQAACEPCEAIbXkhBjY2NqaGgoiDYwMKB6e3stRSYMBCAAAQhAAAIQgEAjBDC8jdB7+9ypqSnV19enRkdHg5\/orzs7Oy1EJwQEIAABCEAAAhCAQCMEMLyN0Hv7XFndnZiYUOPj46q1tVX19\/erjo4OVnktsCUEBCAAAQhAAAIQaJQAhrdRgkoFBleO4eHh4P\/R7y10QQgIQAACEIAABCAAgYQEMLwJwYVPi67oyorv9PR0yQDrtjMzMxZ6IwQEIAABCEAAAhCoT2Dp0qX1GzVJCwyvBaHjGF4xu3869pqa\/dcp4x5\/41c\/NTrnN371k5vaV\/7Zr+NW+tyoUxpDAAIQgAAEQgS6urrUggUL1IULF9TZs2dh44DA6tWr1eDgoML4KoXhtZCAcUoavv3tb6sNQ19Td9\/3x4l6\/NHP34x13g9\/Fq9dvWBRAxw13e+5bUEQ4vZb5\/8vx49\/PBMYernBPvT7HaWfV2ob7X9yclKNjIwEN2Z7e3u94ak47eu1qfW56Wf1+qp7QZYbpDWeRuKanhunfb02pjqKDNXOqdeXZQnrhktjPI3END03bvt67WxpXK+fuoKk0CCNMTUS0\/Tcb33rW+q73\/2u+uAHP6je\/\/73VyRUL2YSfavdx\/X6SkHCmiHTGo+O++CDD6oXXngheMYIw4vhtZLf0RKGSg+tieHt7u52lnhRIywrzt\/6T9Nq9eoPlhiETfUPf3a9jE34\/GgsfZ6J2dYmOGyaX\/2Hryq5QfVn77ntN8tMtfxcfybjlxv5scceq6phvTa1Pjf9zLW+UQj1rj1p4jcS1\/TcOO3rtTHVUbhUO6cZNK7Hs1bemJ4bt329drY09k3fWrmY9P5tNGY9LaLjOnLkSFDet3btWiWrvZWOejGT6FvtOn3TuN61J9VZx5XFJ5e+I+n40zqPFV4LZONsS+bbjWbhsmuGqGaQf22Of22odVv9f2lTzzxHV43DZlgGds+KW4PxRX+e1nU3m75pcfQ5Lhr7rE7jY0PfxhlGIxw7dkxdu3ZN3XHHHVUNr\/1eq0dsNo2b7Xrr5RKGtx6hmJ\/Xe\/EEiRcTZIVmlQ1xuWGOY5bDJjlshLU5vrdjUeJBom9idLk5EY1zI1WigaJvImw1T8Lw2mdqEpGcLqeF4TXJngbakngNwEtwatgkz68Y32yQq60kixmW2mQTU3z+\/Hn17LPPqp07d6qWlpYEI+YU3wmgse8KNTY+9G2MX6WzfTO8zaYxvgPDa\/+ujhGRxIsByVGTaub49PRlVckU65Xie1YsKtUUr162UC17xxV1++23Y3gd6Zh2t2+++aZ644030Dht0I7io6998L4Z3mbTGN+B4bV\/V8eISOLFgORxE22KXzl3ubRaXMsQywqxlEjIg3f6a48vj6HFINBsk2UMJIVqgr725cTw2mdqEhHfgeE1yRdrbUk8ayi9DPSvs5fV6\/\/2hvqXX7QEK7xihsUchw9dKqFrhaV2uJG6YS9BFHhQGKICi6uUQl\/7+mJ47TM1iYjvwPCa5Iu1tiSeNZReBqo1WcrqsF4ZFiMsR9gMixHW5RGYYC\/lDQaFIfJXGxsjQ18bFMtjYHjtMzWJiO\/A8Jrki7W2JJ41lF4GSjJZihE+\/OobwfVEV4Qxwf7JnERj\/66CEVUjgL72cwPDa5+pSUR8B4bXJF+stSXxrKH0MpCtyVJM8PyK8M8rmuCH724Lrv\/huxeXHpjzEkgBB2VL4wKiKcQloa99GTG89pmaRMR3YHhN8sVaWxLPGkovA6U5WVZbCQ6vAmOA00+LNDVOf\/T0UI8A+tYjZP45htecmc0z8B0YXpv5FDsWiRcbVS4bZjlZVlsFxgCnmzpZapzulRC9EgH0tZ8XGF77TE0i4jswvCb5Yq0tiWcNpZeBXE+WehU4XAssBlhKIHgQzk7KuNbYzlUQpRoB9LWfGxhe+0xNIuI7MLwm+WKtLYlnDaWXgXybLLUBPvzqhaAmWA5tgCl\/SJZCvmmc7Co4C8ObXQ5geLNjXaknfAeG10kGknhOsGfWqc9miNVfO2ngs8Z2rrC5o6Cvff0xvPaZmkTEd2B4TfLFWlsSzxpKLwPlabLUBvjpl8+XrfxS+lA7tfKksZc3ieeDQl\/7AmF47TM1iYjvwPCa5Iu1tiSeNZReBsrrZBktfaDut3p65VVjL28YDweFvvZFwfDaZ2oSEd+B4TXJF2ttSTxrKL0MVITJspr5peZ3PuWKoLGXN48ng0Jf+0JgeO0zNYmI78DwmuSLtbYknjWUXgYq2mRZrezh0\/e\/10v+WQyqaBpnwSxPfaCvfbUwvPaZmkTEd2B4TfKl1PbChQtq48aNamZmpvSzgYEB1dvbG3w\/NjamhoaGgq\/DP9eNSbxE2HNzUpEny6j5vWfFItW9anGw5VkzHUXWuJl0rHat6Gs\/CzC89pmaRMR3YHhN8qXUdmpqSu3YsUPt27dPtbWVT\/TyWV9fnxodHQ3a6687OztL55N4ibDn5qRmmCyjuz002zZnzaBxbm64FAaKvvahYnjtMzWJiO\/A8JrkS6ntyZMn1f79+9X4+LhqbW0tiyGruxMTE6XP+vv7VUdHR2n1VxqTeImw5+akZpssK5U8SLlDkVd9m03j3Nx8lgaKvpZAhsJgeO0zNYmI78DwmuRLqW3U1IaDiMGVY3h4OPh\/9HsMbyLkuTqpmSfLp1\/+vtIvuNCrvkWs9W1mjXN1MyYcLPomBFfjNAyvfaYmETG8GF6TfCm1FRMrN68+Vq1aVXVFV8zx9PR0yQCHDe+hQ4fU0qVLS3Gi5RGJBsdJzgkwWargjW6HX31DPXNqvs790\/cvVx+763eCN7wV4UDjIqhY\/RrQ176+x48fV9euXVPLli1TXV1d9jswjFh0jeVZo\/Ahzxx1d3cHf4EO+w5DbIVpfsvc3NxcYa4mpQu5evWq2rZtm1qyZElgYqPfR0sYahne6BB7enrU5s2bUxo5YbMicOPGDXXp0qWgvrulpSWrbr3sZ\/aXb6njZ6+qr\/zzVSVf\/2H7AvXZf\/8uteSd+eaCxl6mm7VBoa81lKVAp0+fDrbzW7x4sVq5cqX9DgwjFl3jAwcOqIMHD95EBcM7jwTDW+GGCa\/mhldyw02lpnfXrl3q6NGjas+ePcFHcUoaBgcHVXt7eymUGCRWeQ3\/1fKw+fXr19XFixeD36Kb3fCG5Rn5+hulcgcxvP1\/tDTY4SGPBxrnUbX4Y0bf+Kzitjxx4kSwQLR8+XIlc6nro+gaywpveJV3cnJSjYyMsML7duJheBPegeGH2OQ3qnAJAw+tJYSa49OK\/qeyRqWRcodPHj6rXjl3OShxkIfb8lbni8aNZoHf56OvfX2o4bXP1CQiNbzltDC8MbJH78G7c+dOtW7duuA3KNmTd9OmTcFODGxLFgNiwZswWcYTWIyvfsgtb8YXjeNpnNdW6GtfOQyvfaYmETG8GF6TfCm1jb54YsOGDWUPpfHiiURYC3MSk6WZlOFtzfJifNHYTOO8tUZf+4pheO0zNYmI4cXwmuSLtbYknjWUXgZiskwmS56MLxon0zgvZ6GvfaUwvPaZmkTEd2B4TfLFWlsSzxpKLwMxWTYmSx6MLxo3prHvZ6OvfYUwvPaZmkTEd2B4TfLFWlsSzxpKLwMxWdqRxWfji8Z2NPY1CvraVwbDa5+pSUR8B4bXJF+stSXxrKH0MhCTpV1ZosZ3dNOd6t6ORXY7MYyGxobActYcfe0LhuG1z9QkIr4Dw2uSL9baknjWUHoZiMkyHVmi25m5NL5onI7GvkRFX\/tKYHjtMzWJiO8ooOHVbz47c+bMTblQ7cURJkljoy2JZ4OivzGYLNPV5vT0ZdV35Gzw+uJ7VixSX3r4zsxfWYzG6WrsOjr62lcAw2ufqUlEfEeBDK\/sfyuv5pVDXv7Q2dl5Uy7ICyK2b9+uFi5cWLWNSQIlbUviJSWXj\/OYLLPRSe\/hK8b30\/cvz\/TlFWicjcauekFf++QxvPaZmkTEdxTE8Mq+uPJK36eeekq1trbWzQFZBX7yySfL9s6te5LFBiSeRZgehmKyzFYUMb5Pv3w+07e2oXG2GmfdG\/raJ47htc\/UJCK+oyCG10R0H9qSeD6okN4YmCzTY1stcvTBNnlVsbyyOK0DjdMi60dc9LWvA4bXPlOTiPgODK9JvlhrS+JZQ+llICZLd7JEH2z7D713pVLfi8buNM6iZ\/S1TxnDa5+pSUR8RwENb\/S1v5USghpek9uEtqYEmCxNidlvL8b3T8deCx5sS6O+F43ta+ZTRPS1rwaG1z5Tk4gY3gIaXrmk\/v5+1dHRoXp7e0tXODY2pqanp4O6Xfl6YmJCHTlyxCRfrLUl8ayh9DIQk6U\/sqRV34vG\/micxkjQ1z5VDK99piYR8R0FNLyywrt161a1e\/fusp0aZBeHHTt2qH379qmLFy8GX3\/1q181yRdrbUk8ayi9DMRk6Zcsaby4Ao390tj2aNDXNlGlMLz2mZpExHcU0PDqFV65ufbu3avWrVun9HZkGzZsYIXX5A6hbSICTJaJsKV+Uri+t9H9e9E4dbmcdoC+9vFjeO0zNYmI4S2o4ZXL0vvyXrlypWzf3fBKb1tbek9x10pEEs\/kNs1fWyZLvzULv7giaX0vGvutcaOjQ99GCd58PobXPlOTiPiOAhtek0TIui2JlzXxbPtjssyWd9LeGqnvReOk1PNxHvra1wnDa5+pSUR8B4bXJF+stSXxrKH0MhCTpZeyVBxU0vpeNM6PxklGir5JqNU+B8Nrn6lJRHxHQQ2vvElt27Zt6syZM2rVqlXq0UcfVZ\/\/\/OeDB9ZMyxik\/nf\/\/v1qfHy87C1ushOE3MBy6FphjVN2gRgaGgq+HRgYKNstQn5G4pncpvlry2SZP83C9b1xyhzQOH8am4wYfU1oxWuL4Y3HKa1W+I4CGl5tdtesWRNsTabN6sGDB4OtyKLGtVZy6YfdxDSHz5Of79q1Sx09elS9\/vrrpa\/FTEuNcF9fnxodHQ1C6687OztLXZF4ad3SfsRlsvRDhySjiFvmgMZJ6ObnHPS1rxWG1z5Tk4j4jgIa3vC2ZLL9mDa8586dK21LFmeVV1ZwT506FawQy4NvYcMrn8khe\/pqg71ly5ZgRwi9x69uX2lPYBLP5DbNX1smy\/xpFh5xnDIHNM63xvVGj771CJl\/juE1Z2bzDHxHAQ2vXJKYzNnZWfXQQ\/mBHfkAACAASURBVA+p5557Lihp+MQnPqHWrl0bmNQ4h6ziVjKw4RVkebFF9PuwGdZj0eZY96sT79ChQ2rp0qWl4cQx4nHGThu3BJgs3fK31bsY38ee\/5565dxlJduYjXzsd0uvKUZjW5T9jIO+9nU5fvy4unbtmlq2bJnq6uqy34FhxKJrLIt\/4WNmZkZ1d3cHf+kO+w5DbIVpfsvc3NxcUa5GlyPo66lUSxvnWqMrttEVXW1q9Zvdoiu64Te8RQ1vtP+enh61efPmOMOijccEbty4oS5duhTUi7e0tHg8UoYWh8B3fvym+osX5yePv1i1SD2yepFC4zjk8tsGfe1rd\/r0aSUmc\/HixWrlypX2OzCMWHSNDxw4oKSUM3pgeOeJFMrwGuZ+1eZpGt7BwUHV3t5e6lsMEqu8tpRzF+f69evB2\/zkt2gMrzsdbPc88vU31NMvn1dL3tmiHl3TptYs+a9obBuyJ\/G4h+0LceLEieAvosuXLw9KBV0fRddYVnjDq7yTk5NqZGSEFd63E68pDW\/4BRXCodKOC+GH3WyWNPCblut\/8tLpv+h\/KkuHWj6iSpmDPNh2+NULgfHd++fvV\/d2LMrH4BllbALcw7FRxW5IDW9sVKk0pIa3HGtuDa\/8FrNx40YlNSq1Dllxk50VTFZRoyu8Ej9ctlDpobXp6elSrTAPraVy73odlMnSa3msDO5fZy+rB\/7uP6nZX76l4mxjZqVTgmRGgHvYPmoMr32mJhExvAUxvFHRK5nMSsY1TrJUOo9tyeKQa942TJbF115rfOj\/fUs9c2omeJjt4bvb1Kfvf2\/xL74JrpB72L7IGF77TE0iYngLaHjD25KF976V0oUdO3YYv3yimlHmxRMmt1pztWWyLL7eYY1llffwq\/P1vWJ8xfSK+eXILwHuYfvaYXjtMzWJiOEtoOGVS9JmVNfj6h0bNmzYEHtbMpNEMm1L4pkSy1d7Jst86ZVktJU0Dtf3ivEd3XQn9b1J4HpwDvewfREwvPaZmkTEdxTU8MplhR9GW7hwYbA9R3jF1yRRbLcl8WwT9Ssek6VfeqQxmloah19TLPv3funhO0v796YxFmLaJ8A9bJ8phtc+U5OI+I4CG16TRMi6LYmXNfFs+2OyzJa3i97iaCw7OciODmKAebDNhUrJ+4yjb\/LozXkmhtet7vgODK+TDCTxnGDPrFMmy8xQO+vIRGMxvbq+lwfbnElm1LGJvkaBm7gxhtet+PiOghheeVBtz5496qmnnlKtra11s0q2EnvyySed1fOSeHUlynUDJstcyxdr8KYayypv+ME2jG8szM4amerrbKA56hjD61YsfEdBDK9chq7Zla+r1evqh9dc1\/SSeG5v\/LR7Z7JMm7D7+Ek1xvi61y7OCJLqGyd2s7bB8LpVHt9RIMOrL0W\/COLMmTM3ZZe8znB8fDzWKnCaqUnipUnXfWwmS\/capD2CRjWO7ujAVmZpK2YWv1F9zXprjtYYXrc64zsKaHjdplS83km8eJzy2orJMq\/KxR+3LY3DOzqwlVl8\/mm3tKVv2uPMU3wMr1u18B0YXicZSOI5wZ5Zp0yWmaF21pFtjaPGlxVfZ9IGHdvW1+3V+NE7htetDvgODK+TDCTxnGDPrFMmy8xQO+soLY0xvs4kLes4LX39uDo3o8DwuuGue8V3YHidZCCJ5wR7Zp0yWWaG2llHaWuM8XUmLSu8KaHH8KYENmZYfAeGN2aq2G1G4tnl6Vu0tM2Qb9fbjOPJSuPow21sZ5ZNtmWlbzZX40cvGF63OuA7MLxOMpDEc4I9s06ZLDND7ayjrDXG+GYrddb6Znt1bnrD8LrhrnvFdxTU8Ia3JpOtyB599FH1+c9\/Xu3bt0+1tbW5zTqlFInnXIJUB8BkmSpeL4K70rjSPr4P371YyQ4PHPYIuNLX3hX4FwnD61YTfEcBDa82u2vWrFEdHR1q\/\/79wd678jKKiYkJ9uF1e881Re9MlsWX2bXGYeMrtHWpA8bXTu651tfOVfgVBcPrVg8MbwENr7xmeOvWrWr37t3q4sWLJcN77tw5tWPHDi9WeUk8tzd+2r0zWaZN2H18nzR++uXvq8OvXlBigsXwsqVZ4\/nhk76NX40fETC8bnXAdxTQ8Mol9ff3q9nZWfXQQw+p5557Lihp+MQnPqHWrl2rhoeHjbJOXkesV4lbW1uDc8VUb9y4Uc3MzJRiDQwMqN7e3uD7sbExNTQ0FHwd\/rluTOIZSZC7xkyWuZPMeMA+alypzpdyB2NpgxN81DfZlfhzFobXrRb4joIaXrksMarbt2+vaEjjpp2OEX0l8dTUVNXVYvmsr69PjY6OBt3orzs7O0vdknhxFchnOybLfOpmMmqfNY6WO7Dqa6LsfFuf9TW\/Gj\/OwPC61QHfUWDD22hqySrxqVOnlJjdK1eulNX+Vlr11f3J6m64VljiSC2xXv2VdiReo+r4fT6Tpd\/62BhdXjSOljuwrVk89fOib7yr8aMVhtetDvgODG\/VDBRTu27duqA8IfqwW6Wf6UBicOXQpRPR78OG99ChQ2rp0qWlMfiwg4TbW7IYvTNZFkPHWleRN431qu8zp+bLsGTVV8zvX6+9vfhiJbjCvOmb4BIzP+X48ePq2rVratmyZaqrqyvz\/qMdFl1jKb0MH1KC2d3dHfiZsO9wLoSjAdwyNzc356hvq92KWd21a5c6evSoevHFF4N62oULFwY7NYRLC+J0WsnciomV31b1ES55iK7oyvnT09NltcP6N61o\/z09PWrz5s1xhkUbjwncuHFDXbp0KdgCr6WlxeORMrSkBPKq8ewv31LHz15V3\/nxm8F\/S97Zoj76vlb1gaUL1B+2s7WZzoe86ps0n7M47\/Tp00GpyOLFi9XKlSuz6LJmH0XX+MCBA4HniR4Y3nkihTC8eluyLVu2qN\/\/\/d8PHi7buXNncIHRh8\/i3HFRw6vjL1myJDCx0e9NDO\/g4KBqb28vDUMMEqu8cVTxu83169eDHULkt2gMr99aJR1dETQW8\/v8a5dKOzyI+f3YXe8Kdnlo9qMI+vqm4YkTJ4L5cvny5UGpoOuj6BrLCm94lXdyclKNjIywwvt24hXC8Ea3JdMrvWJAKm1LJg+Zycqq1OnKsXfv3qCUQR+1yhd0m\/CK8p49e4Ifxylp4Dct1\/\/kpdN\/0f9Ulg61fEUtmsaVHnRr5nrfounrw91FDa9bFajhLedfCMMbXuGVUgJdf5v0xRNxDW\/4BRfhEgYeWnN7k7voncnSBfVs+yyyxvKg27wBnq8B1PW+zbTyW2R9s71Tft0bhtcV+fl+MbwFNLxySXo7MV23Kz+rtD1YnPSLGl69B6+USchKsP5+06ZNwU4MbEsWh2qx2zBZFltfubpm0Fiv+jaj+W0GfbO+SzG8WRMv7w\/DW1DDazOtKq3wRl88sWHDhrKH0njxhE0F8heLyTJ\/mpmOuNk01ub39PRl9cq5y2Urv0V8uUWz6Wua\/0naY3iTULN3DoYXw2svmwwikXgGsHLYlMkyh6IZDrmZNa618nvPilvVvR2LDGn617yZ9U1LDQxvWmTjxcV3FNTwRt+ypi9TnpqXrcpc74RA4sW7QfPaiskyr8rFHzca\/5rV\/Krvz9XTL58vW\/l9z22\/Gez1m8cDfe2rhuG1z9QkIr6jgIZXlxs8\/vjjSrZBkdrdFStWqG3btqk1a9aUvfHMJFlstiXxbNL0LxaTpX+a2B4RGlcmqld\/5YE3+VoO\/dBbnkof0Nf2HaOCvevlxRN33HEHL56wj7duRHxHQQ3v1q1b1e7du5VsvKxf6ysPk1XalqxulqTQgMRLAapHIZksPRIjpaGgcX2wtep+fS99QN\/6+pq2wPCaErPbHt9RQMMbfhHE+vXr1fbt29UzzzyjnnvuOTU7O0tJg917iGgVCDBZFj8t0Nhc41qrv74ZYPQ117feGRjeeoTS\/RzDW0DDqy\/pM5\/5jHrggQeCN16J6Q2\/\/jfdtKofncSrzyjPLZgs86xevLGjcTxO1VpVe\/DtnhWLghII13v+om9j+lY6G8Nrn6lJRHxHgQ2vSSJk3ZbEy5p4tv0xWWbL20VvaGyXeq3yB+kpawOMvnb1lWgYXvtMTSLiOzC8JvlirS2JZw2ll4GYLL2Uxeqg0NgqzrJgYn7lP9n5odLDb1mUP6CvfX0xvPaZmkTEd2B4TfLFWlsSzxpKLwMxWXopi9VBobFVnDWDzZvfy+qHP7te2vpMTtC7P6RhgNHXvr4YXvtMTSLiOwpqeNmH1+Q2oK1tAkyWton6Fw+N3Wmiyx9kBHrvX9sGGH3t64vhtc\/UJCKGt4CGV+\/Du2nTJi\/23K2UkCSeyW2av7ZMlvnTzHTEaGxKLL321ep\/b791QfDWtyQrwOhrX6\/9R7+i\/unCfwleRtLV1WW\/A8OIzaYxvqOghlfvw9vZ2Wl4C2TTnMTLhrOrXprtH1JXnF32i8Yu6dfuu54BjvMCDPS1r+\/\/se+k+vJ\/\/u\/UVz+2AMNrH2\/diPiOAhpeuSQpaXjppZfU8PBw3SRw0YDEc0E9uz6ZLLNj7aonNHZF3rzfaga41hZo6GvOud4ZGN56hNL9HN9REMOryxhmZmZqZszSpUt58US69xTRlVJMlsVPAzTOr8a1tkB7z22\/GfzJHX3t64vhtc\/UJCKGtyCG10R0H9qSeD6okN4YmCzTY+tLZDT2RYnGxlFrC7T1HQvU+j+4XX34fe9qrBPODgg88ren1KkfzKn\/83\/+bUoaHOQEvqNghre\/vz\/Y3FqOvXv3qnXr1jlIq\/pdknj1GeW5BWYoz+rFGzsax+OUt1Z69fett95Sz5ya\/4uhbH+mH4CLU\/+bt2vOarwY3qxIV+4H31Egwytm9x\/\/8R+DkgV5nXBPT4965JFHEu\/UEDbPCxcuVAcPHlThh+BqmeuxsTE1NDQU0B0YGLhpDCSe2xs\/7d4xQ2kTdh8fjd1rkOYItL5zv\/076vnXLqnT05eDvYC1AZayhyS7P6Q5Zt9jY3jdKoTvKIjhlRre6M4M8uDa\/v371fj4uGptbTXKNDGsExMTpXPl+yNHjpTqfyX2rl27gu9ff\/310tdtbW1qampK9fX1qdHR0aBP\/XXYLJN4RnLkrjFmKHeSGQ8YjY2R5eqESvpWK3+Qh99k+7OsX3+cK6CUNDiXC9+B4Y2VhGETK8ZVVnflkF0grl69qrZt26a2bNkSlFBEzbK07ejoKFvlJfFiYc9tI8xQbqWLPXA0jo0qlw3j6Fvr4TdWf2+WnRVet7cCvgPDGysDw4Z3xYoVgcFds2ZNYGK14dXfh82wBI9+Lz8j8WJhz22jOJNlbi+OgQcE0LjYiWCqb3j1V7\/9Lc1XH+eRPobXrWr4DgxvrAwU0zo7OxuUOMgRXtHVplav4kZXdGXFd3p6umxPYJ14hw4dUrJVmj6kJIIj\/wRMJ8v8X3HzXQEaF1vzRvXVq7\/Pv\/YTJV9r8yvbnn3srubc9WHT6CvqBz\/9lRpe18ouDRncPlLqGT5k29bu7u6gXDPsOzIYipdd3DI3Nzfn5cjqDCrNfXjFsH75y18uPbQWLWFoxPBGL0setNu8eXMeJWDMIQI3btxQly5dUvILTEtLC2wKSACNCyhqSvfw7C\/fUsfPXlVf+eerSr5e8s4W9dH3tQb\/\/5M7zZ4vyTP1x57\/XnD9n\/3gf1MrV650filFv4cPHDgQ+JbogeGdJ5Jbw9vInSPlCmI0r1y5EoQJb2cWNbvyebSEoZGShsHBQdXe3l4avhgkVnkbUdOPc69fvx7sFCK\/RWN4\/dDE9ijQ2DZRv+Klpa8YPr3rw49+\/qaS7c9kxXf+wbdb\/YJgeTT\/y99+S81cvqEG1\/6WWrVqleXo5uHS0th8JOmcIQuB4VXeyclJNTIywgrv27ib0vBWS7XozgzhduGyhUoPrYVLGHhoLZ2b2eeojf451OdrY2zzBNC42JmQhb667OHwqxfKyh6Kutfvn33h6+pHP7+uvrD+f6CkwcHtQw1vOXQM79s8ZNuxJ5544qa9dzUutiVzcLfmqMssJssc4SjkUNG4kLKWLiprfbX5jT7wVqStzv5oz2l17dpVDK+jWwfDi+GtmHrhl0qEG4TLHXjxhKO7NgfdZj1Z5gBJ4YaIxoWTtOyCXOobNb\/ykgspd5D\/5\/kQw\/uO\/\/IL9eQf3cYKrwMhMbwYXgdpx7ZkTqBn2KnLyTLDy2zqrtC42PL7oG+1koe8rvpieN3eMxheDK+TDCTxnGDPrFMfJsvMLrZJO0LjYgvvm75FWPXF8Lq9Z\/AdGF4nGUjiOcGeWae+TZaZXXgTdYTGxRbbV33zvOqL4XV7z+A7MLxOMpDEc4I9s059nSwzA9AEHaFxsUXOg75ifp9++ftKdnnQL7bwudwBw+v2nsF3YHidZCCJ5wR7Zp3mYbLMDEZBO0Ljggr79mXlSd9oucOn71+ufNza7A92fUutfMcl1Xfv7\/DQmoPbB9+B4XWQdjy05gR6hp3mabLMEEuhukLjQsl508XkUd+o8ZWXWXzp4TuD1V8fDgyvWxUwvBheJxlI4jnBnlmneZwsM4NTkI7QuCBCVrmMvOurSx3EBPtifDG8bu8ZfAeG10kGknhOsGfWad4ny8xA5bgjNM6xeDGGXhR9T09fVn1HzgZvcnNtfDG8MRIvxSb4DgxviulVPTSJ5wR7Zp0WZbLMDFgOO0LjHIpmMOSi6evDiu9t\/f+g\/qfFV6jhNchDm03xHRhem\/kUOxaJFxtVLhsWbbLMpQgpDxqNUwbsOHxR9Q0b36wfbhPD++fLLwdvjOvq6nKssFJF1bgaWHwHhtfJTUfiOcGeWafN9g9pZmA96giNPRIjhaEUXV8xvk+\/fD4gJ8Y37e3MpKRCShoe\/b2fqPs7l2J4U8jZeiHxHRjeejmSyuckXipYvQla9MnSG9AOB4LGDuFn0HWz6KuNb9r7+GJ4M0jaOl3gOzC8TrKQxHOCPbNOm2WyzAyohx2hsYeiWBxSM+kb3s5MHmyT1d57OxZZpKmUPDz3p2Ovqc+9\/6L6wPuWscJrlW68YPgODG+8TLHcisSzDNSzcM00WXqGPrPhoHFmqJ101Iz6ivH95OGz6pVzl4MdHY5\/8i5r7DG81lAmDoTvwPAmTp5GTiTxGqHn\/7nNOFn6r4rdEaKxXZ6+RWtmfbU5FU1s1ffK64\/FTH\/xD2fVHXfcwQqvg4THd2B4HaQdb1pzAj3DTpt5sswQs9Ou0Ngp\/tQ7R1+lwvW9o5vubKjMQWL9\/T+cC0oaMLypp2\/FDjC8GN6qmdff36+OHTsWfL5w4UJ18OBB1dnZGXx\/4cIFtXHjRjUzM1M6f2BgQPX29gbfj42NqaGhoeDr8M91YxLPzQ2fVa9MllmRdtcPGrtjn0XP6DtPOVzm0MhqL4Y3i6yt3Qe+A8NbMUPEsE5MTKjx8XHV2toaGNgjR46oo0ePqra2NjU1NaV27Nih9u3bF3wfPuSzvr4+NTo6GvxYf63NsvyMxHN\/86c5AibLNOn6ERuN\/dAhrVGgbznZ8GqvPNQme+maHHL+i9\/8F\/Xo7\/2UFV4TcBbb4jswvLHSKWxixbiePHlS7d+\/v2SIw0GiZllWijs6OkqrvxjeWMhz3YjJMtfyxRo8GsfClNtG6HuzdNHdHEweavuTL72m3ro8G7x4gpIGN7cFhhfDGyvzooY3amrDQcTgyjE8PBz8P\/o9hjcW8lw3YrLMtXyxBo\/GsTDlthH6VpdOHmrrO3I2KHeIW+YgL51Y+Y5L6iNLrmB4Hd0VGF4Mb6zUE9M6OztbWtEN1\/dKgFWrVpV9Fl7RFXM8PT1dMsBhw3vo0CG1dOnS0hii5RGxBkcj7wgwWXonifUBobF1pF4FRN\/6ckiZwjOnZpS8tOILf\/a7VR9qm\/3lW8Fb1v7ydy+p3134\/6lly9iHtz7dxlvIs0bhQ5456u7uDso1w76j8Z7yGeGWubm5uXwOPb1Ri2H98pe\/XHpo7erVq2rbtm1qyZIlgYmNfh8tYahleKOj7unpUZs3b07vYoicCYEbN26oS5cuBfXdLS0tmfRJJ9kSQONseWfdG\/rGIy5m9rNf+4n6zo\/fVH+xapF6ZPXNL6wIPnvxghq5a0bJLxKLFy9WK1eujNdBiq2KrvGBAwcC3xI9MLzzRJrS8Eq5ghjNK1euBBD27t2r1q1bF3wdNbvV7j2p6d21a1fwUNuePXuCZnFKGgYHB1V7e3sprBgkVnlT\/Bcuo9DXr19XFy9eDH6LxvBmBD3jbtA4Y+AZd4e+ZsBHvv6Gevrl82rJO1vUx+56V\/C2Njmk7OGx578X\/Dv4vy3+frBAtHz58uCvoq6PomssK7zhVd7JyUk1MjLCCu\/bideUhrfaTRfdmaHWzRl+iE1+owqXMPDQmut\/1rLvnz+HZs886x7ROGvi2faHvua8ww+1SZmDHLffOv9\/McA\/+e7\/o65du0YNrzlaK2dQw1uOEcP7Ng8xsE888UTZ3rsald6Dd+fOncFKsP5+06ZNwU4MbEtm5d7MdRAmy1zLF2vwaBwLU24boW9y6bTxlf+L8X347sXB\/2Vfewxvcq6NnonhxfBWzKHoQ2m6kS53iL54YsOGDWUPpfHiiUZvzXyfz2SZb\/3ijB6N41DKbxv0ta8dhtc+U5OIGF4Mr0m+WGtL4llD6WWg8+fPq2effTZ4uJGnYb2UqOFBoXHDCL0OgL725fHN8DabxvgODK\/9uzpGRBIvBqQcN0HfHIsXc+hoHBNUTpuhr33hfDO8zaZxs11vvQymhrceIUufk3iWQHoaBn09FcbisNDYIkwPQ6GvfVEwvPaZmkQkp1nhNckXa219SzzZkPqFF15QDz74oLU\/wTcS0\/TcOO3rtan1uelnzaCv3Az1mNa6YUzPjdO+XhtTHWtdYzNoXI9n1vrGyTlbGvumb5xrTzJBZanxkSNHgh2M7rnnHnXfffdVHG698STRtxo73zSud+1J9A1fu5TXycP47MM7T5IV3qQZZXievtGib1ozDGOtuX4Di83xNBLT9Nw47eu1qfW56Wf1+rImXMxAaY2nkbim58ZpX6+NqY56spC3E0XvjXp9xZTGWrM0xtNITNNz47av186WxvX6sSacQaA0xtRITNNzv\/a1r6lvfOMb6sMf\/nBNw1vpftOYkuhb7T42Hb+BVImapjUeHVf2\/cfw\/loaDG+iNDU\/SRJQEk82guaAAAQgAAEIFJ1AV1eXWrBgQbCV59mzZ4t+uV5e3+rVq9Xhw4e9HFvWg8LwZkhcTK\/8xwEBCEAAAhCAAATSJiBlDewcNE8Zw5t2thEfAhCAAAQgAAEIQMApAQyvU\/x0DgEIQAACEIAABCCQNgEMb9qEiQ8BCEAAAhCAAAQg4JQAhtcpfhUU82\/cuDGo7Y2+rtjx0OjeMoHPfOYz6oEHHlCdnZ2WIxPOJYGpqSnV09Ojrly5wj3sUogU+w7\/Oz0wMKB6e3tT7I3QLgmcPHlSvfTSS2p4eNjlMOg7BQIY3hSgmoQcGxsLmsuE+fjjj6u+vj4MkQnAHLS9evVq8MpheUr54MGD6JsDzUyGGP5Fhl9qTMjlp638O93R0aE+9KEPqSeffFJ96lOfUm1tbfm5AEYai4D+xeYDH\/gAhjcWsXw1wvBa1kubmy1btqh169aVoss\/mENDQ8H34RWC8ASp\/1ENn2d5eISzQMBU43\/7t38Lej1w4AArvBb4px3CVN\/weDC8aatjJ35SjcUQ7dmzRz311FOqtbXVzmCIkgqBJBr39\/erd7\/73epXv\/qV+tznPpfKuAjqjgCG1yJ7fYOdOXNG7d27t2R45U+esnI7Ojoa9Ka\/lj9tY3gtCpBBqCQa62FhhjIQqMEuGtGXP4U2CD+j05NqrEtXHnnkEUoaMtIqaTdJNNb37+bNm9WLL76I4U0K3+PzMLyWxNH\/GN55551qdnZW7dy5s2R4ZeVWXu03Pj4erArIb5Hy5zGpA8PwWhIggzBJNcbwZiCOhS4a0VfucXmFKnV\/FoRIMUQjGuth8Ze4FAWyEDqpxjIvHzt2rDSC8KKVhWERwgMCGF5LIug\/W\/\/Wb\/1W8BBa2PDKjSSHngzD31PDa0mADMIk1RjDm4E4FrpIqq+sDInZ5UEmCyKkHCKpxvJv9vr164NFjPDXKQ+X8AkIJNVYdyWGmRXeBOBzcAqG17JIuug9anj1iq50F14N4ulfywJkEM5UYwxvBqJY7MJEX6nllAcSpYxJH6wMWRQjpVAmGstCBbvppCREimFNNcbwpiiGJ6ExvJaFSHqTWR4G4VIkgMYpwvUgNPp6IELKQ0DjlAF7EB6NPRDBsyFgeC0LUu0mk24qlTRY7p5wGRBA4wwgO+wCfR3Cz6hrNM4ItMNu0NghfE+7xvBaFqbSTRZ9oCX80Jrl7gmXAQE0zgCywy7Q1yH8jLpG44xAO+wGjR3C97RrDK9lYSrdZLW2JbPcPeEyIIDGGUB22AX6OoSfUddonBFoh92gsUP4nnaN4bUsTKWbTLqo9uIJy90TLgMCaJwBZIddoK9D+Bl1jcYZgXbYDRo7hO9p1xheT4VhWBCAAAQgAAEIQAACdghgeO1wJAoEIAABCEAAAhCAgKcEMLyeCsOwIAABCEAAAhCAAATsEMDw2uFIFAhAAAIQgAAEIAABTwlgeD0VhmFBAAIQgAAEIAABCNghgOG1w5EoEIAABCAAAQhAAAKeEsDweioMw4IABCAAAQhAAAIQsEMAw2uHI1EgAAEIQAACEIAABDwlgOH1VBiGBQEIQAACEIAABCBghwCG1w5HokAAAhCAAAQgAAEIeEoAw+upMAwLAhCAAAQgAAEIQMAOAQyvHY5EgQAEIAABCEAAAhDwlACG11NhGBYEIAABCEAAAhCAgB0CGF47HIkCAQjkgMCFCxfUxo0b1czMzE2j3bt3r1q3bl0OrsLuEE+ePKleeuklNTw8rPr7+4Pg8nX45uNBwwAABOFJREFUqPbz6Eik3fr165uSo11ViAYBCNgmgOG1TZR4EICAtwS04d25cyemTCklPP7qr\/5KfeELX1BtbW0NG95oPG8TgYFBAAJNRwDD23SSc8EQaF4CGN5y7cfGxoIf9Pb2Bv9vdIVXYkRjNm+2ceUQgIBPBDC8PqnBWCAAgVQJ1DO8V69eVdu2bVNLlixRx44dUxs2bAj+vD81NaV6enrUlStX1MKFC9XBgwdVZ2dnMNbwZ2vXrg3arFmzJjCRUQMpZnBiYkKNj4+r1tbWYIU1XGKhyyr0OKSvM2fOBDGXLl2qjh49GqzERvvVY5Kf79ixQ+3bty9op+Ns2bLlphVt+ezxxx9XfX19pWuJY3iljbAJH5qT\/ExKJPbv31+6xlQFJTgEIACBmAQwvDFB0QwCEMg\/gWo1vAMDA4FB1QZRrjRqSnUZhBi6Xbt2BeZTDjGsmzZtCs6Xz7Zv3650vFqGV84Vc63NsRhnMZ+jo6NqxYoVwWezs7NBP2KOtREXAx417npMzz77bGB4tcGVmGEDHFaw0meVzKw+J2xq9c\/CY9a\/AMjYtm7dqnbv3l0y0vnPHK4AAhDIOwEMb94VZPwQgEBsAnFXeLUJrbRiGV41lc+1+Q2vqMZZ4T137txNZlQMZ0dHR7CaHDbD0k\/YPIdNt17x1RBkFXl6ejpYmQ5\/HYVUaSU2zgqvjlONZT3GscWiIQQgAAGLBDC8FmESCgIQ8JtAPTOmzWzY8IppHBoauunCZBVXzGlSw\/vNb34zWA2OHrKS+tRTT9U0vNHSiHCM8Mrtnj17qu6a0IjhrcRJj6FWGYXf2cHoIACBIhPA8BZZXa4NAhAoI5DE8NaqSY2utEaNYK2ShkorvFHTGDbecVd49Rg+8pGPqBMnTpR2YLC5wltrm7J6jElJCEAAAi4IYHhdUKdPCEDACYF6ZqzSymX0HP2Q2uDgoPrQhz5UthIbreGVldgjR46U1eHKhUt9sBzhsgXdj9QD1ytpqDQmXf8rtbR6VXrVqlVVHx6rVsMr46q1D2+t1WU5lxpeJ6lNpxCAQB0CGF5SBAIQaBoCSQyvwAnvxCDf64fStMHTOy3ceeedAcs\/\/uM\/Dh5iCz8kJzspdHd3q9dee63qLg36wbBKxju6qhodU\/jFGfqzRx55pLTlWFTkJLs0fOpTn6r44o6wsWaXhqa5nbhQCOSKAIY3V3IxWAhAwGcCtWpbsxx33FXWNPbM1Q\/e6b19s7xu+oIABCBQjQCGl9yAAAQgYImAL4Y37iqr7Tej2Y5nSRbCQAACEFAYXpIAAhCAgCUCPhheWWE9depU2csxal2emOOXXnrpprrdJEik7\/Xr1\/Pa5iTwOAcCEEiVAIY3VbwEhwAEIAABCEAAAhBwTQDD61oB+ocABCAAAQhAAAIQSJUAhjdVvASHAAQgAAEIQAACEHBNAMPrWgH6hwAEIAABCEAAAhBIlQCGN1W8BIcABCAAAQhAAAIQcE0Aw+taAfqHAAQgAAEIQAACEEiVAIY3VbwEhwAEIAABCEAAAhBwTQDD61oB+ocABCAAAQhAAAIQSJUAhjdVvASHAAQgAAEIQAACEHBNAMPrWgH6hwAEIAABCEAAAhBIlQCGN1W8BIcABCAAAQhAAAIQcE0Aw+taAfqHAAQgAAEIQAACEEiVwP8PIlKJZw5hZFoAAAAASUVORK5CYII=","height":337,"width":560}}
%---
%[output:97e357dd]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","2.443525700815186e-03","2.362998398567786e-03"]]}}
%---
%[output:9d1224cc]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000e+00","-1.901953846588630e+00","9.043571086383212e-01"]]}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ares_nom","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-2.526618726678876e+05","-2.513274122871834e+01"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-2.526618726678876e+01","9.974867258771282e-01"]]}}
%---
%[output:6e27c0a7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:7996ad92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:9ad8d2a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -2.526618726678876e+01"}}
%---
%[output:637faca4]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"     9.974867258771282e-01"}}
%---
%[output:61f9eea3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["1.490161953970131e-01"],["3.652194550246457e+01"]]}}
%---
%[output:306c88de]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:23fb1bfd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:11c23dfc]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-9.869604401089360e+00","9.984292036732051e-01"]]}}
%---
%[output:4b099645]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["1.555088363526948e-01"],["2.716608611399846e+01"]]}}
%---
%[output:00aa870f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["3.719106107041118e-02"],["1.937144673860303e+00"]]}}
%---
%[output:2e5b507a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGmCAYAAACeH1cjAAAAAXNSR0IArs4c6QAAIABJREFUeF7sfQ2YVlXV9kIRxYZfQUfAhPBv8mccRcwwtRcbzNIXfQ2BEIxQcMjePgQxosxess\/ASVJHUFJB5S8zFH8S9bVQNBAT8jPUULBwBEFBmEDwh+9aB\/fTmcN5nrPPOft35j7X5QUye6+1zn2vs5971rPO3i127969m3ABASAABIAAEAACQAAIAIEmikALCN4myixuCwgAASAABIAAEAACQCBAAIIXiQAEgAAQAAJAAAgAASDQpBGA4G3S9OLmgAAQAAJAAAgAASAABCB4kQNAAAgAASAABIAAEAACTRoBCN4mTS9uDggAASAABIAAEAACQACCFzkABIAAEAACQAAIAAEg0KQRgOBt0vTi5oAAEAACQAAIAAEgAAQgeJEDQAAIAAEgAASAABAAAk0aAQjeJk0vbg4IAAEgAASAABAAAkAAghc5AASAABAAAkAACAABINCkEYDgbdL04uaAABAAAkAACAABIAAEIHiRA0AACJREYNGiRTRq1Chq06YNzZo1iyorK40j1tDQQCNGjKBly5ZR\/\/79qba2thADxzdp0iSaP38+lZeXG41t5cqVNHToUNq2bRtNmzaNqqurA\/8i3jPOOINqamqMxpTkbMyYMbRgwYJG8SbNET9XcV91dXW0ePFimjFjBpWVlcm6VjYuzBkb7datW67cKZWbyoI2YEjgUlFRYY0bA7cJF80YAQjeZkw+bh0IyCDgsuBl8TRlypTcokUGh7gxcYJ3\/fr1NGDAAFq3bh2NHTvWKcEr4m3Xrl1qkafivoTY7t27txVRFRangs+8sTQVwRu+j\/Avb1mfDcwDAq4hAMHrGiOIBwgAAWkEIHiloQoGCryiVXIZK01N8GbBIA6npiJ48+aHTA5hDBCwiQAEr0304dt7BET1U9xI3Nf+4arWxRdfTFdddVXhvotVAMUcMTA6Lvohe8455wRtB8XGFwM6LGKKzY2r8IbvqaqqiqZPnx5MD8cpxJWwG\/3quJhYDWMqKk3R+\/3Zz35WaHEI31uxal2xymL4\/qNVLRluoxVejiXMg4gtbDvKLY+Jq6hFv3rnMatXr46taMt8TZ\/mXjmmsCCMYpH2vuLyLOpD5h5KLRhJfEXtyz4rcfPi2ldEu43Msxh9NqLPDv+\/zDMWziXO\/euvv54uvfTS2G8XktYU9inulf9uq33J+w8F3ICzCEDwOksNAnMdgTjhEvchWmpc9EM\/7itXYTMsQEqNy\/NBHuerlOANcxQW+8XuOTzGpOAt1pYh\/j0qxmW5TSt4S9kNi6hiAjPul4diY6O\/fCVhEPe8iZxLErxJ93XCCScU2jzCfpLsy\/aNy\/CVRfCW4kH8cifzLIa5jRO7suuGwKNnz56xv\/CFsZWJL1rlVlHFd33dRnzNFwEI3ubLPe48BwLhD8JwVVN88BYTf+EPmLix4sMwPD9OqEQ\/ZMUHavhDvVRvYnh+WOzFxZQkeKPV5zhswnEJDPIIXvHSmmxLQ9wHebGvotNwm6aHN656Fo5L4FKMm3BcgjNOYdEvHDc\/nG\/FsIqrfsflYTExJHtf0aqleGktCYOk1oM0fKVpPwjHJZ4lvgfx8qTggF+8E\/\/GPxfPYtx9xVXZwzGFn9mwiJd5xqJrgpgju6Zw7GnwybF8YioQsIIABK8V2OHUdwRkviIXHzhibLSKGBUQ\/LZ\/3E4E4Q+huKpNVNjKvBhUbHeBuB0PSgneuDfc4\/zHvd1vUvDGia033ngjdoeFNNymEbzRnI9W+oSwC9uMCp1oLv31r3+N3UEjrnJd7L7CwqqUuJSt\/hW7r2KCN6nynLSLQhq+0gi6YnFFd5koJliL3W84D6IV5DjBW+oZi\/4smjtp1hQRl8z64fv6jfibJwIQvM2Td9x1DgRKfWjG\/azYB0h07NVXXx37tW841KQqnsyHLI8pJnjjYEnq4Y1uLyX7gWla8EYrkUuWLNmrHzYtt2kFb6mvs+MEb7S3N4rZAw88ENxDsSvuK\/CoqI37qj+ulaCU4JW5r2K5WWouzynV1pCWLxWCN4p1KZtxz0KpNok4wRv3TY2syL\/wwgul1xRxX7LfmuRYQjEVCFhBAILXCuxw6jMCaT9kIXjj91o1LXjDvI0cOZJeeumlvfb1TcttGsEbFjpCxB1yyCF7tSSU+mVEh+AVz2KcEAtXEIsJXtn7guCdQeFvFRgP\/oXmy1\/+cuGbHQhenz8ZELvrCEDwus4Q4nMSAdkKCx86kLelIQ6AtFWlqI1iX5uLf588eXLhEIWsFd64F8Hq6+sL+6+aFryMgfDZtWtX2rp1awBL9G30NNymEbzCd1jUxPV5qmhpSFOFjMuvuBiKCV7Z+yomeIu1Dsg++Gn4ylLhFcJUHCrC8Y4bN66QN2mexeeeey5oQQk\/G0k9vKUqvFlbGkphG8enLBcYBwRcRgCC12V2EJuzCKR5UaZYj6TsS2txoirNh2zcaVbFXowKf70svk5PK3jjsIl7AUh8+DPJolc1un1VsW3J0r60JhIp+vV9nJhIw20WwRu3UwXHp+qltWLCslRvNW+plSTEkwRv0n0ViytO9BcbG7cgpOErjeCNy1l+lqLPbXjHhGi7SBTzcM5Hny++N9kKb9w9p3lprdS3CLItSc4uzggMCBRBAIIXqQEEMiKQtBWTqAiVGhcWOvz3YvuVRj8M8wpetldsm6aor7SCNyxW4qCN21GiGAVJgrfUSz9xNouJguhYWW6TfhkRdvk+uH1BHEMcF5vMvrfHHHMMvfrqq40qhKV6YOO2w4pWBUv1lIZFbBQ7voe091XshTbZeyiWJ7J8pRG87KsUNln66dm\/2FUj7l5kBW8cF2xPfHPBR10X+yUy7Df6C19afDIunZgGBKwgAMFrBXY4bSoIRD8Qkw6e+P73v09XXHEF8QcSX7IHT0QrRyoEbzGBHfWVRfCy7aiIicMmruIaPpwjSfBGP\/iT3ugPi5ikPV5luC2120XcQSBRm7KHSYhY4160i\/vlpRTWPD7axhH3i1YcltH4Rf7K3lfUT1hwRXMhiZ\/oGiLDVxZBF\/eLYfi5TfssRu2xrSOOOGKv3TZkKq3Rb4\/CL74W2+FD4Ba3I0epw0maypqN+2i+CEDwNl\/uceeGEJD54DIUCtx4jkCeN+ghZjwnXyJ82a3jipmK21NZwi2GAAEvEIDg\/Ywm\/iCZO3cuzZ8\/n8rLy4uSF\/dVUrEqnRcZgCC1IwDBqx3iJuUgLFqKvdyUdBhDMUCEYM46v0kB7fHNhL89CX\/+5H0BEPnhcVIg9EQEIHhDvYzt2rUrKXjFB1GXLl0Kb5qL34j79u1L4kWaRNQxoFkhAMHbrOhWcrNJfd\/R\/XllnYr1Kmmtk7WHcXYQKNV\/zxGVOmWxWMThYk7W\/LKDBrwCATkEmr3gLfa2ehx80e1oxBjZ6rAcJRjV1BCA4G1qjJq5n7gXudL2tcZFKvIRosYMj7q8FHvBNWv1XojoioqKQkFHV+ywCwRsINDsBa84JrKqqooeeeSRxJaGOJLYxvTp0\/d6EcQGofAJBIAAEAACQAAIAAEg0BiBZi14wxVbfvtZpoe3WMVk+fLlmcQyEhIIAAEgAASAABAAAkBALwLNVvCKr4MGDhxIfBpW1rYE8ZKAzItr69at08smrAMBIAAEgAAQAAJA4DME+MVXXHsQaLaCl\/vYosecpq3wpul5YrHLx1EuXboUuQcEgAAQAAJAAAgAAe0InHrqqcRHxUP4NlPBG\/fyWdoKbxqxyxn95z\/\/mQYPHhwkHp+Gg8ttBPgXk6lTp4Ivt2lqFB0484gsouCXfzxj4MwvBPyKVjxjixcvhuBtrhXepC1\/ktoTRBtDmq1fhOBF4vmxYIAvP3gKRwnO\/OIMfPnFV7hwg88xP7jDM9aYp2bb0hBNV9kKrxC7abd+QeL5sUCIKMGXX3zhwxh8+YeAfxFjXfSLM\/AFwRubsTKCN88hE0g8LBR+IeBftHjG\/OIMfPnFF36pBF\/+IQDBKy14o\/vrJrVClNrIHYu7X4\/K2rVr6e6776aJEydSy5Yt\/Qq+mUYLzvwiHnz5xRdHC8784gy6A4LXSsYi8azAntnphx9+SO+88w4ddthhELyZUTQ7EZyZxTuvN\/CVF0Hz88GZeczzeITugODNkz+Z5yLxMkNnZSIWdiuw53IKznLBZ3wy+DIOeW6H4Cw3hEYNzHlhPf2fG2bQ8psuxS4NzXWXBqMZ95kzCF4bqGf3iYU9O3a2ZoIzW8hn8wu+suFmcxY4s4l+et\/n3foSLXljC\/11zJEQvBC86RMo6wwI3qzI2ZmHhd0O7nm8grM86JmfC77MY57XIzjLi6DZ+RC8jfHGtmSG8g+C1xDQitxgYVcEpEEz4Mwg2ApcgS8FIBo2Ac4MA57THQQvBG\/OFMo2HYI3G262ZmFht4V8dr\/gLDt2NmaCLxuo5\/MJzvLhZ3o2BC8Er+mcC\/xB8FqBPbNTLOyZobM2EZxZgz6TY\/CVCTark8CZVfhTO4fgheBNnTQqJkDwqkDRnA0s7OawVuUJnKlC0owd8GUGZ5VewJlKNPXbguCF4NWfZTEeIHitwJ7ZKRb2zNBZmwjOrEGfyTH4ygSb1UngzCr8qZ2fOOl5+sf7H2KXhs+Qw0trqVMo2wQI3my42ZqFhd0W8tn9grPs2NmYCb5soJ7PJzjLh5\/p2RC8qPCazrnAHwSvFdgzO8XCnhk6axPBmTXoMzkGX5lgszoJnFmFP7VzCF4I3tRJo2ICBK8KFM3ZwMJuDmtVnsCZKiTN2AFfZnBW6QWcqURTvy0IXghe\/VkW4wGC1wrsmZ1iYc8MnbWJ4Mwa9Jkcg69MsFmdBM6swp\/aOQQvBG\/qpFExAYJXBYrmbGBhN4e1Kk\/gTBWSZuyALzM4q\/QCzlSiqd9WxzFPB05wtPAerPHSmv6cCzxA8BoCWpEbLOyKgDRoBpwZBFuBK\/ClAETDJsCZYcBzuoPgRYU3Zwplmw7Bmw03W7OwsNtCPrtfcJYdOxszwZcN1PP5BGf58DM9G4IXgtd0zqHCawXxfE6xsOfDz8ZscGYD9ew+wVd27GzNBGe2kM\/mF4IXgjdb5uSchQpvTgANT8fCbhhwBe7AmQIQDZoAXwbBVuQKnCkC0pAZCF4IXkOp1tgNBK8V2DM7xcKeGTprE8GZNegzOQZfmWCzOgmcWYU\/lXM+YY13aeALL63tgQ4vraVKoeyDIXizY2djJhZ2G6jn8wnO8uFnejb4Mo14fn\/gLD+GpixA8O6NNASvoeyD4DUEtCI3WNgVAWnQDDgzCLYCV+BLAYiGTYAzw4DncAfBC8GbI33yTYXgzYef6dlY2E0jnt8fOMuPoUkL4Msk2mp8gTM1OJqwAsELwWsiz2J9QPBagz6TYyzsmWCzOgmcWYU\/tXPwlRoy6xPAmXUKpAOA4IXglU4W1QMheFUjqtceFna9+OqwDs50oKrPJvjSh60uy+BMF7Lq7T67egudX\/dSYBgvre3BFz286vMMFV5DmOp0g4VdJ7p6bIMzPbjqsgq+dCGrzy4404etassQvKjwqs4paXuo8EpD5cRALOxO0JAqCHCWCi7rg8GXdQpSBwDOUkNmbQIELwSv8uSrq6ujuXPn0vz586m8vLyofQhe5dBrNYiFXSu8WoyDMy2wajMKvrRBq80wONMGrXLDc15YT6PnrArsoqVhD7xoaciRZitXrqShQ4dSu3btIHhz4OjiVCzsLrJSOiZw5hdn4MsvvjhacOYPZzc8voZueHwtBG+IMgjejPnb0NBAI0aMoGXLllG3bt0geDPi6Oo0LOyuMlM8LnDmF2fgyy++IHj94ouru1zlRYX337xB8GbMYW5lWLx4MVVVVdEjjzwCwZsRR1en4cPYVWYgeP1jJj5iPGP+MQnO\/OHsvFtfoiVvbIHgRYU3X9IuWrSIxo0bR7NmzaIlS5ak6uGdPXt2UBEWV6m+33xRYnYeBLCw50HPzlxwZgf3rF7BV1bk7M0DZ\/awl\/G8fv2eii5fVzy4EYI3AhoqvDJZFBrDCTVgwAAaOHAg1dTUUNqX1qLuuAd42LBhKaPAcN0I7Ny5kzZu3Bi8iNiyZUvd7mBfAQLgTAGIBk2AL4NgK3IFzhQBqcnMzJkzg0IcX1v6\/6bgBS+t7YECgjdl4o0ZM4bq6+tpxowZVFZWllrwTp48mbp27VrwyoIKVd6UJBgYvmPHDtqwYUNQjYfgNQC4AhfgTAGIBk2AL4NgK3IFzhQBqckMF+T4v39u\/pBGPrEbgjeCMwRvisQLtzJUVlYGM9NWeLnvN9zSkMI9hhpEAF\/dGQRbkStwpghIQ2bAlyGgFboBZwrB1GgqfKwwu0GFFxXe1OnG1d0FCxYUnTd27NigzSHuwj68qeG2OgELu1X4MzkHZ5lgszYJfFmDPrNjcJYZOqMTw1uS7bN9E62YeBoKbWhpyJ+DqPDmx9BFC1jYXWSldEzgzC\/OwJdffHG04MwPzsJbkrXc9Br95fpvQvBC8OZPXgje\/Bi6aAELu4usQPD6x0rxiPGM+ccmOPODs\/CWZAe8+hAtu\/0qCF4I3vzJC8GbH0MXLWBhd5EVCF7\/WIHgBWdNCQH37yXav1v27C\/puQfugOCF4DWXvOjhNYe1Ck8QvCpQNGsDnJnFO6838JUXQfPzwZl5zNN6jAretovG07N\/eACCF4I3bSplHw\/Bmx07GzOxsNtAPZ9PcJYPP9OzwZdpxPP7A2f5MdRtIdzOwC+sseDF7lB7UMe2ZLqz7zP7ELyGgFbkBgu7IiANmgFnBsFW4Ap8KQDRsAlwZhjwlO6i1d0z222glTMnQPB+hiMEb8qEyjocgjcrcnbmYWG3g3ser+AsD3rm54Iv85jn9QjO8iKod\/6zq7fQ+XUvFZz86OSddOuPayB4IXj1Jl7UOgSvWbzzesPCnhdB8\/PBmXnM83gEX3nQszMXnNnBXdZruJ3h8x0PoGlnt6DBgwdD8ELwyqaQmnEQvGpwNGUFC7sppNX5AWfqsDRhCXyZQFmtD3CmFk+V1qLtDLcOqqAen6yF4A2BjJYGlRlXwhYEryGgFbnBwq4ISINmwJlBsBW4Al8KQDRsApwZBjyFu2h1l09Xg+5oDCAEb4qEyjMUiZcHPfNzsbCbxzyvR3CWF0Gz88GXWbxVeANnKlBUbyOuujvolHII3gjUELzqcy\/WIgSvIaAVucHCrghIg2bAmUGwFbgCXwpANGwCnBkGXMIdi11+UY3\/5It7d7m6yxd0Byq8EimkfggSTz2mOi1iYdeJrh7b4EwPrrqsgi9dyOqzC870YZvV8tUPvE4znn27MP2hmio6\/Yj2ELwxgKLCmzXLUs6D4E0JmOXhWNgtE5DBPTjLAJrFKeDLIvgZXYOzjMBpmjbnhfU0es6qgvX\/OLoj3T+ysvD\/0B2o8GpKvdJmkXhWYM\/sFAt7ZuisTQRn1qDP5Bh8ZYLN6iRwZhX+Rs6jYpdbGbi6y3+KC7oDgtdKxiLxrMCe2SkW9szQWZsIzqxBn8kx+MoEm9VJ4Mwq\/AXn0ZfU4sQuD4bugOC1krFIPCuwZ3aKhT0zdNYmgjNr0GdyDL4ywWZ1EjizCn\/gPFrZ5X\/jPXd5V4boBd0BwWslY5F4VmDP7BQLe2borE0EZ9agz+QYfGWCzeokcGYV\/lRiFxXevbnCS2uG8heC1xDQitxgYVcEpEEz4Mwg2ApcgS8FIBo2Ac4MA\/6ZO25heOrV9+iq+19vFECxyq4YBN2BCq+VjEXiWYE9s1Ms7JmhszYRnFmDPpNj8JUJNquTwJl5+KP77HIE3LM7vl+P2DaGcITQHRC85jMWzeNWMM\/jFAt7HvTszAVndnDP6hV8ZUXO3jxwZg57FrpL3tjSaNsxIXZvGVhR2Gu3VEQQvBC85jI25AmJZwX2zE6xsGeGztpEcGYN+kyOwVcm2KxOAmdm4I+r6gqxG916DIJXnhP08MpjlWskBG8u+IxPxsJuHPLcDsFZbgiNGgBfRuFW4gycKYGxqBEWur98fA3NfmF9ozHcwiBb1Q1PhO5AhVdvxhaxjsSzAntmp1jYM0NnbSI4swZ9JsfgKxNsVieBMz3ws9CtffItmvXn+r0cFNtjVyYS6A4IXpk8UT4GiaccUq0GsbBrhVeLcXCmBVZtRsGXNmi1GQZnaqFloXvD42uC7caiV9aqLiq8xTlCS4Pa\/C1qDYLXENCK3GBhVwSkQTPgzCDYClyBLwUgGjYBzvIDziJXCF1+KU2H0BU2oTtQ4c2fsRksIPEygGZxChZ2i+BndA3OMgJnaRr4sgR8DrfgLDt4Quh+b+6qQPDGCd3rzutJVYe1DbYdU3FBd0Dwqsij1DaQeKkhszoBC7tV+DM5B2eZYLM2CXxZgz6zY3CWHrpnV28J2hbiqrlsTbQu8J+qhC4qvPE8oaUhff5mmgHBmwk2a5OwsFuDPrNjcJYZOisTwZcV2HM5BWfJ8Inq7czn6+lXT71VdILs4RHJHouPgO5AhbeAwJgxY2jBggWF\/582bRpVV1cn5lddXR1NmTKlMG7s2LFUU1NTch4SLxFWpwZgYXeKDqlgwJkUTM4MAl\/OUCEdCDiLh0qI3PuWvUOTF60tKXLPPKoDXXV2d+XV3Din0B0QvAECLHaXL19O8+fPp\/Lyclq0aBGNGjWKksQri93p06fTrFmzqLKyklauXElDhw6lkSNHlhS9SDzpNdWJgVjYnaAhVRDgLBVc1geDL+sUpA4AnDWGjNsVFr68ke54Zl1JLHW2LZRyDN0BwVsQqZMnT25U0WURXF9fTzNmzKCysrK98mj9+vU0YMAA6tWrF9XW1jaqFIfFM37TSr2OOjcBC7tzlCQGBM4SIXJqAPhyig6pYJo7Z0k7LIRBZJFbc+ZhdM6xnYxUc6E7klMYPbwhjCB4kxOmuYxo7gu7jzyDM79YA19+8cXRNkfOuIr74j+20pOr3iv64plgkkXu4N6H0sBe5dZEbjirUOFFhTd2lRF9uUl9vMVaGvr27duo6ht1IhJv9uzZ1K1bt8KPuZ0Cl3sINMeF3T0W0kUEztLhZXs0+LLNQHr\/zYGz+q0f05vvbqMbn1qXKHAZwS5tW9KAkw+hIad2sS5y+Vvo8LVu3ToaPHgwLV68uJHuSM9805jR7Cu8oneX6ezfv39J0SooF32727ZtC\/4pSSTzGCF4o2nD\/b\/Dhg1rGtnUhO5i586dtHHjxqC\/u2XLlk3ozprurYAzv7gFX37xxdE2Nc5Y3PL1zraPafrSLfTi23vvjxtliQXuoW1a0nkVZXRy1wMCwevKNXPmzOD9ougFwbsHkWYveEViNDQ00IgRI4IeXvEiWzRp4saIvt4uXboU7f0NC17uG+7atWvBNAsqVHldWS7+HceOHTtow4YNwW\/FELzu8RMXETjzgycRJfjyiy+Otilwxn24i\/++meYtXy8lcPm+uVXhe2d1oyM6taYvdd\/7\/R5XmGQ9Eq7yLl26lKZOnYoK72cEQfCGMjVpxwVRDY5WdMW86Etw4YcAvTSuLAlycTSHr+7kkPBnFDjzhyuOFHz5xZePnImXzFau20Z\/eGWTVIuCELh9eran8f16BCSpPhDCFPPQHY2RdlLwfvDBB3TbbbcR\/5n1ateuHV1zzTWppssI3nHjxhW2JBPGRZV34MCBRbcmQ+KlosL6YHwYW6cgdQDgLDVkVieAL6vwZ3LuOmf8ghlfpU42i7txFrQscAedcmjw49OPaJ8JH9cmQXd4IHiFgOSG66wXfxXNfStxF1dq44RrsQqusIEKb1Y2\/Jvn+sLuH6L6IwZn+jFW6QF8qUTTjC2XOBPV2zkvvBNUbsXhDzJIsMDt98VOdN4JnYPqra8V3KR7heD1SPBOnDhR6uSzKOksTCdNmlRU8IpeXJ4n9twV1d2KioqivbgqenjRPJ70iLrxc5cWdjcQcT8KcOY+R+EIwZdffHG0tjgT4nbuC+vprfd3SLcmcMxCzA47rQv9n76HB8K4qQrcaEZB8HogePnt+O9973vBf1\/5yldSrwrPPPMM3XLLLTRv3rySc6NHC0dPWYtuQSaMRefJ7O6AxEtNo9UJthZ2qzftuXNw5heB4MsvvkwIXiFs2VeWyq0QuKI9oSlXb2WyB7rDA8H7ySefEP\/XqlUrGU69GIPE84KmQpD4MPaLLxMfxv4h4nbEeMbc5icuOlWciSor99y+u20X3fXc26mqtiI2FrSHdTiAru7XndZt3hn04TaX6q1M9kB3eCB4uYeXN0s+7LDDgpfA+CjffffdV4ZfZ8cg8ZylJjYwVQu7X3ftd7TgzC\/+wJdffOX5pVK8TMZVWxa73HOb9gqL2xbUokn33qbFpth46A4PBC\/vzvA\/\/\/M\/9PDDD9OuXbuoc+fOdOmllxLvgtChQwdVuWDUDhLPKNy5neHDODeExg2AM+OQ53IIvnLBZ2VyEmeiJeGBlzbQ39\/dnknY8o2JXRNOP6JDUMFtKrsmmCYNusMDwStC5JfEnn76abrzzjvpr3\/9a\/DPJ5xwAl1xxRV01llnedXygMQz\/ajn85e0sOezjtk6EABnOlDVZxN86cNWl2XB2Vs729AXDi6jqf\/7D3p9w79yCVuO9evHdaJvHNe0d0zQxUkpu9AdHgnecKj8Itv9999Pd999d3Dka1lZGX3rW9+i4cOHNzq5zEZSyfhE4smg5M4YfBi7w4VsJOBMFik3xoEvN3goFYVoRZi7fD299V663RGidkVLwoCTD6EenQ5ES4IB+qE7PBW8Iuzdu3fT66+\/TnfccQf94Q9\/oO3bt1OPHj2Cqu\/555\/vbNUXiWfg6VboAh\/GCsE0ZAqcGQJakRvwpQjInGZEG8Kqdxroob9upH9u\/jDVnrbFhG3fYw6iXoe3hbDNyU+e6dAdngvecPjc3\/v888\/Tj3\/84+Cf58+fT+Xl5XnyQ9tcJJ42aLUYxoexFli1GgVnWuFVbhx8KYe0pEGu1nKV9aYpZhyBAAAgAElEQVSn3srVXyuciIrtGUd2oNO+sGd3BOyQYJbTJG\/QHU1A8Aqhy+0NLHh5C7Pq6mq6\/vrriY8UdvFC4rnISvGY8GHsF18cLTjzizPwpY4vsc2XqNY+98YWemb15tzVWo5QCNthX+pC7ff\/lFp\/0kCnHtuDWrZsqe4GYEkLAtAdngpeFrUrVqygOXPmFFoZfNq9AYmn5XnWZhQfxtqg1WYYnGmDVoth8JUO1qiofe9fu2jGs28rE7UcDe9j26dnh0K1NlqxBWfpOLM9GrrDI8HL\/brr1q0LXlR76KGH6L333gt6dM8991waOXIkHXXUUdSiRQvbOSXlH4knBZMzg7CwO0OFdCDgTBoqJwaCr71pYFErLv77poZd9JslakRtuFrL23wJYSv+XSYpwJkMSu6Mge7wQPDu2LGD7rnnHpo1axbV19cHoraioiLYi\/ecc84Jdmjw7ULi+cUYFna\/+OJowZlfnDVXvoSoFX++Ut9AD7+c\/2Uxwb5oQTi6\/HPUv\/Lg4J9V9dc2V878erL+HS10hweCl09aGzBgAG3ZssWrrcdKPRRIPL+WDCzsfvEFwQu+XENA9NP+4\/0dtHLdNvrbO9n3q43em2g1OOuojnRqj3bB4QyqRG0pHLEuupZlpeOB7vBA8HKFd+3atdSzZ09ntxlLm\/ZIvLSI2R2Phd0u\/lm8g7MsqNmb0xT4EvvU8gti\/KIYX1mOzY1jQYja6oqDqOrzbY2JWghee8+Eas\/QHR4IXlHhnThxYrD7Qtpr0aJFNGnSJFq8eHHaqdrGI\/G0QavFcFP4MNYCjMNGwZnD5MSE5gNfou3giVXv0V\/+sTXYn1aVoGVIRPvBmUd1oK7tD3BC1ELw+vUclYoWugOC10o2I\/GswJ7ZqQ8fxplvrolOBGd+EWuTr+jLYYzcb19cT29u2qFk1wPBhKjScsvBd77chXZ+vNt5UQvB69dzBMErz1eL3bwVgmOXqPDyDg1Zr27duqHCmxU8zMMLUB7mgE0B5SFc1kPWzVf05TBRpc17klgUOFGlPeLgA6l393\/304oKrnWgFQagmzOFocIUEaHQ5kGF94MPPqDbbruN+M+sFx9Acc0112SdrnweEk85pFoNYmHXCq8W4+BMC6zajKrii\/tod33yKT316vv013XblFZow6KV96g9rksZHd+1TdCKwIKat\/dqTpcqzpoTZjbvFbrDA8FrM0F0+Ubi6UJWj10s7Hpw1WkVnOlEV71tGb7EcbiP\/20TrfjnNuU9tGFBy20HfY\/pSL0Ob1c4IhdH5TbmXYYz9ZkCi1kRgO6A4M2aO7nmIfFywWd8MhZ245DndgjOckNozABXRz\/++GP6y+v\/pE6dOtO6Dz6iecvXB\/5VvhQWFbQXnXQI9ex8YODHxDZexgA15AjPmCGgFbmB7oDgVZRK6cwg8dLhZXs0FnbbDKT3D87SY6ZzhtiH9s1N2+n1DduDvWhV989GBS23HZx+RIfAD\/89\/HOd99pcbOMZ84tp6A4IXisZi8SzAntmp1jYM0NnbSI40w89i1jRvyr+PuvP9bR0zZ73LVRXZ8OClQXssYeW0TdP6By0NqBCq5\/vqAc8Y+Yxz+MRugOCN0\/+ZJ6LxMsMnZWJWNitwJ7LKTjLBV9hcnh3g4adn9Cj\/28jrVG8XVc4UrHLQWW3NnTOsZ0aVWXRQ6uGU1VW8IypQtKMHegOCF4zmRbxgsSzAntmp1jYM0NnbSI4k4NetBr8\/d3t9NI\/t2oXsxwVvxD2pR7t6MyjOhbaDZiv\/XZupsMOO4xatmwpFzxGWUUAz5hV+FM7h+7wWPDylsGbN2+mTz75hDp27Ej77LMPffTRR14cP4zES\/2sWp2Ahd0q\/JmcN2fOoq0GDODvV2wIemd19M0KgkR19siDD6Sqw9pSj06tpXc4aM58ZUpwByaBMwdISBECdIeHgpeF7hNPPEE\/\/vGPaePGjcSHSsyfP5\/2339\/+u53v0u9evWisWPHOi18kXgpnlIHhmJhd4CElCE0dc5EZfat93fQ6ne30\/K3tmoXs0zBV4\/uGGzX1b71fgEjqnpnmzpfKdPXi+HgzAuaCkFCd3goeP\/0pz\/RyJEj6aSTTqIjjzySnn766UDwlpWV0Q9\/+EN6\/PHHaeLEiTR06FBnsxGJ5yw1sYFhYfeLL47WV86EkOV74B0NWMjq7JkVopX\/5FaDsysOopM\/37YgZsM\/15kFvvKlExPXbYMz1xlqHB90h2eCd+fOnTRq1Cj69NNPadq0afTMM8\/QpEmTAsFbXl5Ou3btojFjxtCmTZtoxowZgQiWvXjeggULCsPZfnV1deL0RYsWBTGJq3fv3om+kXiJsDo1AAu7U3RIBeMiZ2Ex27DzY3r8lfdo9Ub9bQZCzPKOBm0PaBkIW\/ECmCsvgrnIl1SiNeNB4Mwv8qE7PBO869evpwEDBtDll19OQ4YMIRabYcHLt3PvvffS7bffXhDBMinJYnf58uWFOULEcmtETU1NURN1dXU0ZcqUQHyzOBbxdenSpaToReLJsOLOGCzs7nAhG4kpzuK25rpv2Tv03BtbglB198wKMVt1WBuq\/mInb\/ecNcWXbP5gXDIC4CwZI5dGQHd4KngvueQSuuyyy2IFL4vQ+++\/n+bNm0edO3dOzLeVK1cG7Q+TJ09uVNFlEVxfX19UuApxO3DgwEaimMXyuHHjaNasWVRZWRnrH4mXSItTA7CwO0WHVDCqOAtXZdnxs6s3F\/aX1bHPrLg5UXnlauyXe7an7ge1drIyK0WGxCBVfEm4whBFCIAzRUAaMgPd4ZngFS0N27dvpzvuuIOYwHCFd926dXTppZcGW9tw1ZVfZMt6JQneuOqyrC8knixSbozDwu4GD2miKMVZXFX29yvepdc3\/Cs4xEBnVZbvQYhZPjzhrKM60sef7m7SYlaGNzxjMii5NQacucVHUjTQHZ4JXg73pZdeCloa+IU1\/u\/RRx+lH\/zgB7R69Wr63e9+F\/TxTp8+nc4888wk\/ov+PNqqEDeQxyxevJiuv\/76QGSz2Oarf\/\/+VFtbW9K3SLzZs2cHu0yIi\/uQcbmHABZ29zgpFVH91o\/pzXcbaP36d6hTp060fN0OWvKGvtO\/RCxd2u7ZP5arsrwl1\/Fdy6iifM97BK71zLrGKJ4x1xhJjgecJWNkcwR\/Cx2+WKMMHjw40C1h3WEzRpu+W+zmPb88uJ577rlgW7I1a9Y0ipZbGH7+85\/T2Wefnekuwi+gJQlX8ZJbmzZtCu0LaXt4o0Fya8WwYcMyxY5J+hDgbxZ4Czz+hQSb4uvDOUnEsqBkMfvOto\/p0DYtaeGqhuDv4t\/4T12XELPs90ufb02Vh+759oj\/ny\/xc13+m7pdPGP+MQzO3OZs5syZgTaJXhC8exDxRvBysKzNeTeGv\/3tb8QP3rHHHhsIkn333Td3FjY0NNCIESOCHl6xA0TUqBC80d0chGgutcuDqPBy33DXrl0Lpjl+VHlz06fcwI4dO2jDhg3Bb8UQvMrhDQzu6ZXdEfz9X7s+oSdXvU+vrv9XQdDq8brHqjgwgf88otMB9J+VnYN4RFUWYlYn+nts4xnTj7FqD+BMNaJq7XEBLlzlXbp0KU2dOhUV3s9g9krwqk2Nva2Jl9l4z9+4nRpY8D711FN7vZxW7GW2sAf00uhmT619fHWXHk8WjOISf\/90925asOJd4mNs+dL50hfbZ6HKv6BwiwH3yp7ao10hJlUHJqRHBjPiEMAz5l9egDO\/OIPuaMyX84L3gw8+oNtuu434z6SLe\/e4teG4447LVPVNErzcw8u9wtHdGCB4k5jx7+dY2PfmLLx7AVdm323YRU+tet+IkA3vYNC9U2s69tAyOve4To2qsgcfSPTOO+8EL7CiKu\/+M4dnzH2OohGCM784g+D1TPByH+X3vvc9euWVV4h3auCLhS1f3N4Qd33lK1+hm2++mdq23XN6UPQqto1YUmtCMUGMbcn8WgRkom0uC3t09wLG5pX6Bnr45Y0BTLp3L2AfYTH7rZMPoS90OjDwnfalr+bCmUz++jAGfPnAUuMYwZlfnEHweiZ4Odxly5bR6NGjg71zv\/Od7xROU+PdGX77298SV15vueUW+uIXv0gPPfRQsG0Zj73qqqtis1P06\/IPxelsQsxWVFSUPEAi2tYgqru9evUquVMDEs+vhaIpLezPrt5zIML\/vvY+LX\/rA+LXVE0L2ZM+35aOPuRAOv2IDo2qsipP\/WpKnPn1tGSLFnxlw83mLHBmE\/30vqE7PBO8QpweddRRdN1111GLFi0a3QG\/yHbttdfSW2+9FezD27p1a7rhhhuC\/Xp\/\/\/vfl8yQ6NHC0VPWirUwiC3MhPGk3R14HBIv\/cNqc4bLC7vojxV\/vrlpO\/32xQ1WKrJHHHxg8MJXy332yVSVVcmxy5ypvM+mYgt8+cckOPOLM+gOzwSvqKByhffiiy+OzTY+Ye3WW28t7K7wwAMP0E033RS8mejKhcRzhQm5OGwt7GEx+\/Rr79OytXt6101XZCsO\/VzQJ9uz84GpWwvkEFY\/yhZn6u+keVgEX\/7xDM784gy6wzPBu3nzZho+fDj17NkzOPChVatWje6A2xomTJhAb7zxBt15553UoUOHoH+XD6d47LHHnMlOJJ4zVEgFonphDwvZNZu205\/XfGDkhC++2UYvfB3Umo7rWhaI2fDPVLYWSAGsYZBqzjSECJMhBMCXf+kAzvziDLrDM8HL4d5+++00ZcoU+ta3vhW8wCb2reXqL\/fuzp07N+jXveKKK4J+3x\/+8IfBHr0sfF25kHiuMCEXR9LCbvuo2qiQPbZLGXFVtmfoha+mIGLl2NozKomzNLYwVj8C4Es\/xqo9gDPViOq1B93hoeDlKi4L3rvuuos++eSTRnfAh07wi2zcf8sP42WXXRYcHsEimV9Ac+VC4rnChFwcnEsPL19DnTp1pre3fkRvbtxRaC\/QvZdsuCLLx9VyNZYFbVjkNjcxK8MaPoxlUHJnDPhyhwvZSMCZLFJujIPu8FDwipB5izI++IF3VOCLq7hf\/epXCyeX8SkwXPXt0qUL7b\/\/nmNAXbmQeG4wEa3Mrtm0g1546wPiP3X3yYaF7OEHtaZvHNeJtuz4ODgkIe0WXG6g6VYU+DB2i4+kaMBXEkLu\/RycucdJqYigOzwWvH6lWuNokXjm2Av3y25q2EV3Pvd2sBWXzsqsOKqWK7JHHnwgdSprBSFrjvLAEz6MDQOe0x34ygmghengzALoOVxCd3goeHnrsTfffJMWLFgQe9jEzp076e233w7OjBb9vTlyRMtUJJ5aWMOnfvH+si+s\/UCLoA1XZU\/o2oa+flynoBLcp2f74IbQWqCW1zzW8GGcBz3zc8GXeczzegRneRE0Ox+6w0PByzsu8J653Msbd\/HODXzwA7+kxrs0uHgh8bKxIoTt395poIV\/3ai07SAsZr\/csz11P6h1oSrLC\/t+OzfjmNpstFmZhQ9jK7Bndgq+MkNnbSI4swZ9JsfQHZ4J3n\/96180cuTI4EW0X\/\/613T44YcHL6ZVVVUFp6\/NmTOH7r777uDQieOPPz5TUpiYhMRLRlm0Itz13Nu0\/K2tuSu2os2AT\/jik74+37G1dK8sFvZkvlwbAc5cY6R0PODLL744WnDmF2fQHZ4JXnHwBG9JduWVVwbR\/\/SnP6V33303aGHgi6u\/\/JLa5MmT9zqJzZX0ROI1ZkJUbrfs+IimL16XWdwKUXtxr3Lad58WhQpt3lYDLOyuPDnycYAzeaxcGAm+XGAhXQzgLB1etkdDd3gqeH\/wgx\/QhRdeGER\/77330sKFC+mOO+6gtm3bBv9\/33330T333EOdOnWynWOx\/pt74onq7Z\/+vpnmL1+fSuAK8drvi53ovBM6B\/jyv+UVtaUSBQu7k49RyaDAmV+cgS+\/+EKF1z++mrvuiDLWYje\/EebwtXXr1qCF4aSTTqLx48cHkT755JNBlZeFbvfu3QPhy2J3\/vz5eGnNMS6fXb2F5rzwDs15Yb1UZKJie2HVIcFuB7qFbbGg8GEsRZdTg8CZU3QkBgO+EiFybgA4c46SkgFB8HpW4eVwb7zxxuDQiauvvpoGDhwY9PMOGTIkOHnt61\/\/Ol1zzTXEul0cLexiSjaXxBOtCvf\/ZQPN+nN9IhUsaL9+bCf6xvGdrYnbuCCxsCdS59wAcOYcJajI+0VJYrR4xhIhcmpAc9EdsqA7X+HlG+EqL\/fv8p8satu3b0+TJk0KXlZjoduiRQu69tpraejQobL3bXxcc0i82cveoV8uWkuifSEOZBa4g085lA7reAANOqXcOA+yDrGwyyLlzjhw5g4XMpGALxmU3BoDztziIyma5qA7kjAI\/9wLwcsBs7Ddtm0btWnTJhC4fMTwc889R8888wz169cvaHngf3f1aqqJx+K27k\/\/pNufWVcUeha5twyscKqCm5QnWNiTEHLv5+DMPU5KRQS+\/OKLowVnfnHWVHVHVha8EbylbpDF7wcffEDt2rWjfffdNysWWuc1tcRjoXvD42uK9uayyB3fr0dwQIPOl8t0kYaFXRey+uyCM33Y6rAMvnSgqtcmONOLr2rrTU135MXHecErtiWbOHEiVVdXx97vI488QjfccANeWsubDRLzWeg+8NIG+tkjb+41moXt6Ud0oKuru3spcsM3hIVdIhkcGwLOHCMkIRzw5RdfqPD6xxcEb2POnBS8fFQwtyts376dtmzZQjfddBNdcMEFVFlZuVfG8Vju6+Wx8+bNo86d92xb5drle+KJl9HOr3spVuiOOftwOuuojt4LXXFz+DB27QlKjgecJWPk0gjw5RIbcrGAMzmcXBnlu+5QjaOTgpdvctasWXTdddcFvbtJF7cxjBs3Lti+zNU+Xp8Tj8UuC93oy2hc0b354mPoK0e6eZxzUt6U+jkW9jzo2ZkLzuzgntUr+MqKnL154Mwe9lk8+6w7stxv0hxnBe+uXbuCvtwNGzbQqFGj6L\/\/+7\/pK1\/5yl7306pVK+rQoYOzQlcE7Gvi8f65o+es2gv3O4ce+9lxvQck5ZiXP8fC7h9t4MwvzsCXX3xxtODML8581R26UHZW8Iob5hfS3n\/\/\/eBENT4+2NfLt8Tjai6LXX4xLXxxVfehmqom07pQLJ+wsPv3pIEzvzgDX37xBcHrH1++6Q7dCDspeIXI5T9lL25r6NixI3ZpkAWsxLi4FgaxtdjpR7RX4MF9E\/gwdp+jaITgzC\/OwJdffEHw+scXBG9jzpwUvGJnhnXriu\/tGk29bt26YZcGBc8ji90TJz3fLKu64ZvGh7GCZDJsApwZBjynO\/CVE0AL08GZBdBzuITg9UDw7tixg5YsWUK8A4Psxe0Offr0odatW8tOMTrOh8R7dvWW4OW08MX76C4cXWUUKxecYWF3gYV0MYCzdHjZHg2+bDOQ3j84S4+ZzRk+6A6T+DhZ4TUJgClfridenNgd3697cHhEc7ywsPvHOjjzizPw5RdfHC0484sz13WHaTS9ErybN2+mZ599ll588UXi3RmqqqroS1\/6UrBLg+uXy4kXtxNDcxa7WNhdf5ri48OHsV+8gS+\/+MK66B9fLusOG2h6IXj55bXbb7+damtrKfoiG7+sNnr0aKqpqQlEcJprzJgxtGDBgsKUadOmFT3NrZjduro6mjt3bmL\/sKuJF9eze+ugChp0SnkaKJvcWHwY+0cpOPOLM\/DlF18QvP7x5arusIWkF4L3\/vvvp\/Hjx9MZZ5xBV111FXXv3j3Aa+3atXTjjTcGp7JNnjyZzj\/\/fGkcWewuX768IFQXLVoU7Pc7duzYQDzLXCtXrqShQ4dSu3btvBS8ELvFWcaHscwT4NYYcOYWH0nRgK8khNz7OThzj5NSEUHwNkbHecHb0NBAI0aMoLKyMrr55pv3eimNX3C78sorg8ovV2hl9uoVQpVFcnV1dQERFsH19fU0Y8aMwF+pS8S1bNkyktkhwrXEi9t6DJXdfzOOhd2vhR3VJ\/DlHwL+RYx10S\/OXNMdttFzXvCKLcouv\/xyGjJkSCxe9957b9DyMH\/+fCovz\/5VfBrBy60MixcvDvqIH3nkkUTfLiUei93\/nv8q\/en1zQU8IXYbpxYWdttLU3r\/4Cw9ZjZngC+b6GfzDc6y4WZrlku6wxYGYb\/OC96NGzfSxRdfTBdccEFQyY27uPL7+9\/\/nubNm0edO3fOhCsL2ClTpgRV4nDVN84Ytz+MGzeOZs2aFWyflqaHd\/bs2UFFWFx5BHqmG\/1s0sFXP1OYfurhZc1y67FS+GFhz5NdduaCMzu4Z\/UKvrIiZ28eOLOHvYxnLhCGLz7LYPDgwUFxLqw7ZGw1xTHOC96PPvqIuPK6atUquuOOO6hHj8bbZK1Zs4Yuu+wyqqioCF5q22+\/\/VLxJHp3eVL\/\/v0DG6UuUXEeOHBg0Oub9qW1qG3uAR42bFiqmPMMfvHtD+nyB\/79UHRp25IWDvu3AM9juynN5T2g+Zct\/oWkZcuWTenWmuy9gDO\/qAVffvHF0YIztzmbOXNmUIiLXhC8exBxXvBykC+\/\/DINHz48eNi+9rWvBQdM8MXV1SeeeCLo2+WWBm4vyHqJnlzu4S3VGhFte0greLlvuGvXroUwWVCZrPL2+r8vUP3WjwP\/fFzwA5cfTyx6cTVGgHvDN2zYEPxWDMHrR3aAMz94ElGCL7\/44mjBmduccUEuXOVdunQpTZ06FRXez2jzQvByrK+99hpNmDCBVqxYQbt3796j1lu0oBNPPJGuv\/56Ovroo3NnoniZbeTIkbE7NYRbGSorKwN\/aQWvzd+07lzyNo393esFnNC3Wzxl8NVd7sfJuAFwZhzyXA7BVy74rEwGZ1Zgz+wUPbyNofNG8IqwucrLB1DwxQdOyOzKIJstSYI3um9v1G6pLc1sJ150C7LmemSwbC5gYZdFyp1x4MwdLmQiAV8yKLk1Bpy5xUdSNLZ1R1J8pn\/uvODlPso777wz2GP3qKOOIj5oIu8VV6llm6KfV+bFNRGDDxVeFruj56yiJW9sCcLmVoaHaqqCP3HFI4CF3b\/MAGd+cQa+\/OKLowVnfnEGwetZhVfs0sCHTBx00EGB8L300kuD3kpuachyiX5dniv23BXVXX75TWYfXp8E77Ort9D5dS8VoEIrQ3LWYGFPxsi1EeDMNUZKxwO+\/OILgtc\/viB4PRO8HO6uXbvo+eefp7vvvjv4k\/+fd2sQ25Vl3Yos2qIQbUng6u306dODtx5Fz2405V2v8EZbGbiqu2Liaf49uYYjxoexYcAVuANnCkA0aAJ8GQRbkStwpghIQ2YgeD0UvOGQo+KXty3jqizv4vDNb36TWrVqZSiV0rmxlXjXP7aGpjyxthAstzKcfkT7dME3w9FY2P0jHZz5xRn48osvVHj948uW7nAVKed7eEsBt3XrVrrrrruCym+bNm0STzuzSYKNxItWdwedUk7czoArGQF8GCdj5NoIcOYaI6XjAV9+8QXB6x9fNnSHyyh5J3i5\/\/bpp58OxC2TydcJJ5wQVHh5j15UeP+dbv9Zt4KeWb1nRwtuZbhlYAWqu5JPIz6MJYFyaBg4c4gMiVDAlwRIjg0BZ44RkhAOBG9jgLwQvFGR+8knnwQ9vN\/5znfoG9\/4RrA9meuX6cRDdTdfRmBhz4efjdngzAbq2X2Cr+zY2ZoJzmwhn82vad2RLUpzs5wXvOIoXz4Tml9Ou+SSS+iCCy5odFqZObiyezKdeNHqLrYhS8cdFvZ0eLkwGpy5wIJ8DOBLHitXRoIzV5iQi8O07pCLyt4o5wUvHzIxZ84c6tevH33hC1\/IvBWZPYj3eDadeB3HPF24ZfTupmcfC3t6zGzPAGe2GUjnH3ylw8uF0eDMBRbkYzCtO+QjszPSecFrBxb1Xk0mHh8yMeeF9cFN4JCJbFxiYc+Gm81Z4Mwm+ul9g6\/0mNmeAc5sM5DOv0ndkS4yO6MheA3hbirxor2746q70w\/P6WHoLpuOGyzs\/nEJzvziDHz5xRdHC8784syU7vAFFQheQ0yZSjyu7HKFF9XdfMRiYc+Hn43Z4MwG6tl9gq\/s2NmaCc5sIZ\/NryndkS0687MgeA1hbiLxuLrLRwjzn3z16dmeFo6uMnSHTcsNFnb\/+ARnfnEGvvziCxVe\/\/gyoTt8QgWC1xBbJhIvXN3l28KpatnJxYdxduxszQRntpDP5hd8ZcPN5ixwZhP99L5N6I70Udmb4bzg\/eCDD+i2224Ljg0+7rjjYpF68cUX6aabbqLa2tpg6zIXLxOJd96tL9GSN7YEt88vq62YeJqLUHgRExZ2L2hqFCQ484sz8OUXX6jw+seXCd3hEyrOC16xD+\/EiROpurp6L2z5EIopU6bQI4880qyPFo6+rMZHCPN2ZLiyIYAP42y42ZwFzmyin943+EqPme0Z4Mw2A+n8Q\/A2xstJwbtr1y6aMGECPfDAA9Ls\/sd\/\/AfdfPPN1Lp1a+k5JgfqTrxH\/98mGnLny6juKiIVC7siIA2aAWcGwVbgCnwpANGwCXBmGPCc7nTrjpzhGZ\/upOBlFNasWUMzZ84kPnji6aefphNPPDH2dLU2bdpQVVUVnX766cR\/d\/XSmXjR6u7wPl1pyn8d5SoUXsSFhd0LmhoFCc784gx8+cUXRwvO\/OJMp+7wC4k90ToreAWYMj28PgCvM\/GeXb0l2J1BXHhZLX9GYGHPj6FpC+DMNOL5\/IGvfPjZmA3ObKCe3adO3ZE9KnsznRe89qBR61ln4p1ft4KeXb05CBgvq6nhDQu7GhxNWgFnJtHO7wt85cfQtAVwZhrxfP506o58kdmZ7aTgFVVdhuTb3\/423XfffcT\/Vupq164dXXHFFcR\/unjpSrxoOwOqu2rYx8KuBkeTVsCZSbTz+wJf+TE0bQGcmUY8nz9duiNfVPZmOyl4xc4MDAu\/iHbllVfSunXrSjW9cesAACAASURBVKLUrVu3ZrlLA9oZ9Dw8WNj14KrTKjjTia562+BLPaa6LYIz3QirtQ\/B2xhPJwWvWsrdsKYr8bD3rh5+sbDrwVWnVXCmE131tsGXekx1WwRnuhFWa1+X7lAbpTlrELyGsNaReNh7Vx95WNj1YavLMjjThaweu+BLD646rYIzneiqt61Dd6iP0pxF5wWv6OdFD+\/eSYF2Bn0PChZ2fdjqsgzOdCGrxy740oOrTqvgTCe66m1D8DbG1HnBK\/p50cO798OAdgb1C4SwiIVdH7a6LIMzXcjqsQu+9OCq0yo404muetsQvJ4J3mIpsHv3btq0aRPNnTuXHnzwQbrpppvouOOOU58xiiyqTrxoO8P4ft1pfL8eiqKFGSzs\/uUAOPOLM\/DlF18cLTjzizPVusOvu987WucrvEkAs\/C99tprgxPZamtrab\/99kuaYuXnqhMP7Qx6acTCrhdfHdbBmQ5U9dkEX\/qw1WUZnOlCVo9d1bpDT5TmrHoveBmqefPm0a233pp6W7IxY8bQggULCmhPmzaNqqurS6Lf0NBAI0aMoGXLlhXGjR07lmpqakrOU514o+esojkvrA984rAJ9Q8MFnb1mOq2CM50I6zWPvhSi6cJa+DMBMrqfKjWHeois2PJe8G7a9cumjBhAr3yyit0zz33UKdOnaSQZLG7fPnygkhetGgRjRo1ikqJV9FP3KVLF5oxYwaVlZXRypUraejQodS3b9+gwlzsUp14J056nritga9Bp5TTrYMqpO4bg+QQwMIuh5NLo8CZS2wkxwK+kjFybQQ4c42R0vGo1h1+3f3e0ToveJN2aXjzzTcD4cqi8yc\/+Qm1aNEikRMhUidPntyoossiuL6+viBmo4ZYFI8bN45mzZpFlZWVhR\/X1dUFvcTz58+n8vLyWP8qEw+nqyVSnHsAFvbcEBo3AM6MQ57LIfjKBZ+VyeDMCuyZnarUHZmDcGii84I3aZeGVq1a0YUXXkjXXHMNtW3bNhe0SYK3mHEWvNOnT99LCIfHq0y8cP8u2hlyUV50MhZ2PbjqtArOdKKr3jb4Uo+pbovgTDfCau2r1B1qI7NjzXnBawoWFq1TpkwhmT7eaEzR9oi4mFUmXng7sj4929PC0VWmYGo2frCw+0c1OPOLM\/DlF18cLTjzizOVusOvO4+P1lvBy7szbNu2jdq0aSPVxlCMLNG7yz\/v379\/yT7cOBsyvb88TyTe7NmzqVu3bgVTxVogisVbv\/Vj4v5dcV3Vtxu2I9PwJGJh1wCqZpPgTDPAis2DL8WAGjAHzgyAnMMFfyMevvj8gsGDB9PixYsb6Y4cLrye6oXg5RfTfvWrXwUvpnElll8W423Ihg8fHvTc\/uhHP6Lzzjsvl\/AVuy+wvVK9uGG2RS9wRUVF0b5fMV4I3mi2cO\/xsGHDpJPoxbc\/pMsf+HdSLxzWjbq0bSk9HwPlENi5cydt3Lgx6Mlu2RL4yqFmdxQ4s4t\/Wu\/gKy1i9seDM\/sclIpg5syZQWtl9ILg3YOIF4L39ttvD9oNuFd34sSJgeDdsWMHPf7444HQXL16dVCZPffcc3NloxCwI0eOTNxmLI3Y5aCE4OUX5bp27VqIkwVVmirvgy+\/R1fMeT2Yz\/27fx6LdoZcpBeZzPm1YcOG4LdiCF4dCKu3Cc7UY6rTIvjSia4e2+BMD66qrHKFN1zlXbp0KU2dOhUV3s8Adl7wikrukUceST\/\/+c\/3OliCq7\/cQ8unromtwrImj6zgFW0MvXv3lvapqpcm3L\/7w3N60Ljq7llvF\/NKIICv7vxLD3DmF2fgyy++OFpw5hdnqnSHX3ddPFrnBa\/YpWH06NF08cUXx97JAw88EBwtLNuKUGx7MSFkS724Jsak7fdVlXgdxzxdwOChmio6\/Yj2TSUXnboPLOxO0SEVDDiTgsmZQeDLGSqkAwFn0lA5MVCV7nDiZhQE4bzg5crtJZdcQmeddRaNHz8+9pZvuOEG+uMf\/yh98ITo12Vj0QMkSvXjyh4yERekisSLHif8fu1XFaQATMQhgIXdv7wAZ35xBr784gsVXv\/4UqE7\/Ltrjyu8vBvDz372M3r44Yfpl7\/8ZSB8xeES\/DMWuldffTV985vflD54QsARPVo4espadH\/d6PgorKUqwyoS74bH19ANj68N3GL\/Xb2PIT6M9eKrwzo404GqPpvgSx+2uiyDM13I6rGrQnfoicyOVecrvAwLb61x+eWX06uvvhocLsFbkfHFb4xyBfiYY44hfrEtvN2XHTiLe1WReOH+3dFnHUb\/c\/4Rrt1mk4kHC7t\/VIIzvzgDX37xhQqvf3yp0B3+3bXHFV4ROr8dyj26\/N\/WrVuDf2bxO2DAgOC\/1q1bO81L3sTDccJm6cWHsVm8VXgDZypQNGcDfJnDWpUncKYKSTN28uoOM1Ga8+J8hZfbFh588EE69thjiXdq8PXKm3jR\/t0VE08L2hpw6UEAC7seXHVaBWc60VVvG3ypx1S3RXCmG2G19vPqDrXR2LfmvODlzf95d4aLLroocW9c+3AWjyBv4qF\/1yy7WNjN4q3CGzhTgaI5G+DLHNaqPIEzVUiasZNXd5iJ0pwX5wWv2KWBT1Krqakxh4xiT3kTL9y\/26dne1o4GgdOKKaokTks7DrR1WMbnOnBVZdV8KULWX12wZk+bHVYzqs7dMRk06bzgpfBefLJJ+mnP\/1pUOn9xje+QQceeOBemO27777UsWNH4j9dvPIkXrR\/99ZBFTTolHIXb7PJxISF3T8qwZlfnIEvv\/jiaMGZX5zl0R1+3alctM4LXnHwBO\/UUOriHRpkD56Qg0btqDyJF+3fxYETarmJs4aFXT\/Gqj2AM9WI6rUHvvTiq8M6ONOBqj6beXSHvqjsWXZe8PLuDEuWLAm2ICt17b\/\/\/tSnTx9nd2vIk3jo3zX\/gGBhN495Xo\/gLC+CZueDL7N4q\/AGzlSgaM5GHt1hLkpznpwXvOag0OspT+Khf1cvN6jwmsdXh0d8GOtAVZ9N8KUPW12WwZkuZPXYzaM79ERk1yoEryH88yTeiZOeJ+7j5Wt8v+40vl8PQ1E3XzdY2P3jHpz5xRn48osvjhac+cVZHt3h153KReuk4BV9u3wLN998M1155ZXBaWulrqbaw4sDJ+QSWfUoLOyqEdVvD5zpx1ilB\/ClEk0ztsCZGZxVeYHgbYykk4L3gw8+oNtuuy2I9Nvf\/jbdd999xP9W6mrXrh1dccUVxH+6eGVNPBw4YYdNLOx2cM\/jFZzlQc\/8XPBlHvO8HsFZXgTNzs+qO8xGac6bk4LX3O2b85Q18fDCmjmOwp6wsNvBPY9XcJYHPfNzwZd5zPN6BGd5ETQ7P6vuMBulOW9eCF4+Xvixxx6jxYsX049\/\/GP63Oc+R1u3bg0Oothvv\/3ommuuoaOPPtocahk8ZU08vLCWAWwFU7CwKwDRsAlwZhjwnO7AV04ALUwHZxZAz+Eyq+7I4dLpqV4I3kcffZTGjBlDlZWVNG3aNOrQoUPQ4vDLX\/6SFi5cSLwl2e23305VVe6ePpY18TqOebqQQHhhzdyzhIXdHNaqPIEzVUiasQO+zOCs0gs4U4mmfltZdYf+yOx4cF7wNjQ00IgRI4L9dfkFtrKyskZIbd68mUaNGhWcvsZimMWvi1eWxIu+sLZi4mn0+Y4HuHh7TS4mLOz+UQrO\/OIMfPnFF0cLzvziLIvu8OsO00XrvOAVOzZcfvnlNGTIkNi7u\/fee4MKb1M7aS36wtr7tV9Nxy5GZ0YAC3tm6KxNBGfWoM\/kGHxlgs3qJHBmFf7UziF4G0PmvODduHEjXXzxxXTBBRcE25PFXXV1dXT\/\/ffTvHnzqHPnzqmTwsSELIn3v6++TxfdvjIIjyu7XOHFZQYBLOxmcFbpBZypRFO\/LfClH2PVHsCZakT12suiO\/RGZNe684L3o48+Cvp3V61aRXfccQf16NH40IU1a9bQZZddRhUVFVRbWxu8xObilSXx8MKaPSaxsNvDPqtncJYVOTvzwJcd3PN4BWd50DM\/N4vuMB+lOY\/OC16G4uWXX6bhw4fTtm3b6KSTTqLDDz88QGjTpk30zDPPUJs2bejOO++k448\/3hxyKT1lSTycsJYSZIXDsbArBNOQKXBmCGhFbsCXIiANmgFnBsFW4CqL7lDg1lkTXgheRu\/tt9+mX\/ziF\/Tkk0\/Srl27AkBbtWpFZ599Nv3whz+krl27OgsyB5Y28XDCml06sbDbxT+Ld3CWBTV7c8CXPeyzegZnWZGzMy+t7rATpTmv3ghec5Do8ZQ28XDCmh4eZK1iYZdFyp1x4MwdLmQiAV8yKLk1Bpy5xUdSNGl1R5I9338OwWuIwbSJhxPWDBFTxA0Wdrv4Z\/EOzrKgZm8O+LKHfVbP4CwrcnbmpdUddqI05xWC1xDWaROv\/20raPHfNwfR9enZnhaOdvdQDUMQGnWDhd0o3EqcgTMlMBozAr6MQa3METhTBqURQ2l1h5GgLDqB4DUEftrEC+\/QgBPWDJEUcoOF3TzmeT2Cs7wImp0PvszircIbOFOBojkbaXWHucjseILgNYR7msSLvrB266AKGnRKuaFI4YYRwMLuXx6AM784A19+8YV10T++0ugO\/+4ufcQQvOkxC\/YFXrBgQWEmH2lcXV1d0lKaxMMODRlIUTwFH8aKATVgDpwZAFmhC\/ClEExDpsCZIaAVuUmjOxS5dNoMBG9KeljsLl++vHCM8aJFi2jUqFE0duxYqqmpKWotTeLhSOGUpGgYjoVdA6iaTYIzzQArNg++FANqwBw4MwCyQhdpdIdCt86a8kbw8olqf\/jDH+gf\/\/hHLJjt2rWjK664gvhPXdfKlStp6NChNHny5EYVXRbB9fX1NGPGDCorK4t1nybxsEODLgbl7WJhl8fKlZHgzBUm5OIAX3I4uTQKnLnERnIsaXRHsjX\/R3gheB999NGgjUAcOBEHe7du3QpVV9O0qBa8OFLYNIN7+8PCbp+DtBGAs7SI2R0Pvuzin8U7OMuCmr05ELyNsXde8DY0NNCIESPo3XffpV\/\/+td07LHHUosWLexlUMRzXV0dTZkyhZL6eNMkHo4Utk8vFnb7HKSNAJylRczuePBlF\/8s3sFZFtTszUmjO+xFac6z84J3\/fr1NGDAALrkkkvosssuM4dMgifRu8vD+vfvT7W1tSVniMSbPXs2cTVaXOXljXdfqN\/6MbHgFdcDlx9Ppx\/R3pn7bi6BYGH3j2lw5hdn4MsvvjhacOY2Z6yXwte6deto8ODBtHjx4ka6w+270Bed84J38+bNNHz4cDr33HOdEryCElGB5h7e+fPnU1TAinFC8Eap5J7gYcOGFf6ZBe95M9cV\/v\/2C8vp5K4H6MsAWI5FYOfOnbRx48aAz5YtWwIlDxAAZx6QFAoRfPnFF0cLztzmbObMmTRr1qy9goTg3QOJ84KXg7z99tvpscceC\/7s3LmzcxknXmYbOXJk0Z0ahODlF966du1auAcWVGGR\/OzqzTTgzlf\/LYCvP825+20OAe3YsYM2bNgQ\/FYMwesH4+DMD55ElODLL744WnDmNmdc4Q1XeZcuXUpTp05Fhfcz2pwXvPyA\/fGPf6S77rqLXn31VTrjjDOoTZs2e2WdiV0aiqV6GsGb9JsWdmhwY0HBV3du8JAmCnCWBi37Y8GXfQ7SRgDO0iJmdzx6eBvj77zgFT283ItS6jKxSwP37Y4bNy74yqCysrIQjujnLfXimmziYYcGuwuE8I6F3Q0e0kQBztKgZX8s+LLPQdoIwFlaxOyOl9UddqM05915wWsOimRPol+XR4o9d0V1t6KiQsk+vGHBe\/PAY+jbvQ9NDgwjlCOAhV05pNoNgjPtECt1AL6UwmnEGDgzArMyJxC8nlV4lTGv0FD0aOGkU9bYtUziRY8UvnVQBQ06pfEuDgpvA6ZKIICF3b\/0AGd+cQa+\/OKLowVnfnEmozv8uqN80XpT4X3jjTdo0qRJ9Pzzz9PBBx8c7Ihw4IEH0jXXXEPnnHMOnXfeeU7tzxulRSbxooL3oZoqbEmWL78zz8bCnhk6axPBmTXoMzkGX5lgszoJnFmFP7VzGd2R2qjHE7wQvC+\/\/HKwNdk+++xD3bt3p3feeScQvPvvv3\/w76tWrQoOpaiurnaWCpnEiwre92u\/6uz9NPXAsLD7xzA484sz8OUXX6jw+seXjO7w766yR+y84P3oo4+CY4X5pTXeluyll14KKr1iz9utW7cG+\/NytZdfGmMR7OIlk3jYocEd5vBh7A4XspGAM1mk3BgHvtzgIU0U4CwNWvbHyugO+1Gai8B5wcub\/1988cV00UUXBXvc8o4IYcHLUN1xxx10zz33lDz4wRyk8Z5kEm\/0nFU054U9J6X06dmeFo6ush12s\/WPhd0\/6sGZX5yBL7\/4QoXXP75kdId\/d5U9YucFr9iW7PLLL6chQ4bECt577703qP6WOuksO0RqZsokXniHBn5ZjV9aw2UHAXwY28E9j1dwlgc983PBl3nM83oEZ3kRNDtfRneYjciuN+cFr9gKrFOnTlRbWxscQhHX0rDffvvR9OnT6XOf+5xdRIt4l0m8Eyc9T9zHyxd2aLBLIxZ2u\/hn8Q7OsqBmbw74sod9Vs\/gLCtydubJ6A47kdnx6rzgZVgeffRR4q2\/uMJ7+OGH02233Rb063Jf7y233BK8tHbDDTcEbQ+uXkmJhx0a3GIOC7tbfMhEA85kUHJnDPhyhwvZSMCZLFJujEvSHW5EaS4KLwTv7t27g9PNJk+eTNu3b2+Ezr777hu81MYtD\/x3V6+kxHt29RY6v+6lQvjYkswuk1jY7eKfxTs4y4KavTngyx72WT2Ds6zI2ZmXpDvsRGXPqxeCV8DDOzIsW7Ys+G\/Xrl108skn0+mnn04dOnSwh6Ck56TEiwpebEkmCaymYVjYNQGr0Sw40wiuBtPgSwOomk2CM80AKzafpDsUu3PenFeC13k0SwSYlHjYkswtdrGwu8WHTDTgTAYld8aAL3e4kI0EnMki5ca4JN3hRpTmovBG8HJbw+uvv06\/+93vaNu2bQFC\/CLbhRdeSD169DCHWEZPSYmHLckyAqtpGhZ2TcBqNAvONIKrwTT40gCqZpPgTDPAis0n6Q7F7pw354Xg5VaGH\/3oR8HLayx8w1eLFi1o4MCBdO2111KrVq2cBTwp8bAlmVvUYWF3iw+ZaMCZDErujAFf7nAhGwk4k0XKjXFJusONKM1F4bzgZYHLL6vdeeedNHr0aBo2bBi1bds2QIiFMB86wXvwXnXVVcGLa65eSYkX3pJsfL\/uNL6f+1VrV7FWERcWdhUomrUBzszindcb+MqLoPn54Mw85nk8JumOPLZ9nOu84N20aRNdcskldMopp9B1111HXNENXyyIubr78ssvB6LY1RfYSiVedEuyFRNPo893PMDHfGoyMWNh949KcOYXZ+DLL744WnDmF2cQvI35cl7wipPWuLrLRwzHXQsXLgyqwL6etIY9eN1bRLCwu8dJUkTgLAkht34OvtziQyYacCaDkjtjIHg9E7w7d+6kK6+8MujP5ZPWon26vD3ZhAkTgvaGm2++mfbff393si0USanEw5Zk7lGGhd09TpIiAmdJCLn1c\/DlFh8y0YAzGZTcGQPB65ng5XD5RDXuzz3ssMPo6quvpu7du9M+++xDXP3lk9YWLVpEv\/jFL+i4444r3B0fQtG5c2dnMq9U4mFLMmdoKgSChd09TpIiAmdJCLn1c\/DlFh8y0YAzGZTcGQPB65ngFS0NLHrTXN26daPFixenmaJ1rKzg7dOzPS0cXaU1FhhPRgALezJGro0AZ64xUjoe8OUXXxwtOPOLMwhezwTvjh07aMmSJcStDWkubm04++yz00zROrZU4oW3JIPg1UqDtHEs7NJQOTMQnDlDhVQg4EsKJqcGgTOn6EgMBoLXM8GbyKgnA2QFL7Ykc4NQLOxu8JAmCnCWBi37Y8GXfQ7SRgDO0iJmdzwEr8eCd\/PmzfTss8\/Siy++GLy8VlVVRV\/60pec3YosDHWpxOs45unC0FsHVdCgU8rtPiXwjq\/uPMwBfBj7RRr48osvjhac+cUZBK+HgveTTz4JDpfgXRr47+GLX07jLctqamq8PGkNe\/C6uYBgYXeTl1JRgTO\/OANffvEFwesfXxC8Hgre+++\/n8aPH09nnHFGcKIa79LA19q1a+nGG2+k5557LtiH9\/zzz3c2I4slXnRLMhw64QaF+DB2g4c0UYCzNGjZHwu+7HOQNgJwlhYxu+MheD0TvA0NDTRixAgqKysL9tlt3bp1ozvgl9p4n16u\/E6bNs27fXixB6\/dBaGYdyzsbvKCCq9\/vOAZA2dNBwG\/7gSC1zPBK7Yl4314hwwZEptt9957b9Dy4ONJa9iD180FBILXTV4geP3jBYIXnDUdBPy6EwhezwTvxo0bgyOFL7jggqCSG3dx5ff3v\/89zZs3z6nDJsKxFku8Xy5aS\/\/3D2uCodiSzJ3FBILXHS5kIwFnski5MQ58ucFDmijAWRq07I+F4PVM8H700Uc0ZswYWrVqFd1xxx3Uo0ePRnewZs0auuyyy6iioiJ4qW2\/\/faTyjLRKrFs2bLC+LFjxwYvvyVddXV1NGXKlFTziiUe9uBNQtvOz7Gw28E9j1dwlgc983PBl3nM83oEZ3kRNDsfgtczwcvhvvzyyzR8+PDg8Imvfe1r1KdPn+Au+ECKJ554Iujb5ZYG3qZM5hJtEl26dKEZM2YE\/cErV66koUOHUt++fQPhXOxisTt9+nSaNWsWVVZWFuaNHDmypFiWEbzYg1eGPTNjsLCbwVmlF3CmEk39tsCXfoxVewBnqhHVaw+C10PByyG\/9tprNGHCBFqxYgXt3r07uIsWLVrQiSeeSNdffz0dffTR0pmzaNEiGjduXEG0ioksZufOnVu0F1gI5V69ejUSxVyBXr58ecke4mKJhz14pWkzOhALu1G4lTgDZ0pgNGYEfBmDWpkjcKYMSiOGIHg9FbwibK7y8gEUfHXo0EHprgzR6m00I3UL3odqquj0I9obeRDgpDQCWNj9yxBw5hdn4MsvvjhacOYXZxC8ngtenekmU6kt1tKQ1AoRl3jYg1cnm\/lsY2HPh5+N2eDMBurZfYKv7NjZmgnObCGfzS8ELwRvbOZwm8OoUaNI5sU10e+7bdu2wBbv\/1tdXV0yI0XizZ49m7p16xaMXfthGZ1f91Jh3ru\/\/Eq2rMYs5QhgYVcOqXaD4Ew7xEodgC+lcBoxBs6MwJzZCX8LHb7WrVtHgwcPpsWLFxd0R2bjTWBii92iIbYJ3EzWWxAClnd6EC+xxdkSOzvU19cX+nXjXoCLmysEb\/hnHx7zn\/ThMXtOh+vStiUtHLZHCOOyjwC3zvCWeOXl5dSyZUv7ASGCRATAWSJETg0AX07RIRUMOJOCydqgmTNnBu8mRS8I3j2INHvBKyt2GSxRBY5WdIUNPt64WKVXCF4e07Vr1wD8+175mO575aPg77wH72+\/W2HtQYHjxgjwCX4bNmwIfiuG4PUjO8CZHzyJKMGXX3xxtODMbc64ABeu8i5dupSmTp2KCu9ntDVrwSsEbO\/evUtWdkWKF9vdQVR5Bw4cWHRrsrheGuzB6+7iga\/u3OWmWGTgzC\/OwJdffHG04MwvztDD25ivZit4hdjt379\/yX13w3CpqPCGv1oIC17swevWQoKF3S0+ZKIBZzIouTMGfLnDhWwk4EwWKTfGQfBC8EofMhFNWRU9vGHBe+Kk5+kf738YuIHgdWOBEFFgYXeLD5lowJkMSu6MAV\/ucCEbCTiTRcqNcRC8ELzBUcULFiwompGiR7fYvrzR+TJV4mjisdBlwSsu7MHrxgIBwesWD2miwYdxGrTsjwVf9jlIGwE4S4uY3fEQvBC8VjIQgtcK7JmdYmHPDJ21ieDMGvSZHIOvTLBZnQTOrMKf2jkELwRv6qRRMSGaeDh0QgWq+mxgYdeHrS7L4EwXsnrsgi89uOq0Cs50oqveNgQvBK\/6rJKwmCR436\/9qoQVDDGFABZ2U0ir8wPO1GFpwhL4MoGyWh\/gTC2euq1B8ELw6s6xWPvRxLvh8TV0w+Nrg7Gf73gArZh4mpW44DQeASzs\/mUGOPOLM\/DlF18cLTjzizMIXgheKxlbSvDyoRMLR1dZiQtOIXibSg7gw9gvJsGXX3xB8PrHFwQvBK+VrI0mHg6dsEKDtFN8GEtD5cxAcOYMFVKBgC8pmJwaBM6coiMxGAheCN7EJNExoJTgrb3oaLr0y110uIXNjAhgYc8InMVp4Mwi+Blcg68MoFmeAs4sE5DSPQQvBG\/KlFEzPJp4OHRCDa66rGBh14WsPrvgTB+2OiyDLx2o6rUJzvTiq9o6BC8Er+qckrIXTbyOY54uzLt1UAUNOqVcyg4GmUEAC7sZnFV6AWcq0dRvC3zpx1i1B3CmGlG99iB4IXj1ZlgR6+HE+\/TATjhlzQoL8k6xsMtj5cpIcOYKE3JxgC85nFwaBc5cYiM5FgheCN7kLNEwIpx4az8so\/PrXip44S3JeGsyXO4ggIXdHS5kIwFnski5MQ58ucFDmijAWRq07I+F4IXgtZKFpQQvDp2wQklJp1jY3eMkKSJwloSQWz8HX27xIRMNOJNByZ0xELwQvFayMZx4973yEQ6dsMKCvFMs7PJYuTISnLnChFwc4EsOJ5dGgTOX2EiOBYIXgjc5SzSMgODVAKpGk1jYNYKryTQ40wSsJrPgSxOwGs2CM43gajANwQvBqyGtkk2GE+8Xz2yjOS+sDybhlLVk7GyMwMJuA\/V8PsFZPvxMzwZfphHP7w+c5cfQpAUIXghek\/lW8BVOvCse3EhL3tgS\/Iy3I+NtyXC5hQAWdrf4kIkGnMmg5M4Y8OUOF7KRgDNZpNwYB8ELwWslE4sJ3vH9utP4fj2szwN8UwAAHKVJREFUxASnxRHAwu5fdoAzvzgDX37xxdGCM784g+CF4LWSseHEO6H274UYcOiEFToSnWJhT4TIuQHgzDlKSgYEvvziC4LXP74geCF4rWStSLx7Fyyib979z0IMD9VU0elHtLcSE5yiwtuUcgACyi82wZdffEHw+scXBC8Er5WsheC1Antmp\/gwzgydtYngzBr0mRyDr0ywWZ0EzqzCn9o5BC8Eb+qkUTFBJN4v7nqI+KU1ceGUNRXoqreBhV09protgjPdCKu1D77U4mnCGjgzgbI6HxC8ELzqsimFpWKCF6espQDR4FAs7AbBVuQKnCkC0pAZ8GUIaIVuwJlCMA2YguCF4DWQZnu7EIl3xY3ziffh5evzHQ8grvDicg8BLOzucZIUEThLQsitn4Mvt\/iQiQacyaDkzhgIXgheK9koEu8bP5pFfLQwBK8VGqSdYmGXhsqZgeDMGSqkAgFfUjA5NQicOUVHYjAQvBC8iUmiY4BIvOOvuJ2eeWffwAVOWdOBtBqbWNjV4GjSCjgziXZ+X+ArP4amLYAz04jn8wfBC8GbL4MyzhaJ137gVFr7YVlgBaesZQTTwDQs7AZAVuwCnCkGVLM58KUZYA3mwZkGUDWahOCF4A0QaGhooBEjRtCyZcsKiIwdO5ZqamoS02\/RokU0atSowrjevXvTjBkzqKxsj5CNu+IEL05ZS4Ta2gAs7Nagz+wYnGWGzspE8GUF9lxOwVku+IxPhuCF4KX169fTgAEDqEuXLgWhunLlSho6dCj17duXamtriyZmXV0dTZkyhaZNm0bV1dWxtkoJ3q3VN9CnB3YKhkDwGn\/+pR1iYZeGypmB4MwZKqQCAV9SMDk1CJw5RUdiMBC8ELzEFdpx48bRrFmzqLKysoAIi9m5c+fS\/Pnzqby8fK9kEkJ54MCBjSrBxeyFDYjE29L\/N4V\/xilric+rtQFY2K1Bn9kxOMsMnZWJ4MsK7LmcgrNc8BmfDMELwVuyejt9+vS9hLCYwMJ20qRJRQVxqWzmxBs44vvEFV5xQfAaf\/6lHWJhl4bKmYHgzBkqpAIBX1IwOTUInDlFR2IwELwQvEWTZMyYMbR8+fKigpYrwIsXL6brr7+eLr30Ulq3bl1gq3\/\/\/iXbIHgMBG\/is+nUACzsTtEhFQw4k4LJmUHgyxkqpAMBZ9JQOTEQgheCNzYRxYtopV5cY0G8YMECatOmTaEKHNcPHOeAE2\/A96+jhtOvLvz44UsPo95f7O7Eg4EgGiOAhd2\/jABnfnEGvvzii6MFZ25zxnokfHFRbvDgwUGhrlu3bm4HbyC6Frt3795twI\/TLsQLaxUVFSV3WxCCV7ywJm5KiOXov4dvmgXvRdfcSttPGl745\/YLvhu8KDds2DCn8WmOwe3cuZM2btwY9HK3bNmyOULg3T2DM78oA19+8cXRgjO3OZs5c2ZQjIteELx7EGn2gldW7DJYLHifeuqpvXp8i73MVkrw8rHC085uEQiquBfk3H6smn50O3bsoA0bNgS\/FUPw+sE3OPODJxEl+PKLL44WnLnNGWuRcJV36dKlNHXqVFR4P6OtWQteUZmV2UeX8eIe3riX2mQF74U\/m0cfHnN+AD1OWXN74cBXd27zExcdOPOLM\/DlF18cLTjzizP08Dbmq9kKXiF2ZV44E5CJavDIkSMzbUvWf8qTtOvzfSB4PVgzsLB7QFIkRHDmF2fgyy++IHj94wuCF4KXZA+ZiEvvaFuDqO726tWr5E4NnHjn162gjzsdHZjFscJuLx74MHabH1R4\/eMnGjGeMf84BGd+cQbBC8Eb9OLybgvFLvHyWbEWBnHampgvUyWOCl6csub2woGF3W1+IHj94weCF5z5j4BfdwDBC8FrJWM58b559z9xrLAV9NM7heBNj5ntGeDMNgPp\/IOvdHi5MBqcucCCfAwQvBC88tmicCQn3rnzdxQs3jqoImhrwOUmAljY3eSlVFTgzC\/OwJdffHG04MwvziB4IXitZGxU8K6YeBrx1mS43EQAC7ubvEDw+sdLsYjxjPnHJTjzizMIXgheKxkbFbwP1VTR6Ue0txILnCYjgIU9GSPXRoAz1xgpHQ\/48osvVHj94wuCF4LXStaiwmsF9sxO8WGcGTprE8GZNegzOQZfmWCzOgmcWYU\/tXMIXgje1EmjYkJU8L5f+1UVZmFDEwJY2DUBq9EsONMIrgbT4EsDqJpNgjPNACs2D8ELwas4peTMhQUv9+5yDy8udxHAwu4uN8UiA2d+cQa+\/OKLowVnfnEGwQvBayVjIXitwJ7ZKRb2zNBZmwjOrEGfyTH4ygSb1UngzCr8qZ1D8ELwpk4aFRPCgrdPz\/a0cHSVCrOwoQkBLOyagNVoFpxpBFeDafClAVTNJsGZZoAVm4fgheBVnFJy5iB45XByZRQWdleYkI8DnMlj5cJI8OUCC+liAGfp8LI9GoIXgtdKDoYFLx84wQdP4HIXASzs7nJTLDJw5hdn4MsvvjhacOYXZxC8ELxWMjYseMf3607j+\/WwEgecyiGAhV0OJ5dGgTOX2EiOBXwlY+TaCHDmGiOl44HgheC1krFhwYtjha1QkMopFvZUcDkxGJw5QYN0EOBLGipnBoIzZ6iQCgSCF4JXKlFUDwoLXpyyphpd9fawsKvHVLdFcKYbYbX2wZdaPE1YA2cmUFbnA4IXglddNqWwBMGbAiwHhmJhd4CElCGAs5SAWR4OviwTkME9OMsAmsUpELwQvFbSD4LXCuyZnWJhzwydtYngzBr0mRyDr0ywWZ0EzqzCn9o5BC8Eb+qkUTEhLHj5lDU+bQ2XuwhgYXeXm2KRgTO\/OANffvHF0YIzvziD4IXgtZKxYcH7fu1XrcQAp\/IIYGGXx8qVkeDMFSbk4gBfcji5NAqcucRGciwQvBC8yVmiYYQQvFzZ5QovLrcRwMLuNj9x0YEzvzgDX37xhQqvf3xB8ELwWslaCF4rsGd2ig\/jzNBZmwjOrEGfyTH4ygSb1UngzCr8qZ1D8ELwpk4aFROE4O3Tsz0tHF2lwiRsaEQAC7tGcDWZBmeagNVkFnxpAlajWXCmEVwNpiF4IXg1pFWySU688+tW0Le+fiaOFU6Gy\/oILOzWKUgdADhLDZnVCeDLKvyZnIOzTLBZmwTBC8FrJfmQeFZgz+wUC3tm6KxNBGfWoM\/kGHxlgs3qJHBmFf7UzqE7IHhTJ42KCUg8FSias4GF3RzWqjyBM1VImrEDvszgrNILOFOJpn5b0B0QvPqzLMYDEs8K7JmdYmHPDJ21ieDMGvSZHIOvTLBZnQTOrMKf2jl0BwRv6qRRMQGJpwJFczawsJvDWpUncKYKSTN2wJcZnFV6AWcq0dRvC7oDgjdAoKGhgUaMGEHLli0rIDJ27FiqqalJlYV1dXU0d+5cmj9\/PpWXlxedi8RLBav1wWvXrqW77747yJFu3bpZjwcBJCMAzpIxcmkE+HKJDblYwJkcTq6Mgu6A4KX169fTgAEDqEuXLjRjxgwqKyujlStX0tChQ6lv375UW1srla9iTrt27SB4pRDzZxAWCn+4EpGCM784A19+8cXRgjO\/OANfELy0aNEiGjduHM2aNYsqKysLiMhWa6MVYq4AosLr10KQFC0WiiSE3Ps5OHOPk1IRgS+\/+ILgBV\/+IQDBW5QzFrzTp0\/fSwjHTeCxixcvpqqqKnrkkUcgeH1\/EiLx48PYP0LBmV+cgS+\/+ILgBV\/+IQDBW5SzMWPG0PLlyxPFa7hCvGTJklQ9vLNnz0ZPqAdPzbp162jw4MEEvjwg67MQwZk\/XHGk4MsvvsCZv3xxcQ7vohC12L17927\/aFQfMYvYUaNGUdKLa6L\/d+DAgcELbrJtELy4cxvF0qVL1QcPi0AACAABIAAEgAAQiCBw6qmn0pw5c4ALQfAGSSBePquoqCi8xFYsO7gKXF9fXxgnK3jFb8csfHEBASAABIAAEAACQEA3AlzZRXV3D8rNvsKbRuzGveyWRvDqTmzYBwJAAAgAASAABIAAENgbgWYteEUbQ+\/evRMruwwdV3cXLFhQNI+S2iGQgEAACAABIAAEgAAQAALmEWi2gleI3f79+0vvuxtHDyq85pMWHoEAEAACQAAIAAEgkAaBZil4sxwyUQxUCN406YaxQAAIAAEgAASAABAwj0CzFLxJrQnTpk2j6urqYAeGpH15IXjNJy08AgEgAASAABAAAkAgDQLNUvCmAQhjgQAQAAJAAAgAASAABPxGAILXb\/4QPRAAAkAACAABIAAEgEACAhC8SBEgAASAABAAAkAACACBJo0ABG+Tphc3BwSAABAAAkAACAABIADBqzkHGhoaaMSIEbRs2bLAE594Mn\/+fCovL9fsGeaTEAi\/vNimTRuaNWsWVVZWlpwmtrMTg8BnEsrqfp6Fr7D36LHg6iKDpWIIZOEsumaybfEiMZDWi0AWvsRzJU4RxZqol6Ms1plXvmpra7NMbzJzIHg1UikW7i5duhQSjRNv+fLlEL0acZcxHXdEtMyOHFOmTGn04ct2nnrqKSmxLBMXxsQjkIWvqCXxYY4DYsxkWRbOhHjiNXPGjBlUVlYW7JYTfe7M3EHz8pKHr169euEzztF0Ec9P3jMHHL29VGFB8KaCK93guG3NUGVKh6GO0aJKG64axf1yEvZd7OfgUwdDjW1m4SsaVbgyD8HrLmdx2zwmPZv676bpe8j6jMV9xol97keOHEk1NTVNHzxH7zD6TQkELxEEr8Zkjf7GLFwV+3eNocB0CIFieydn2VNZCN5whQNgq0UgL1+CI24t4qrhwIED8UGslqK9rGXhTHxAn3HGGeBHMz9R81n4YhsQvIaJknQnnqX6+nq6++67acKECRT+plnSTJMbBsGridJSVQm0NWgCXdJssV84ZA4aibpANUMS9BzD8vAVfg6vvvpqGjBgAARvDi5kp2bhTPxiMnHiRFq9enXQxsCXbH+9bGwYtzcCWfhiK3G\/8KPNy60Mwzck\/+YDgldTbpZKsiyVRE1hNkuzxRZ3\/lpv3Lhx0v244d+i8SKivlTKw1f4WeMIIXj18RS2nIUz8cvjtm3bKPz1K3p49XOWha8o3wsWLAj+qXfv3oX+a\/2Rw0MSAhC8ELxJOZL75xC8uSHUZiDv4i4CEy9B4Q1ybVQFhrPyJQTU5MmTg6PC0W+tlydVgreioqKRYBJrKdsXL7KZu5Pm4SnrMyZ6f8N98fgFxa2cgeCF4NWekWhp0A5xZgdZv76Lq2hA7GamQXpiFr7inj8IXmnIcw\/Mwpn4BaVv3757bZ+Eb8VyU1LSQBa+Sv0igvdU9PKVxjoELwRvmnzJPBYvrWWGTuvErC9ocFDhN18hdrXSVDCeha\/w1+NxUWKvUL3cZeGs1AugELzu8YVvMfVyoso6BC8Er6pcKmknbpFGlckI9CWdxPXqyiwKYsyqVauk+3zt363\/EWTlK3rnePbM5UJWzuJe6JV5Ns3dWdP0lIUvVHj9yAU8PxC8RjI1bhN17NBgBPqSTuJeNpPZoQFvH9vhLitfELx2+Ap\/E8LbIokXOmWesbi2Bpl59u60aXjO+oyhh9d9\/iF4IXiNZWl082d8lWoM+kRH4qUzHhi39VH4l5MNGzbQ0KFDid8gj7vwZnIi3LkHpOEr7uhuVHhzU5DaQBbOokfVYluy1LBnnpCFr2j7EPjKDL+WiRC8ELxaEgtGgQAQAAJAAAgAASAABNxDAPvwuscJIgICQAAIAAEgAASAABBQiAAEr0IwYQoIAAEgAASAABAAAkDAPQQgeN3jBBEBASAABIAAEAACQAAIKEQAglchmDAFBIAAEAACQAAIAAEg4B4CELzucYKIgAAQAAJAAAgAASAABBQiAMGrEEyYAgJAAAgAASAABIAAEHAPAQhe9zhBREAACAABIAAEgAAQAAIKEYDgVQgmTAEBIAAEgAAQAAJAAAi4hwAEr3ucICIgAASAABAAAkAACAABhQhA8CoEE6aAABBIj8Ann3xCDz74IO3YsYO+\/e1vpzbw9ttv04wZM6impoY6d+6cen6eCU8++ST99Kc\/pfr6euratSvNnTs3+DPtJY7T7dWrF9XW1qad7tz46PHAY8eODfixcUWPvp02bRpVV1fbCAU+gQAQsIgABK9F8OEaCAABIiGOBg4cmEkU1dXVBUJz\/vz5VF5ebgzSrVu30mWXXRaI3VGjRtEhhxxCffr0odatW6eOoakK3kMPPZSGDh1KRx55JB111FGpcVExYfPmzbR06VL6y1\/+EvxiBMGrAlXYAAL+IQDB6x9niBgINCkEfBW8KkWqSlsuJIeL97No0aLgFxMIXhcyBDEAAfMIQPCaxxwegUCzQWD37t20cOFCuvHGG2ndunW033770YknnkjXXXcdHX300RT9urlNmzY0a9YsqqyspIaGBrrllltowYIF9O6771KLFi2oW7dudNVVV9F5550X\/P+YMWOCn4urf\/\/+hZaA1157jX7yk58ElT2+TjjhBLr66qvp1FNPTcSf2yRuvvlmevjhh2n79u3UpUsX+u53vxu0XLRq1YqEeAobKvW1PePw2GOP0a9+9St68803AxxOO+00mjhxIvXs2bNQ5a6qqqLevXvTr3\/9a9q4cWPgl2MW98v+du3aRffddx\/dfffdAaZs++CDD6bhw4fTpZdeGsTHl6h8f+c73wn8fvrpp\/Szn\/2MLrzwQnrjjTeCvz\/33HO077770qBBg+j4448PeBH4sw1uM2E\/d955J7333nt00EEH0X\/913\/R9773PSorKyuKY5zgZT5HjBgR3NNZZ51FP\/\/5z2nTpk1BHkyYMCGojjOnxa5ilXzZCj8Eb2LaYwAQaNIIQPA2aXpxc0DALgKPPPJIIFDPPPNM6tevH23ZsoV+85vfBMJm9uzZxAKXhciUKVPoy1\/+Mn39618PBOnnPve5QMz+8Y9\/pIsuuohOOeUUWrNmDc2ZMycQXtOnTw9svvjii4FAY+H2gx\/8gI455hg6+eSTA5s8v3v37jRkyJAAhHvvvZdWr14dCOJzzz23KDAsIlk4suDkPw8\/\/PBAVC9ZsoQGDBgQiEIWak8\/\/f\/bO3eQOrooCg82ioVEsVVQsFCsrFIkgq9KQSxEEMROkRBCihCtLBQUbbQStBCxEnyAhSCiAQ1ENJBgoRAb0T4gYiFa\/Hwb9uVk\/pnxXJ\/36j6QJs6dc86aq3yzZu0934KJiYmgvLw88bE9QIqryB4rKytlPQAh6ykqKgpmZ2cFOjk3\/08MoLu7O8jNzRXQZB1ohgaca3x8PJiZmREIrq2tDc7Pz+VcgPTg4KCsRYGXG43CwsKgp6dH5qivrxfYZl9cC9ZSUFAgUAvcAtPuDQdg+\/Pnz9Q12N\/fDxYXF+WmBdDks1EjCXiPjo5kDe3t7aIt87F2ID8pW2vA+7y\/yza7KZDtChjwZvsVtPWbAhmsANCJWwqgKRx9\/\/5dYHR4eFgAJyrSgDtLkRNuolvsBHDhtPJoWv8\/DEJkawE68rwAn2ZqgTnmJXML4EXBGkAJNFJEB2wCzwwK6wBWnFWFT9\/H9qenp0FnZ6dAIrCtDuzu7m7w8ePHYGBgQGAf4MWFnZ+fD8rKymRe3S9ZYY4FtHt7eyUPi0MLODJ0DtxhLXpDF9Y8MjISdHR0yHG6P9zm6enpAEeZ4UK+Ai+Qj0POed69e5f6lrFuAJr5cdTTBd7fv3\/\/c9NBxhZ3mpuguOuiAB+V1TaHN4P\/ANjSTIEMUsCAN4Muhi3FFHhpCgCcwC7AhvMY1UUhnQyvHtvc3Bx8\/fpV5AoDz69fv8QhbWlpkcfk7sD5xTV2H9u7Pwcou7q6BDgnJydTQMkxOJPsAceZuX2Bl04OwDnxjDgHU88FgOIa66N93zk0LoBjrOtGF5xwd69J+yPCQVEXxxP\/wDHHQcflxYnXgTPMGt+\/fx\/bUSLJ4eU8zONGIlgr3xNgv7q6OvLXwBzel\/bXwfZjCjytAga8T6u3zWYKvCoFcHcVnNg4WVNiC4CjuphJwHt1dSWPu4HNvb29AHcYh9bN6oZBKCpf64oOTAKCjY2N\/7sWUUCtB4UhzhdGATlgNw6yOX\/cueL+H7cabQ8PDwVKd3Z2RCdiDwqTUcDLZ3B7+YdjHL4Z+PLli6yTXDF5WzSPG7jS6Ej8JDxuy\/CGW6\/55GsNeF\/Vnw7brCnw4AoY8D64pHZCU8AUcBXgMTowtrCwEKyvr6eK1zRLGwW8AB2P48m3EicgfkB\/W4rZNjY2\/nEX44D3LtX4jwG8UeDpA4hRIIyWACk5Xorp8vPzJfNL\/17gFAc9CXhPTk6k8I4WcD7AG+XG+ny77wK8fX19sTcizGnA66O8HWMKmAJxChjw2nfDFDAFnlSBP3\/+SAaXgiWglAwn+VW3D+\/W1lYAAJHt\/PDhQ+rxtwIbhW1uVtXNdgJ+ZHiJNWjswXeDPpEG3Glg0dfhjYs0UIxGfKKpqUkgFA3CL54Iz6GxCvY\/NDQkBWkMzcFS6JYEvEn7c8G8qqoq+PTpk3TRILdM8V86Iwl4yVRz3VmrDubm5oa54vr1xh1DFwtyxbf1YfZxkdPZox1rCpgC2aWAAW92XS9brSmQNQpcXFwEnz9\/FocWWNHisb9\/\/wqMFhcXxwIvxwNFc3NzqcIq3E2ghhZWra2tscCrRWtkTfl8SUmJaIZrzGdpU0bhmUYqXEFvK1oDyihm43G+L\/DGFa2trq5KyzG6E5CZ9QFehTbytuSYdRD1ICcMqCYBb1zRmgIzTrxbtEaHDSIp5Hg1V0yEgsI5Wpnx86hxW5cGtyCQlnN8H4i7cM3jXtxBJwogf2pqSrpNMM7OzuSzNzc3BrxZ85fBFmoKPI8CBrzPo7vNagq8CgXoBEA0gW4HQCpjZWVFnENtQ6WuIz\/D1aUY6uDgQKCKGAMuLxBE5wTcWzoZAHvq8CoIAT60PmOutbU16chAYRTnoeiKzwNrQDgOc1zPV5+2ZHRa8AVety0ZzixFb8fHx1KgVVNTIzcDFJ35AC8OL\/tEA\/ZFhGF7e1vasF1fX8vNQRLwojEuOc4ybchoS8Y5AFD2nZeXlwJebhyA6B8\/fghgojmZ4eXl5eDNmzfSUYG+yOkCL9eQ4jp3buZyu0Yo2Lu9jXXvdKagUweDGxrWArCrw6vXhX6\/bnGcObyv4k+ObdIUiFXAgNe+HKaAKfBoCuDuEjcAZgAqRvhFAwAh8AcYX15eSo6zoaFBXlgxNjYmRWpkVem7CxDT+YHes9rCikIs4JZ+sbzMQQupeJ0snweeAUTgjMf0gDc9aZNG1IsniDG0tbWl2or5Ai\/zhF88AcADuArl6RSt0XOYlm60btMXeVBshmO8ubkp\/Y1LS0sFpMNdGnTPvHhC3W598QRrIlLgFtfpyz+Wlpak\/zHXoa6uLujv75ebkbiR5PCiBdcStxwXHuinxRnfCx1RwMvP3L0DzXT\/4LqOjo4a8D7ab7Gd2BR4GQoY8L6M62i7MAVMAVPgXgoQk6CwkH9JMOszSbptyXzOed9jzOG9r4L2eVMguxUw4M3u62erNwVMAVPAWwHavJHHzcnJEadce+ESKcAtJeYR7pHrfXLnQAPeu6hmnzEFTIHHVMCA9zHVtXObAqaAKZBhChBZIBLx9u1b6WdMMZ\/va5d9t6LAS8s0ulpUVFQEZGrp7ct4CKj2XQv5XuItFCsy713a1fnOZceZAqZA5ipgwJu518ZWZgqYAqbAgytArpoCPt7IRq4at5cuEeSAKaqLK+ZLZyEKvJrbpvgM8H0O4KVAkrnpGsIw4E3nStqxpsDLUeA\/pe6cJyi5WKwAAAAASUVORK5CYII=","height":337,"width":560}}
%---
%[output:6f3c261a]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"    44"}}
%---
%[output:7d5e9062]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     3.000000000000000e-02"}}
%---
%[output:71a97270]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"     7.500000000000001e-02"}}
%---
%[output:5cd9d49f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     6.215564738292011e-09"}}
%---
%[output:60ed39a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     3.217728531855956e+03"}}
%---
