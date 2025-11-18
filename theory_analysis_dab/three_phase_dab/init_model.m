%[text] ## dabGeneral Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1.5;
transmission_delay = 125e-6*2;
s=tf('s');

% model = 'single_phase_dab';
model = 'three_phase_dab';
use_thermal_model = 1;
%[text] ### Voltage application
application400 = 0;
application690 = 1;
application480 = 0;

% number of modules (electrical drives)
n_modules = 2;
%[text] ### PWM and sampling time and data length storage
fPWM = 12e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*2; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

TRGO_double_update = 0;
if TRGO_double_update
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_dab = 1/fPWM_DAB/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_dab = 1/fPWM_DAB;
end
ts_battery = ts_dab;
tc = ts_dab/100;

z_dab=tf('z',ts_dab);
z_inv=tf('z',ts_inv);
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 275e3;
ubattery = 1250;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 1500;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:0994022b]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:60f8fa48]
%[text] ### dead\_time and delays
dead_time_DAB = 0;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ### Grid emulator initialization
grid_emulator;
%[text] ### Nominal DClink voltage seting as function of the voltage application
if (application690 == 1)
    Vdc_bez = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_bez = 750; % DClink voltage reference
else
    Vdc_bez = 660; % DClink voltage reference
end
%[text] ### ADC quantizations
adc12_quantization = 1/2^11;
adc16_quantization = 1/2^15;
%[text] ## HW design and settings
%[text] ### DAB
%[text] #### Input filter or inductance at stage 1
LFi_dc = 400e-6;
RLFi_dc = 5e-3;
%[text] #### DClink input stage or capacitor at stage 1
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 900e-6*5;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*5;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
% single phase DAB
% Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8)

% three phase DAB
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/2) %[output:82d0ee5c]

f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:48d0ef44]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = 5e-3;
rfe_trafo = 1e3;
rd1_trafo = 5e-3;
ld1_trafo = Ls1;
rd2_trafo = rd1_trafo/m12^2;
ld2_trafo = ld1_trafo/m12^2;
%[text] #### DClink Lstray model
Lstray_module = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_module + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi_dc1;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ## Active Front End (AFE)
%[text] ### LCL switching filter
if (application690 == 1)
    LFu1_AFE = 0.5e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (100e-6*2);
    RCFu_AFE = (50e-3);
else
    LFu1_AFE = 0.33e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (185e-6*2);
    RCFu_AFE = (50e-3);
end
%%
%[text] ## Control system design and settings
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
iph_grid_pu_ref = 2.5;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 1;
ki_v_dab = 45;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:7efb82ab]
Iac_FS = I_phase_normalization_factor %[output:2a7b0ed3]

kp_afe = 0.15;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Aresd_nom = eye(2) + Ares_nom*ts_afe %[output:1f9a2a65]
a11d = 1 %[output:99ce4091]
a12d = ts_inv %[output:89491fc7]
a21d = -omega_grid_nom^2*ts_inv %[output:8f588fa8]
a22d = 1 -2*delta*omega_grid_nom*ts_inv %[output:11c418e8]

Bres = [0; 1];
Cres = [0 1];

Ares_min = [0 1; -omega_grid_min^2 -2*delta*omega_grid_min];
Ares_max = [0 1; -omega_grid_max^2 -2*delta*omega_grid_max];

Aresd_min = eye(2) + Ares_min*ts_afe;
Aresd_max = eye(2) + Ares_max*ts_afe;
Bresd = Bres*ts_afe;
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
%[text] ### Single phase pll
freq = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:15b716c3]

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:77afc900]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for voltager grid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:257cd48f]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:05e98ae0]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:609016e2]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:48d2ea22]
%%
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
%%
%[text] #### RMS filter
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] #### Butterworth filter
omega_c = 2*pi*5;
P1 = s^2 + 0.7654*omega_c*s + omega_c^2;
P2 = s^2 + 1.8478*omega_c*s + omega_c^2; 
Hb_flt = omega_c^4/P1/P2; 
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
%[text] ### 
%[text] ## Lithium Ion Battery
typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Pbattery_nom = Pnom;
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
figure;  %[output:88456e2e]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:88456e2e]
xlabel('state of charge [p.u.]'); %[output:88456e2e]
ylabel('open circuit voltage [V]'); %[output:88456e2e]
title('open circuit voltage(state of charge)'); %[output:88456e2e]
grid on %[output:88456e2e]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
heatsink_liquid_2kW; %[output:43e486e9] %[output:222f24c4] %[output:76ecdb95]
%[text] #### SKM1700MB20R4S2I4
% danfoss_SKM1700MB20R4S2I4;
infineon_FF1000UXTR23T2M1;
% Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% Rsnubber = 1/(Csnubber*fPWM_DAB)/5
%[text] ## C-Caller Settings
open_system(model);
% Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
%[text] ## 
%[text] ## Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);
% 

%[text] ## Enable/Disable Subsystems

if use_thermal_model
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
else
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":40.5}
%---
%[output:0994022b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:60f8fa48]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:82d0ee5c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     5.918560606060606e-05"}}
%---
%[output:48d0ef44]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.857555033442859e-05"}}
%---
%[output:7efb82ab]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:2a7b0ed3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:1f9a2a65]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-8.224670334241132","0.998691003061004"]]}}
%---
%[output:99ce4091]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:89491fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     8.333333333333333e-05"}}
%---
%[output:8f588fa8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -8.224670334241132"}}
%---
%[output:11c418e8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.998691003061004"}}
%---
%[output:15b716c3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.031062519151360"],["1.619345474049183"]]}}
%---
%[output:77afc900]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.125263346003807"],["30.829381225835562"]]}}
%---
%[output:257cd48f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:05e98ae0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:609016e2]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-8.224670334241132","0.998691003061004"]]}}
%---
%[output:48d2ea22]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.129590696960579"],["22.638405094998713"]]}}
%---
%[output:88456e2e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdgAAAEcCAYAAABkoKU7AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ+QXlWZ5t\/IlKYRkTR\/DE0TE6DbQR07JBMnRCVSGXBnqxJmmXHSHXe0elunVSKzlaTSCQhVCCadTEItBbj2OE0PjpU\/O7tEyW65QEVkwBD\/ENM7OCgNIQmhRQkhIkucmV2z9d7s6Zw+ufc+937fvfe757vPV2Vh+nvuvef+3vec5zvnnnPulBMnTpwQfkiABEiABEiABDIlMIUGmylPnowESIAESIAEAgI0WCYCCZAACZAACeRAgAabA1SekgRIgARIgARosMyB0hE4evSo9PX1BeUaHh6W1tbW3Mo4NjYmvb29MnfuXBkcHJSWlpbcrmWfuMh7THJDhsMXvvAFWbp0aZJDSq3ZsGGDDA0NBWXs7++XgYGBVOXdvn27rF27VtavX19KHnp\/e\/bsyb1+pIJG8WkEaLBMitIRKNJ8ogxWG7CFCxfK\/Pnzc+ETdY95XzfqZmppsLWBf+yxx1KZVy3HpA2AXmPZsmUThzWjwTbbD6K0MfZFT4P1JVIsZyEEjh8\/LmvWrJGdO3fKli1bCjNY7TkXcd0wiKaxXrx4cWKzND28NOZVyzG1BN0YbJqyudcpew\/W5OmhQ4fYi60lSQo6hgZbEOhGXcYeKtMy2ENedu9t9uzZcvvttwfF1IbWHi41va3R0dHTvrfPcd1118mnP\/3pQNPW1iYjIyPS0dEReuu2kbl6t3en35sh40WLFsmdd94pXV1dQcNiGxM6jw41m+s+9dRTQfn0Y4aIb731VvnSl74UmKv5uCz074apbcCmUbf1ppE257IbfPse77nnHtm4cWPodQ8fPhyUb3x8fKJMdgxtjsr83nvvlfvuu0\/M\/Sl\/l7VhZ4bew8zExNW+rrlf975MrNvb2yd+JLj8HnzwwWDI1Xzs\/HDPh37YuPkYd664PIzr6dpMtMym7K5pu\/XLZmvOsWLFCtm1a5do\/TGxs+9Z\/2auYccWcQnLw0a1M7xuOAEabJNmhtuo2rdpGomwRtRtGPU8am7GXN3vwwwgzpz0u6iymcbw3HPPnfQM1hisXQY1sjBDtE3WPU9WBhvWQ3IbO7fhjeKqf48y2NWrV8vy5ctPYx9naPrd+eefL6+88krwAyLM9PSathG4ZY\/KC3PdvXv3hprlAw88MPHc084320Bcg3XPZb6PMtm4nNVjDh48GGnkdplcc3V\/BBlz+8hHPiKPP\/74pFYizCTD6pdrkKoJK6P+3VwHndvmUvZedpM2raluiwabCpc\/YtOA2L\/gTeOkd2H33rSXYiqu3YDZjYH9y91ukNXETA\/LnMNc2+0pGXph39uNxTXXXBNpsPYv\/LTnQQarvXb9oKHauB629qpfffXV05jYvS7l1NnZOekekwwRu8PXhr2Jp\/ZW3bhrWfR5ZFjPWlkuWbIkuF+7x5tk4leS4V5X4\/7bMDE\/BrT86Nom98Lux\/xNf4jpPUcNEdscTT65MX3kkUcCow7rkUad1x3FML12+xxx1zY9XJP\/iEsWQ+H+tGh+lpQG62fcYKmjft2aBkobljlz5oTOoLU1Bw4cCO2VaAHsc2ivycz4RZOU0C\/vKAOzGxy9ftrzZGWw9rXVLPVjN+hRDZ9tMJ\/5zGcSG2xYjz\/suloOdwg8qoeoWjUKUw6bbdj13KHyOIONGhp3j4nrjYb9OHPvzTx+cI3a\/KiIMkKUn3Z87XNExdXtDRtWxmCjHg3YM+TtXDb10h6eNxXe5hL2WAI2DBQUSoAGWyju4i5WtMHay1xQA5bWGJVa2LKdpOexzcNtjPXc9jKdJD1Y1dgTg\/TfuiTE7cG7DXxag3U5ur1c19izMliTpVHGrjOrwwzWHWpGPVgfDDZsxMTE1c2\/qB6sfY6oukGDLa5tLPJKNNgiaRd4rVqHiN2hTPNMK6o3EDakhww2bmjX7lUpLv2VH2WwSc+jQ2+u+Zmhc9dg1cSSTB5xzcfu4bnD7GpIaIhYe9fIoNxz1DpEbKdhVK\/QTVXXLN3enLlneyTD3I\/JHfeYsCFiVEXyHiI2P8ZMzz\/KYMN6\/oaR24ONmpTmDk\/HDRGHceEQMcqWxn9Pg218DHIpQd6TnOIMChlsXNnCnk9GGSw6jw6nmeepLuQkBqvHhM0iNudyZ4LaGzSkmeRkhgrtY\/S6119\/fdC7Dvsop7D7SzrJSc9pfnS4xh41Acg+xtboNe+66y654447TpuQpce4Bqt\/i5owZe4V\/aALGz5FIwg2x6h7jDNH29BuvPHGyNyKO4eWIWzyU9JJTjYXNIKTS8PCk6YiQINNhcs\/cdJlOvYSG7RMJ2ziVJohYqUYN\/yIJhHZOzvFnUev4y7puOWWW2Tfvn0Tk3rCerB24xs1Ucs+t\/tsOMyAbaOxj9X\/bww27Lpf+9rXJu1IpJtf2M97a1mmYxul3eCHLeGKWh7kcrWfCes5ldumTZtk1apVAQ57JMLMBo9a9oPWr8Yt09FrJe3ZRT071VGMMPOK6rUro7AlUmG94KgfZ\/p3d+eoqGfZ5hxJRlr8a7Gaq8Q02OaKZ6q7QTM2U52M4sIJhA1FuzPFo9Yh24WtZaOJwm+2SS5o\/yAyPyTCZhaj2+VGE4hQOb73wmC1AdA1gboYP6zBCOvFoF\/B5cDf2FLQYBvLv96rxw2Rxw1th123lq0S6y1\/VY8PGyJWFmhzlrAfRc2yd3Sz5kLpDTbJJAwdMlq5cqXcdNNNkTsHNWsA67kvGmw99MpxrDtcqqVKa656DPe2LTae7qObNOaqJeUPomLjVevVSm+w+txDk0k\/UT1YbRzWrVsnmzdvzvXNK7VC5nEkQAIkQALVI1Bqg9Vf57fddpv09PQEJhtlsMaE8361WfXSg3dMAiRAAiRQK4FSG6w+q9CP7mwS9wzWfaYRN\/OzVlA8jgRIgARIgATSECitweqw7\/333y8333yz6IbycQarvVudKm\/eAOP+2wVyySWXpGFELQmQAAmQQAMJ6NuIZs2a1cAS1Hbp0hqs\/eJpNIvYvXWkV4Pdv39\/bcQqchQZ4UCTERlhAljBPGpeRqU02LCZkSYE6F2RqkOTnpjQzZvQ+M6yUzCPMEsyIiNMACt8zaNSGmyaHqlZxrNgwQLRrerMv3Xa+8DAQGjkfA0WTsPsFGSEWZIRGWECWME8al5GXhqsMVGdXawbqcdtvB4WOiY0TugXXnjBy2ce+M6yU5ARZklGZIQJYIWvbbYXBovxp1P4Gqx0d1mfmg0j5kdGZIQJYAXzCDPytc2mweLYVlLBSo\/DTkZkhAlgBfMIM6LBYkalUfgarCIBstJj2mRERpgAVjCPMCNf22z2YHFsK6lgpcdhJyMywgSwgnmEGdFgMaPSKHwNVpEAWekxbTIiI0wAK5hHmJGvbTZ7sDi2lVSw0uOwkxEZYQJYwTzCjGiwmFFpFL4Gq0iArPSYNhmRESaAFcwjzMjXNps9WBzbSipY6XHYyYiMMAGsYB5hRjRYzKg0Cl+DVSRAVnpMm4zICBPACuYRZuRrm80eLI5tJRWs9DjsZERGmABWMI8wIxosZlQaha\/BKhIgKz2mTUZkhAlgBfMIM\/K1zWYPFse2kgpWehx2MiIjTAArmEeYEQ0WMyqNwtdgFQmQlR7TJiMywgSwgnmEGfnaZrMHi2NbSQUrPQ47GZERJoAVzCPMiAaLGZVG4WuwigTISo9pkxEZYQJYwTzCjHxts9mDxbGtpIKVHoedjMgIE8AK5hFmRIPFjEqj8DVYRQJkpce0yYiMMAGsYB5hRr622ezB4thWUsFKj8NORmSECWAF8wgzosFiRqVR+BqsIgGy0mPaZERGmABWMI8wI1\/bbPZgcWwrqWClx2EnIzLCBLCCeYQZ0WAxo9IofA1WkQBZ6TFtMiIjTAArmEeYka9tNnuwOLaVVLDS47CTERlhAljBPMKMaLCYUWkUvgarSICs9Jg2GZERJoAVzCPMyNc2mz1YHNtKKljpcdjJiIwwAaxgHmFGNFjMqDQKX4NVJEBWekybjMgIE8AK5hFm5GubzR4sjm0lFaz0OOxkREaYAFYwjzAjGixmVBqFr8EqEiArPaZNRmSECWAF8wgz8rXNZg8Wx7aSClZ6HHYyIiNMACuYR5gRDRYzKo3C12AVCZCVHtMmIzLCBLCCeYQZ+dpmsweLY1tJBSs9DjsZkREmgBXMI8yIBosZlUbha7CKBMhKj2mTERlhAljBPMKMfG2z2YPFsa2kgpUeh52MyAgTwArmUTyjJ547Jtd\/aZv88uufxTBLpqDBliwgZSkOKz2OBBmRESaAFcyjeEZbf\/iy3LD1GTl659UYZskUNNiSBaQsxWGlx5EgIzLCBLCCeUSDhVly9OhR6evrk9HRUai1BV1dXbJjx45Ux9Qr9nU8v977TnM8Kz2mRUZkhAlgBfOIBguzxBjswMCAzJ8\/H+pVsGfPHtmwYQMNNhGtYkWs9Jg3GZERJoAVzKN4RhseOiAbHnqh2kPENFhckXxSsNLjaJERGWECWME8osHiLPFIwSFiHCxWejLCBLCCeURGmAANFjKyn8H29\/eLDhWX9UODxZFhw0hGmABWMI\/ICBOgwSZmpM9Uh4aGAn1bW5uMjIxIR0dH4uOLENJgMWU2jGSECWAF84iMMAEabGpG7qziMvVqabA4nGwYyQgTwArmERlhAvEKXQOra2G5DjaC09jYmPT29sr4+HgperU0WJzybBjJCBPACuYRGWECNNh6GU0cb5bmDA8PS2tra2bnTXMiGiymxYaRjDABrGAekREmQIOti5Hdg9UTbdmyJfFa2bouHHEwDRZTZcNIRpgAVjCPyAgToMGmZnT8+HFZs2aN7Ny5Mzh28eLFMjg4KC0tLanPlfUBNFhMlA0jGWECWME8IiNMgAabmJE9i7gRvVXtLa9evVo2btwYOXuZBovDyYaRjDABrGAekREmEK9Ycu+P5Ynnj1V7kpM9a7hRvVXTa37qqadilwfRYHHKs2EkI0wAK5hHZIQJ0GAho+eee05uvPFGufXWWxM\/X816L2JzPi0se7AwZLECNoyYHxmRESaAFcyjeEaz73hSDh39DXuw+jadRm32rz3o2267TXp6eoIXCNBgccWOU7DSY35kREaYAFYwj+IZta54NBBUeh1so19Xt3379iAIc+bMSfQM1oR0165duAZUUHH48GFpb2+v4J0nv2UywqzIiIwwgXDFokWL5LdnnievX7uBBlsrxCyO04lN999\/v9x8882iFZqTnOqnyl\/VmCEZkREmgBXMo2hGTzx3TJZ85cc0WJxG+Sl0SHjhwoXBs1\/OIs6GMys95khGZIQJYAXzKJqReRds5YeIcRrlo4gbmo7a0IKziHEsWOnJCBPACuYRGWEC0QqzROctbx6RI1\/9eD2nasixU06cOHGiIVfO6aLswWYDlg0j5khGZIQJYAXzKJyRzhzWGcT6+Z0jP5Vffv1zGGbJFDTYkgWkLMVhpceRICMywgSwgnkUzkjfoKNv0tHP2Q8PyIGnf4BhlkzRdAabhC+HiDElVnoywgSwgnlERpjA6QrtvS7f+kywg9OM1qny+n2fkP3799dyqoYek6vB6tKZtWvXBjeoz0MPHjwou3fvbviexDRYnHNsGMkIE8AK5hEZYQLhBmuGh79w9Qz5u7+8hgZrY9KZvfr+V10ys3z58mADiq6uruAFAG1tbcG\/G\/WhwWLybBjJCBPACuYRGWECkxXae9WlOfpf7b3e0325fPLaOTRYg8nM7FUT7ezsFHuHJ74PNm26NUbPhhFzJyMywgSwgnl0ipE9NKx\/vbv7d+UTH7xQfO0U5TJETIPFlarsClZ6HCEyIiNMACuYRycZqbl++ydHZO2OseDf2nvd98Urg\/9Pg3XySJ+\/6vNWe4jY9Ga7u7tl6dKlOPNyUvgarJxwhJ6WlR7TJiMywgSwgnl0ktFfPXxA1v\/PFybM9cHPXxGYLA02Iod0OHjZsmWTvl2\/fn1DzdXnYOGqmp2ClR6zJCMywgSwoup5ZD9zNT1X21x9brNzGSLGKdVYBXuwmH\/VKz0mJEJGmBIZkVEUATXW7z1\/bGKtqzFXMyxsH+drm02DxflfSQUbRhx2MiIjTAArqphHbq9VKX340nPknp7LJ4aFabARuZP01XX9\/f0NWa7j668hXFWzU1Sx0qelR0aYGBmRkU1A346z8aEXgg0kzMcsxfnwZedEwvK1zc6tB6uTnLZt2ybDw8PS2toagDPGq5OclixZ0rA1sb4GC1fV7BRsGDFLMiIjTAArqpBHaqzLtz0TzBS2jXXp3OnyiT+4MLTXyh4s6MHqOlh9hZz9sdfBPvvss6IbUuzYsQNnYYYKGiyGWYVKjynEK8gIEySj6jIK660qDe2xLuxslZV\/+G5orIaer212Lj1Yex0sDRZXsDIq2DDiqJARGWECWNEseaS9U\/3f1h\/+XHSjfvejxvr3f9ElHReciaE4ChqsAwQNEes6WLNW9q677koNvJ4DfA1WPfec9thmqfRp7zuNnowwLTJqfkbu7kthxqrbHcY9Y0WUfG2zc+nBGlhh62DNS9DVXO+++24ZGRmRjo4OxDfT730NVqYQwMnYMGLaZERGmABW+JZHaqj\/addB+dsnxyNvzkxc0v+azSIwiWiFr212rgZbD9A8j\/U1WHkycc\/tW6Uvko25Fhlh6mTkLyOz2b7+d9MjB+Qb3\/95rKHOmDY1WGajnyxM1b6Yr202DRbnfyUVbBhx2MmIjDABrChLHpkZvseO\/6t88ZvPTVpKE3YXaqLLPzpDfnf62+sa\/sWEuBfxaYzGxsakt7c3eGWd+9HX1tnLd5IAzlLj66+hLBmgc5Wl0qNyNvJ7MsL0yai8jHSWb9s73yr\/8b\/8DJqp6ZX2zLtQeuZNz7yHiij52mbn0oM9fvx4sMZ1wYIFE+tde3p6Tnt1HYKa1\/e+BisvHmHnZcOIaZMRGWECWFFEHmnv9L\/u\/YV892dHE5upDvn+5aJ3B7N+sx7yxVQmK3xts3MxWHeZjq51nTlzZrDJv0582rp1qwwODkpLS0tazpnofQ1WJjef8CRFVPqERSmtjIxwaMioWEbaK33H1DPklm89J4deO7lsBn2MeX5+4cXyb953XsPNNKy8vrbZhRiszhg+cOBAsC0iX7iO0r0c37NhxHEgIzLCBLCiljxS49z74uty3xMvJTZSM8yrPdM\/+\/3pclXHtKBwje6dYkJ8BnsaI+216sc11UceeSR4Tyx7sEnSqnGaWip940rbmCuTEeZORrUzsjduePHobxIN7ZqrGdPUDfS7553citAHI42ixR6sQ8Z+DqtDw2q4Q0ND0tbW1pC1r3bxfA0WrqrZKdgwYpZkREaYQLxCh3Rffvnn8g8vTZEDR46nMlG793nte8+VJR+4YMJEfTZTDhHXm1UNPp4GiwNA8yAjTAArmEciaqJnTz1Dhh4\/LGl7onaPVId2r+qcJn82d7o3Q7s4Q5IpfG2zC3kGayPkM9hkCdVoFRtGHAEyIiMlYIZyH9j3C3nuF2+meiZqEwyGcadNlfdMf7tc13VB8FU92wvi6PijoMFasUq62b95jV3RYfY1WEVyonlg2mRUDUbGQHWZy\/5XajdQM6SrJjr\/knMmJhlddMZrMmvWLAyzwgpf2+xMe7A6W3jt2rUwDRr1onVTMF+DBcFmKKB5YJhk5D8js4xl76HX5ZFnXg2GcJMubwm7e\/Ps80OXniM3fHSGnPW2M+BwLvMI55GvbXamBmswxfVgMcr8Fb4GK38yp67ASo9pk1G5GRnzPCEi3\/j+uHx\/\/6\/qMk\/TA9X\/qoHqs9Az3jKl7qFc5hHOI1\/b7FwMFuNqrMLXYBVJjZUe0yajxjIyQ7f\/\/R9fkX8afyMozBPPH8OFilGY56CXnn+m\/MVH2uXt\/78HaptrXRcIOZh5hIn62mbTYHFsK6lgpcdhJ6PsGRnTVKN77pU35Zv7fhksX6ln2NaU0gzf6jPQP559gXS+6+2lWNbCPMJ5VHmDNcPCo6OjkBY3+4eIGi5gpcchIKP0jIyBPj3+hnz76SNy4sSJzM3zA+3vCLb8048Ps3CZRziPKm+wGFF5FL4Gq0iCrPSYNhmdYqRrPbWHeOSNf5H7do\/LoVez6XXaQ7Nm9u2\/\/4MLJwXH900VmEe4rvnaZnOIGMe2kgpWehz2ZmdkbxSvzza\/99xrdc+ydamaZ54Xt04N1n6e+daTs26152le+I0j4bei2fMoi+jQYEMohi3bWb9+ffBWnUZ+fA1WkcxY6TFtnxmZodoXjhyXpw69Xvf6zjDj1L+d3yJy7e9dKFdecs6kvXB973Xi7Eiu8DmPkt9lfUpf2+zcerBqrtu2bZv0YnXznLa7u7uhJutrsOpL0XRHs9JjXmVjZEzzonPeJv\/j6SPy8E+OBDeRxQQhQ8OeKPTetrNEX3Fmerphvc6yMcJRLV5BRpi5r212LgbLnZxwwpRdwUqPI1QEI3tW7e7nj8njOQzT6p3axvnBWe+Uj3a2BgDsnmYtvc4iGOFIlVtBRjg+NFiLEQ0WJ0zZFaz0OEK1MrJN8x9fekO+\/ZMjwaQg\/dS7jtMutW2a+ozz6s5Wmf7Ot9VtmpjMKUWtjNJcw3ctGeEI0mAdRhwixklTZgUrPY6OzcieEDT+q3+Wb3z\/57mYptvbnD3jbFnYMU06LjhzYlJQLT1NfLe1KZhHmBsZYUY02BBGnOSEE6esClb6k29JMZ8fHXxddv20\/r1qo+Jt9zZ1A4QPzjpb5s86NZNWv\/dxVi3zCNdwMsKMaLCYUWkUvgarSIDNWunNek1lue1HL8vBV49nvvTExMk2zUvOP1P0pdjvbztrUk\/TR9NMk4fNmkdpGCAtGSFCIr622blOcmr0bOGosPkaLJyG2Sl8qfT288yxX74pui\/t8798MwCR5ezZMNM8b+r\/laVXzpLLp799AryvPc3sMmfymXzJo7zuP8l5yQhT8rXNzsVgFZc7PLxlyxaZP38+JlmAwtdgFYBm4hKNrPTay9SPmtU\/jL0mT+4\/llsv01wn+O+0qXLZu86UZfMulN\/862+D65teaFhPs5GMisyFeq5FRpgeGWFGvrbZuRmsjWzDhg0yNDQU\/KmtrU1GRkako6MDU81J4WuwcsIRetqsK70Zmn359X+Wnf\/rFRl98deF9DKnTJkiV1z8Dun90EWBSSPTTMM4a0Zpru2LloxwpMgIM\/K1zS7EYF2z3bNnz6QNKDDebBW+BitbCvFnQ5XeHpr9wYFfyaM\/O1pYL1MnAXVd\/I5g9qzpWTZiaBYxKjJeZb0WGeHIkBFm5GubXYjB2j3YRr9JR0Ppa7BwGtansE3zvu\/8VPb+Ql9Vnf+zTF2jefG0qdI9b7q8ZcqUhppmGoJsGDEtMiIjTAArfG2zczPYeoeFjx8\/LmvWrJGdO3cG9Pv7+2VgYCA0Eq4W6X0NFk7DcIW93OQrj72Y2cup3avZs2ZnntciV7+nVebOOHuSrExrNGvlaY6jeWCCZERGmABW+Npm52KwcTs5YZQnFWrQ+lFTRXsY6\/crV66Um266KdGzXV+DFcdOn3G2n\/M2ufu7L8rYL\/53JjsC2Yap1772fefJ7110lsw6t6WUmxokza2sdDQPTJKMyAgTwApf2+xcDBbjSq+wDdc9emxsTNatWyebN2+W1taTe6jGfXwNlt6T9kb1ZdVffezFmk3UNs7LLzxLruqYFhin\/TyTDSPKIhEyIiNMACuYR5iRr222FwaLesQ6aUoNeHh4uOkMVnumGx96IZWZmndszjy3RT7++9NlSg3v12Slx5WejMgIE8AK5hFmRIPFjGpSmGe5ixcvlsHBQWlpaTntPO6aWzSRSoNlPrt27aqpXHkd9NRLv5Gdz7wR\/C\/u03b27wRfL778LJl70VS58B0n\/23+Xm\/5Dh8+LO3t7fWepqmPJyMcXjIiI0wgXLFo0aJJX+zfv7\/WUzXsOC96sEpHjXZ8fDzUZN3v4rR6rrL9GtJe6tYf\/ly2\/vDl0EQwQ7p\/8+fvCzZA0Pdu5v3hr2pMmIzICBPACuYRZlS2NhuX+KQiF4PN43V1+px19erVsnHjRjiRCWnLEiw11uXbnpm0qbwJnJrq3d2Xy7ut3YSSBjULHSs9pkhGZIQJYAXzCDMqS5uNSzpZ4Y3BpnnOiiY9NTJYZsnMkq\/8+DRjVVO9p\/vyQnqoKFFY6REhTnLChMiIjJIQwJpGttm4dNGKTA027PV0YZeOW9Nq9PasYbPOVbdZdNfCmu8WLFggS5culTitOXejghXWY1VTvW3xZcF2fmVaI0qDxdWKjMgIE8AK5hFm1Kg2G5csXpGpwZpLoVm\/SQrtbh5hT3Iy3\/X09AQvEIjThl2r6GBpr\/XbTx+Rtd8cmyhOmXqrYYxY6XGWkhEZYQJYwTzCjIpus3GJkilyMdhkl26cquhgzb7jyUnDwUOfeK98fO67GgcgwZVZ6TEkMiIjTAArmEeYUdFtNi5RMgUNNhmnmlQ6JKzPWs1He60Pfv6KUg0FR90YKz0OORmRESaAFcwjzKjyBmsPC3d2dkpfX5+Mjo6GkkPrVDHu+hRFBEuX3Nyw9ZmJgvZ96CL5qz\/prK\/gBR7NSo9hkxEZYQJYwTzCjIpos3Ep0ivYg03PDB7hmqv2WotYuwoLlkLASo9hkREZYQJYwTzCjGiwmFFpFHkGyx4WLvtEpriAsNLjdCUjMsIEsIJ5hBnl2Wbjq9euyKUHa4aLqzZErLOFdUKT+fjYczVlZ6XHlYqMyAgTwArmEWZEg8WMgtfOpXmtXIJT1iTJI1hqrvbmET6bq0JlpcepRUZkhAlgBfMIM8qjzcZXrV+RSw82rli6I9PWrVsjN+6v\/5bwGbIOlprr8q3PTLzx5m8++T65fvYFuCAlVrDS4+CQERlhAljBPMKMsm6z8RWzUTTEYNO8Wi6b25x8lqyDZQ8N63PXfV+8Mo9iF3pOVnqMm4zICBPACuYRZpR1m43oiGcjAAAXx0lEQVSvmI2icINFb7rJ5rbiz5J1sFpXPBpc0Kd1rogzKz0ixGF0TIiMyCgJAazJus3GV8xGkYvBxk1y0v2ER0ZG4Btxsrm98LNkGawl9\/54YmjY9+euNi0aLM5AMiIjTAArmEeYUZZtNr5adopcDFaLF7UJv9mUP7tbSH+mrIJlL8n58KXnyIM3XJG+MCU9gpUeB4aMyAgTwArmEWaUVZuNr5StIjeDDRsKNj3b7u7u4M03jfpkFSyzx7DP612jYsBKj7OTjMgIE8AK5hFmlFWbja+UrSIXg0UvXG+GWcT2xKatn\/6AfOy952YbmQafjZUeB4CMyAgTwArmEWZEg7UYIYP1fRZxM84adlOclR5XejIiI0wAK5hHmBEN1mLkPn+18elL2Xfv3u31Olh7r2FdklOmF6XjVE2mYKXHnMiIjDABrGAeYUY0WIeRbiixatWqSTOGx8bGpLe3VzZt2hS8KL1Rn3qCVYXeq8aFlR5nJxmRESaAFcwjzKieNhufPT9FLs9gTXHVZJctWzap9Fu2bGmouWph6gmWPXP43p7LpWfe9Pyi08Azs9Jj+GRERpgAVjCPMKN62mx89vwUuRpsfsWu78z1BMuse22WHZuiSLLS4xwjIzLCBLCCeYQZ1dNm47Pnp8jFYM0z2J6enob3VsPQ1Rosu\/fa\/5F2Wf\/vOvKLTIPPzEqPA0BGZIQJYAXzCDOqtc3GZ85XkYvBxs0izvd2kp291mD96dCofOdnR4OLNNOuTWHUWOlxLpERGWECWME8woxqbbPxmfNV5GKwWuQyzBaOQldLsKoyuckwY6XHFY+MyAgTwArmEWZUS5uNz5q\/IheDbcYXrttLc5p5chMNNnmlY8OIWZERGWECWEGDxYxKo6glWFWZ3ESDTZ6mNA\/MiozICBPAilrabHzW\/BW59GDzL3Z9V0gbLHt4eGHHNNnxudn1FcCDo9kw4iCRERlhAljBPMKM0rbZ+IzFKDIzWHtiU2dnp\/T19cno6GjoXXR1dcnw8LC0trYWc5fOVdIGyx4ebvbJTezBJk9JNoyYFRmRESaAFWnbbHzGYhSZGWwxxc3mKmmDVbXhYaXMhhHnGhmRESaAFcwjzChtm43PWIwiN4NtlvfB2sPDzfbO17gUY6XHFZCMyAgTwArmEWZEg3UYNdP7YFtXPBrcXVWGh9mDxRWejMgoGQGsosFiRjRYixF6XZ1P74Ot4vAwzQNXeDIio2QEsIoGixnRYFMYrC\/vg63q8DDNA1d4MiKjZASwigaLGdFgLUbN8j7Yqrw5Jyy9WelxpScjMsIEsIJ5hBnRYB1GzfA+2D8ZGpVHK7L3sJvirPS40pMRGWECWME8woxosCGMfH8fbFWfv3L4E1d4MiKjZASwigaLGdFgMaPSKJIEy37+uuqamXLTH80qTfmLKAgrPaZMRmSECWAF8wgzStJm47MUr8htHWzxt5L8ikmCVcXdm2yCrPQ4n8iIjDABrGAeYUZJ2mx8luIVNNgI5mZ4WL\/e98UrZUbr1OKj08ArstJj+GRERpgAVjCPMCMaLGZUGkWSYM2+40nRYWI1VjXYqn1Y6XHEyYiMMAGsYB5hRknabHyW4hXswYYwt5+\/rvzDd8vN\/\/aS4iPT4Cuy0uMAkBEZYQJYwTzCjGiwmFFpFChYtsFWaXtEO0Cs9DhdyYiMMAGsYB5hRqjNxmdojCK3HuzY2Jj09vbK+Pj4aXdW9tfVVf35qwaMlR5XSDIiI0wAK5hHmBEN1mJkdnJqa2uTgYEBTK9gBQpWlde\/mlCw0uOkJCMywgSwgnmEGaE2G5+hMYpcerBxm\/035jYnXxUFy7w9p0qvp3PjwkqPM5WMyAgTwArmEWaE2mx8hsYocjFY04Pt6emR+fPnN+bOYq4aFyx7\/+GqPn\/lEHGylGXDiDmRERlhAlhBg3UY6TaJjX5rTlTYaLA4odkwkhEmgBXMIzLCBLCCBmsxMkPEo6OjoeSynuRkesw7d+4Mrtff3x\/77DcuWHz+ejJkbBhxpScjMsIEsIJ5hBnRYDGj3BTaU9aPTqgy5t7d3S1Lly4NvWZcsKq+wYQBxkqP05WMyAgTwArmEWZEg8WMClPYhht20ahgVfkF6y4nVnqcrmRERpgAVjCPMCMabAij7du3y9q1a4NvtmzZIgcPHpTdu3fL4OCgtLS0YKo1KJLMYI4KVpVfsE6DTZ9sbBgxMzIiI0wAK2iwDiPtReomE6tXr5bly5cHw7f67HXNmjWS1\/pYvebQ0JAsXrw41sQ1WOaza9euif\/\/198\/JkM\/OBb8+6+vny5zL6rWBv92CA8fPizt7e048yusICMcfDIiI0wgXLFo0aJJX+zfv7\/WUzXsuFyW6di9yM7OTunr6wsMVpfsFDG72Jh7VE856tcQJzidykP2PHCdJCMywgSwgnmEGbEHazFqtMHqNo3ac964caN0dHScFr2oYHGCEw0WV3UyIqM0BLCWBosZ0WAdRvr8VZ+32kPEpjcbN8MXo8YK1EsOCxYnOE3mykqP84yMyAgTwArmEWZEgw1hpEa3bNmySd+sX78+cvkMxhyusGcNJ9kHOSxY9gSngY\/NkoGPzay1OE1xHCs9DiMZkREmgBXMI8yIBosZ5aZwN5pIMsnJfWC+9Ycvyw1bnwnKWOUtEk2QWOlxupIRGWECWME8woxosJhRaRRhwVJzVZPVz74vXikzWqs7g1gZsNLjdCUjMsIEsIJ5hBnRYEMY2etgzde6HrbRLwAICxZnEE8OICs9rvRkREaYAFYwjzAjGqzDSM1127ZtMjw8LK2trcG3SbYxxKjrV4QFi6+oo8GmzSw2jJgYGZERJoAVNFiLUdxuSmiGL0Zdv8INlj2DuGfedLm35\/L6L+L5Gdgw4gCSERlhAljBPMKMaLAeGyxnEJ+e4Kz0uNKTERlhAljBPMKMaLAOI+2prlq1SkZGRiY2eyjrEDFnENNgcRUnIzKqhQA+hgaLGdFgQ3qwUe+DtXHq\/sQ7duzAhDNUuMHa8NAB2fDQC8EVOIP4JGhWepxwZERGmABWMI8wIxosZlQahRssziBm76yW5GTDiKmRERlhAlhBg8WMSqNwg2X2IP7wpefIgzdcUZpyNrIgbBgxfTIiI0wAK5hHmBENNoRR2DrYPLZKxOGZrLCDxT2Iw+mx0uOsIiMywgSwgnmEGdFgHUa+rIO1DZZ7EJ8KIis9rvRkREaYAFYwjzAjGqzFyKd1sPYSHe5BTIPFVZ2MyCgNAaylwWJGNFhPDdaeQUyDpXngqk5GZJSGANbSYDEjGqynQ8T2Jv9H77waR7oiClZ6HGgyIiNMACuYR5gRDTaEkQ+TnLhEJzy5WelxpScjMsIEsIJ5hBnRYDGj0ijsYHGJDg221sRkw4jJkREZYQJYQYPFjEqjMMHiEp3okLBhxOlKRmSECWAF8wgzosFiRqVRhBksl+hMDg8rPU5XMiIjTAArmEeYEQ0WMyqNwgSLS3TYg60nKdkwYnpkREaYAFbQYDGj0ihMsLhEhwZbT1LSPDA9MiIjTAAraLCYUWkUJlj2Eh2+RYdDxGkTlOaBiZERGWECWEGDxYxKozDB4hId9mDrSUqaB6ZHRmSECWAFDRYzKo2CBotDwYaRjDABrGAekREmgBU0WMyoNAoTrNYVjwZl4mvqTg8NG0acrmRERpgAVjCPMCMaLGZUGoUG67s\/+ifRTSb087mFF8uXr7usNOUrQ0FY6XEUyIiMMAGsYB5hRjRYzKg0Cg3W1x\/eK0u+8uOgTFwDyx5sLcnJhhFTIyMywgSwggaLGZVG4Ros36JDg60lOWkemBoZkREmgBU0WMyoNAoNVv9\/\/o5seOiFoExcokODrSU5aR6YGhmRESaAFTRYzKg0ChosDgUbRjLCBLCCeURGmABW0GAxo9IoNFjvX\/nf5Innj8mM1qlBD5afyQTYMOKMICMywgSwgnmEGdFgMaPSKGiwOBSs9GSECWAF84iMMAGsoMFiRqVRaLCO\/fFwUB6ugQ0PCxtGnK5kREaYAFYwjzAjGixmVBrFzPd\/UF6\/dkNQnp550+XenstLU7ayFISVHkeCjMgIE8AK5hFmRIPFjEqjsA12x2dny8LOaaUpW1kKwkqPI0FGZIQJYAXzCDOiwWJGpVHM+OAfyRsfXh2UR3uv2ovlZzIBVnqcEWRERpgAVjCPMCMaLGZUGkX7R\/9c3pzzH4LycJOJ8LCw0uN0JSMywgSwgnmEGdFgMaPSKOweLA2WBltrYrJhxOTIiIwwAaygwWJGpVFM\/9M75F9mfCgoD3dxosHWmpg0D0yOjMgIE8AKGixmVBrFBZ\/8qvyf897DTSZiIsKGEacrGZERJoAVzCPMiAaLGZVGQYPFoWClJyNMACuYR2SECWAFDRYzKo3ivM\/+vfz2zPO4yQR7sHXlJM0D4yMjMsIEsIIGixmVRtG64tGgLNzFKTokbBhxupIRGWECWME8woxosJhRaRTGYPmidRpsPUnJhhHTIyMywgSwggaLGZVGYQyWm0zQYOtJSpoHpkdGZIQJYAUNFjMqjYIGi0PBhpGMMAGsYB6RESaAFTRYzCiV4ujRo9LX1yejo6PBcYsXL5bBwUFpaWk57TzHjx+XNWvWyM6dOye+6+\/vl4GBgdBrGoPlJhPswaZKSkdM88D0yIiMMAGsoMFiRokVxjAXLFggS5cuFfPvtra2UNNUM165cqXcdNNN0tHRAa9jDJabTNBgYbLECGgemB4ZkREmgBU0WMyoLsX27dtl9+7dob3YsbExWbdunWzevFlaW1vhdYzBHr3zaqitqoANI448GZERJoAVzCPMiAaLGdWliDPYPXv2yIYNG2R4eDixwc5onRpsk8hPOAFWepwZZERGmABWMI8wIxosZlSzwjyP7e7uDoaM3Y+a79q1ayf+3NXVFWu22oN9y5tH5OyHB2TXrl01l6uZDzx8+LC0t7c38y3WfW9khBGSERlhAuGKRYsWTfpi\/\/79tZ6qYcdNOXHixImGXT3Bhc3zV5VGTXLS3uv4+PjE9+6\/3cuowXKTiXj4\/FWNk5OMyAgTwArmEWbEHixmlFqRxFzDTqrPZFevXi0bN24MnfQUGOxl5wTvguUnnAArPc4MMiIjTAArmEeYEQ0WM0qlQDOH406GJj2pwXIXp\/hw+JrQqZKsTjEZYYBkREaYAFb4mkelHSJGw7wmJGmX9OhxNNjmTWh8Z9kpfK302RHAZyIjMsIEsMLXPCqlwbqbTBj8ZvKSbjahG0v09PTI\/PnzJ9bJmo0m4jalMAZ75t775K2HvocjSwUJkAAJkEDDCXCSU8NDwAKQAAmQAAmQQDkIlLIHWw40LAUJkAAJkAAJ1E6ABls7Ox5JAiRAAiRAApEEaLBMDhIgARIgARLIgQANNgeoPCUJkAAJkAAJ0GCZAyRAAiRAAiSQAwEabA5QeUoSIAESIAESqJTB6uYVQ0NDQdS3bNkSrKGt6kd3u+rt7Q32cEbrhm1u+k7ekZGRRO\/d9Z1tGkbmXt2NT3xnkKT8aTi5a9yrUg\/TMLK1Vapvcblm6pXZ+yBJXpZBUxmDtV9p9+yzz6Z6vV0ZApVlGWwTWLJkSbBph3m5vXsd9zWB+u9t27YlfjVgluUu8lxpGNnlMm92Wr9+feibn4q8hyKulYaTu\/0p2jO8iPIXcY00jMwPkIGBgaADUJX6lsRcdSMh336QVcZgtRemH01cX38NZdUYuA2b\/vjYunVr5NuK7OtWpVGshZE2jitXrpRjx45J1KsVs4phWc6ThhPaI7ws95R1OdIysl9UUpX6FsXc9Obnzp0rhw4dCtpvn0YeK2GwUfsVR\/Xasq5gZTuf+4L6NC+sr0qFr4WR\/oibN2+efOtb34ocEShbLtRbnjSc3NGQeq\/ty\/FpGIX1YHfv3p3ox68vPNKU86WXXgrkuj1uX18fDTYNvKK0YT1WbQxnzpxZiWE8l7PbY03Ts0j6EoaiYpvXddIyUob333+\/rFixQm677bZKGaw9+hGXS2qwBw4cCEJWpbkQaXPJtFc6JNrf3x+YStU\/7g8PX3hUqgdrPyCnwZ4aEk5qsNpA3n333ZWY5JSmUdQG8ctf\/rJ86lOfkvb29thn2r40DEnLmYaTeT5tnqPpsatWrWr6fErDyAyJbtq0KRgKTTO6lDRmPuposCWOGoeIJwcnzZCVObJK5qr3nIaRah977LFJz\/er8vghDSd3iLgqM67JqH5zoMHWzzDXM9g9Vk5yGpN169bJ5s2bpbW1NTCTuElOVZzJ6Pbq4xjZy5jsJK7C8F4aTi7DqtTDNIyq+iMENf40WESowd9zmc6pAKRZNlCVYTw3PdMwso+tSq\/M3HMaTm4jWZXhzzSMwoaIqzCMjuyBBosIleB7bjRxKghxC9\/NZBSdXBHVO\/NtPVot6ZeUUZUNVu89DSd7o4kqbaKQhpH+8Fi2bFmQVlViFFdHabC1tGA8hgRIgARIgASalEAlZhE3aex4WyRAAiRAAiUmQIMtcXBYNBIgARIgAX8J0GD9jR1LTgIkQAIkUGICNNgSB4dFIwESIAES8JcADdbf2LHkJEACJEACJSZAgy1xcFg0EiABEiABfwnQYP2NHUuegsDzzz8v06ZNC3auSvLRdXevvfaaXHrppUnkNWnMGuOurq5U79f15Y1G5v6KWstZ9PVqCjoPqhQBGmylwl3Nm027Y1ARi9rrMcl6ji0yA+x3MBd1XV\/YFMWD12ksARpsY\/nz6gUQKKPBpi2TjckXE6HBFpDcvESpCdBgSx0eFi4pAXsLPj3GDLs+++yzE9vO6d\/NFo\/uFpBmGPPcc88NXuw8OjoaXNps2G+\/o9M+f9yQs30Ne5jUvLbN3Nv69etD30scpTMGu2TJErn99tuD07jDsPZ2e+51lNXKlSvlqquuCo43rF599VXp7e2V8fHx4JBbbrlFHnzwQdm4caN0dHQEf4u6p7A4uQar\/\/71r38d\/E\/fdWrzDTs+7AXt6KXtvvz4SJrX1PlNgAbrd\/xYehEJ22Dfbtzd3mLUG0sU5uDgYHA+NVndi1nfyWnMu7u7e8II494wZMpjztfS0iLu6\/5QD9bV25vA648ANcK5c+cG5TXn37ZtW\/AsV41y9erVk4zRPp\/5ETFjxoyJ4917NP9+5ZVXgve1mvfcqpGbF4CjF0GEGay+aN38oAjjaic0DZbV23cCNFjfI8jyhxqgjQWZmWrtxtw12LBX1cW9NSesF+Xq48qE3sjjvnFFy496bvb3xmDdHwy7d++eMFw9p22g+m\/7FYeGb9wwcJjBau\/Y\/Cgw11Cd\/jBwRwNosKzcvhOgwfoeQZY\/IGAPp7qzcqPMzB1GXbx4cWgP1h2qtZGHDe9GXc9+S1GcwaJJVmFmGvY3d9jcHQY3PXQz9Kv\/Nb1T17S1V2ze8OKmXNR7b8MMNu4aZhjanJ8Gy8rtOwEarO8RZPknEbBNxX4Oa\/eSjGG6z0VND87tweqxbs8rDnuUecYNW9vnq9dg9VzmWar5ARDWg01jsHv37hUzBJ10qRMNlpWz6gRosFXPgCa9f9ukTA9NhyH1eeWaNWtkwYIFkyYW2SbqGmzc89YwfEUMEbvPWO1rqhnGDfeaIWLbYMN6i\/YQsfZg0774O48hYvRjBw2VN2m687ZKSoAGW9LAsFjJCYRNlrF7kfakn7DJOqZHa4aI9cq2CZvz64QnM4Qa9hzUlDirSU52j9F+LjtnzpzTJjG5Bmsfa8qq5dMJS2EGm3SSk57DPENFk5TqneRkhvDNzG9zH\/bkLjdLaLDJ6w2V+ROgwebPmFcogIBpfM0SE3v4115io0Om11xzzaSlOGqs1113ndx6662BAemzQNd0Ta\/WLN\/RWzINf9TtxS1pSTrxau3atROnDxvuNc8tXWNxr71p06ZgiY1ObDL3b\/dg9SIuQ3eZjrtUSY+JWmJkRg30v+ZHSdgyHfv4MHO0n39rnGbPni379u0LTN79IWTuwe3dF5B+vAQJhBKgwTIxSIAEQgmo4YXNHE6KK8kz2KTnSqpjDzYpKeqKIECDLYIyr0ECJSfgPmc2vVV73WvaW6DBpiVGfbMRoME2W0R5PyRQIwF3d6uo5TdJT+9uvv\/AAw8Eh9pLgZKeK4mOm\/0noURNkQT+HwWG9IxqQmDlAAAAAElFTkSuQmCC","height":284,"width":472}}
%---
%[output:43e486e9]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:222f24c4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:76ecdb95]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
