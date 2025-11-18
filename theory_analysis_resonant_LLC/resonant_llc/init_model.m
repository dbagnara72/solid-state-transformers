%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

model = 'dcdc_llc';
rpi_enable = 0;
rpi_ccaller = 0;
%[text] ### Settings voltage application
application400 = 0;
application690 = 1;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 5e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*4; % PWM frequency 
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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:7c09ef44]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:076b441a]
%[text] ### AFE simulation sampling time
dead_time_DAB = 0;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ### Nominal DClink voltage seting
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
%[text] ### DAB dimensioning
LFi_dc = 400e-6;
RLFi_dc = 50e-3;
%[text] #### DClink, and dclink-brake parameters
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 900e-6*4;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*4;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
% LLC
fres = fPWM_DAB;
Ls = (Vdab1_dc_nom^2/(2*pi*fres)/Pnom*pi/8) %[output:02159090]
Cs = 1/Ls/(2*pi*fres)^2 %[output:5d197f8f]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = 1e-3;
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
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
iph_grid_pu_ref = 3.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:53fa3074]
Iac_FS = I_phase_normalization_factor %[output:215718df]

kp_rpi = 0.25;
ki_rpi = 18;
kp_afe = 0.25;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Ares_min = [0 1; -omega_grid_min^2 -2*delta*omega_grid_min];
Ares_max = [0 1; -omega_grid_max^2 -2*delta*omega_grid_max];
Bres = [0; 1];
Cres = [0 1];
Aresd_nom = eye(2) + Ares_nom*ts_afe;
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
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:0bd29de1]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:915851da]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:46c059b7]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:7459e256]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:4579d87c]
%[text] ### 
%%
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;

kp_rc_pos_grid = 0.35;
ki_rc_pos_grid = 35;
kp_rc_neg_grid = 0.35;
ki_rc_neg_grid = 35;
%%
%[text] ### Settings for First Order Low Pass Filters
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
freq = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:74e07718]

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Lithium Ion Battery
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

R0 = 0.035;
R1 = 0.035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:0825f5ce]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0825f5ce]
xlabel('state of charge [p.u.]'); %[output:0825f5ce]
ylabel('open circuit voltage [V]'); %[output:0825f5ce]
title('open circuit voltage(state of charge)'); %[output:0825f5ce]
grid on %[output:0825f5ce]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:4e40d541] %[output:563fc97c] %[output:6761ce4b]
%[text] #### DEVICES settings
danfoss_SKM1700MB20R4S2I4;

dab.Vth = Vth;                                  % [V]
dab.Rds_on = Rds_on;                            % [Ohm]
dab.Vdon_diode = Vdon_diode;                    % [V]
dab.Vgamma = Vgamma;                            % [V]
dab.Rdon_diode = Rdon_diode;                    % [Ohm]
dab.Eon = Eon;                                  % [J] @ Tj = 125°C
dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab.Eerr = Eerr;                                % [J] @ Tj = 125°C
dab.Voff_sw_losses = Voff_sw_losses;            % [V]
dab.Ion_sw_losses = Ion_sw_losses;              % [A]
dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
dab.Rtim = Rtim;                                % [K/W]
dab.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
dab.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
dab.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
dab.Lstray_module = Lstray_module;              % [H]
dab.Irr = Irr;                                  % [A]
dab.Csnubber = Csnubber;                        % [F]
dab.Rsnubber = Rsnubber;                        % [Ohm]
% dab.Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% dab.Rsnubber = 1/(Csnubber*fPWM_DAB)/5

% infineon_FF1000UXTR23T2M1;
% inv.Vth = Vth;                                  % [V]
% inv.Rds_on = Rds_on;                            % [Ohm]
% inv.Vdon_diode = Vdon_diode;                    % [V]
% inv.Vgamma = Vgamma;                            % [V]
% inv.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv.Eerr = Eerr;                                % [J] @ Tj = 125°C
% inv.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv.Rtim = Rtim;                                % [K/W]
% inv.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
% inv.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
% inv.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
% inv.Lstray_module = Lstray_module;              % [H]
% inv.Irr = Irr;                                  % [A]
% inv.Csnubber = Csnubber;                        % [F]
% inv.Rsnubber = Rsnubber;                        % [Ohm]

infineon_FF2400RB12IP7;
inv_t.Vth = Vth;                                  % [V]
inv_t.Vce_sat = Vce_sat;                          % [V]
inv_t.Rce_on = Rce_on;                            % [Ohm]
inv_t.Vdon_diode = Vdon_diode;                    % [V]
inv_t.Rdon_diode = Rdon_diode;                    % [Ohm]
inv_t.Eon = Eon;                                  % [J] @ Tj = 125°C
inv_t.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv_t.Erec = Erec;                                % [J] @ Tj = 125°C
inv_t.Voff_sw_losses = Voff_sw_losses;            % [V]
inv_t.Ion_sw_losses = Ion_sw_losses;              % [A]
inv_t.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv_t.Rtim = Rtim;                                % [K/W]
inv_t.Rth_switch_JC = Rth_switch_JC;              % [K/W]
inv_t.Rth_switch_CH = Rth_switch_CH;              % [K/W]
inv_t.Rth_switch_JH = Rth_switch_JH;              % [K/W]
inv_t.Lstray_module = Lstray_module;              % [H]
inv_t.Irr = Irr;                                  % [A]
inv_t.Csnubber = Csnubber;                        % [F]
inv_t.Rsnubber = Rsnubber;                        % [Ohm]
% inv_t.Csnubber = Irr^2*Lstray_module/Vdc_bez^2
% inv_t.Rsnubber = 1/(Csnubber*fPWM_INV)/5

infineon_FF1800R12IE5;
inv.Vth = Vth;                                  % [V]
inv.Vce_sat = Vce_sat;                          % [V]
inv.Rce_on = Rce_on;                            % [Ohm]
inv.Vdon_diode = Vdon_diode;                    % [V]
inv.Rdon_diode = Rdon_diode;                    % [Ohm]
inv.Eon = Eon;                                  % [J] @ Tj = 125°C
inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv.Erec = Erec;                                % [J] @ Tj = 125°C
inv.Voff_sw_losses = Voff_sw_losses;            % [V]
inv.Ion_sw_losses = Ion_sw_losses;              % [A]
inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv.Rtim = Rtim;                                % [K/W]
inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
inv.Lstray_module = Lstray_module;              % [H]
inv.Irr = Irr;                                  % [A]
inv.Csnubber = Csnubber;                        % [F]
inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = Irr^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(Csnubber*fPWM_INV)/5
%[text] ## C-Caller Settings
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

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.8}
%---
%[output:7c09ef44]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:076b441a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:02159090]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.775568181818182e-05"}}
%---
%[output:5d197f8f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     3.566505664210290e-06"}}
%---
%[output:53fa3074]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:215718df]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:0bd29de1]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["2.831309534039184e-01"],["6.766822226281998e+01"]]}}
%---
%[output:915851da]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:46c059b7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:7459e256]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.000000000000000e-04"],["-1.973920880217872e+01","9.968584073464102e-01"]]}}
%---
%[output:4579d87c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["3.110176727053895e-01"],["5.433217222799691e+01"]]}}
%---
%[output:74e07718]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["7.338637605205141e-02"],["3.802432508328568e+00"]]}}
%---
%[output:0825f5ce]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjMAAAFTCAYAAADMTo0FAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQm4FMXV9w\/IEpBdUGSJIG43LoDBLSrRqJi4BYkCIqIislw0yYsgaDAuIRoEiYiyCC8ioiwvKu4BNRgQEMQA+hnQgKBBBEF2QUDle06RuvZteqarqvvM9PT99\/PwoEzV6e7fqan6z6lTVeX279+\/n3CBAAiAAAiAAAiAQIESKAcxU6Cew2ODAAiAAAiAAAgoAhAzaAggAAIgAAIgAAIFTQBipqDdh4cHARAAARAAARCAmEEbAAEQAAEQAAEQKGgCEDMF7T48PAiAAAiAAAiAAMQM2kCZJzBr1izq2bMnVa9enSZOnEjNmzfPOZOdO3dSt27daNGiRdS2bVsaNmxYyTPw8w0aNIimTZtG9evXz+mzLVu2jLp06UI7duyg0aNHU5s2bdT99fO2bt2aiouLc\/pMYTfr06cPzZgxo9TzhtXRn8fxXiNHjqQ5c+bQuHHjqFq1aqa3jq2c12dstFGjRpHaTra2GdtD58CQ5lJUVJQ33+TgNcvsLSBmyqzr8eKaQJLFDA+MQ4cOjTwguXo7SMysX7+e2rdvT2vXrqW+ffsmSszo561Zs6b1AB7He2khdfrpp+dlwPQKD+3zqM+SFjHjfQ+vMHf9bqBesghAzCTLH3gaEChFAGLGrkFoXv7olomVtIkZFwZBnNIiZvjdorQPkzaEMvkjADGTP\/YFcWcdtdAPGzQV4\/012qFDB7rttttK3i3TL3ddRxf0l\/N3oL\/85S\/VVFCm8plgegeoTHWDIjPed2rZsiWNGTNGVfc+p+4YtV1\/OD+TEPEy1b8Q\/e973333lUw7ed8t06\/sTBEB7\/v7f42a+NYfmeFn8fpBP5vXtt+3XCbol7B\/OoTLrFy5MjASZTJ1YvOu\/Ezewd7Pwva9gtqZ\/x4m75CtUwjzl9++6XclqF7QlKKeAjX5Lvq\/G\/7vDv+\/yXfM25a47d9\/\/\/10ww03BEYFw\/oUvqd+V\/7vfE0pF0THX4APCTFTgE7L1SMHDUpBHWS2cv4OPSgMrm16B5ds5aJ00kH3yiZmvKy9Qi7TO3vL5FLMZJoq0\/\/uF1qmvrUVM9nsegfITOIhSBhmKusX1mEMgr43us2FiZmw9zrllFNKpt689wmzb5qnZeIvFzGTzQ9auJt8F72+DRIypv2G5tGsWbNAMe9la\/J8\/uhUHNG3XPW\/uI8dAYgZO15lprS3k\/NGI3Snmmlg93YeQWV1R+etHzQI+TtQ3Vl6O+xsuQDe+t6BPOiZwsSMP2oUxMb7XJpBFDGjE4BNp5mCOulM0wM2vrXJmQn61et9Ls0lk2+8z6V9xl84nZ8TVN\/b3jKxCopaBbXDTAOd6Xv5ow06ATiMQdh0kI2\/bKaEvM+lv0v8DjoRXfuAk5j1v\/Hn+rsY9F5B0THvM3m\/s16BZvId8\/cJuo5pn8LPbsOnzHT2KXlRiJmUODLu1zCZttCdiS7r\/\/XvHxx4VUzQih1vBxP0a8svWkySLDOtwglaGZRNzAStBAm6f9AqmFyKmaCBdNWqVYErkWx8ayNm\/G3Q\/wtdD9pem\/5BzN+W3n\/\/\/cCVZkERp0zv5R00swkH01\/tmd4rk5gJixiFrTay8ZfNYJ3pufyrsTKJkUzv620H\/shPkJjJ9h3zf+ZvOzZ9in4uk\/4j7v4U9uQJQMzIMy64O2TrEIM+y9Q5+MvefvvtgaH4oLB8tmcw6YwyiZkgZ4TlzPiX2Jrcn++TazHjjyDMmzfvoPwTW9\/aiplsUwxBYsafS+Nn9txzz6l3yHQFTUv4BUvQ9EvQ9E42MWPyXpkG92x1uU62qSZbf8UhZvysbb+L2aaugsRMUITVVMC1a9fOuE\/R72Ua7Sy4jruMPzDETBlvAEGvb9uBQswE7yWSazHj9VuPHj1oyZIlB+1bY+tbGzHjHcT0AH3EEUccNE2UTWhKiBndxoMGWe8v\/0xixvS9IGbGkTcaqKekfvazn5VEZCFmMOBIEYCYkSJb4HZNfxnxhmlRp5lsBZVJZCTTVIb+9yFDhpRsAOcamQlKql23bl3J\/iK5FjPeaFDDhg1p+\/btCq1\/1YaNb23EjH5f74AVlFcRxzSTTfQgqH0FPUMmMWP6XpnETKbpHNMuwsZfLpEZLTr0hoj8vP369StpNzaRmfnz56tpQe93IyxnJltkxnWaKRvbIH+a+gLlkksAYia5vsnrk9kkHWbKSTBNAA4aMG060KBdVjMlmXpD\/nqKw1bMBLEJSqbUHTs7UueG+JfwZlqabZsArBuLf0olaKCw8a2LmAla0cXPF1cCcCbRkC2XiZcVh4msMDET9l6ZnitI0GUqG\/Slt\/GXjZgJarP8XfJ\/b70ri\/xTeH7m3jbv\/37xu5lGZoLe2SYBOFv0z+THUF47X9zciQDEjBO2slEp23LQsH1FvIS8HWCm\/Tj8HV1UMcP2Mi1V9d\/LVsx4B6KglhC08ipTiwkTM9kSKINsZurw\/WVNfRsmNLVdfg+eUtJHHwQ9m8m+LieccAKtWLGi1C\/7bDknQUuC\/b\/ms+VweAWKn52OWNi8V6bkYNN3yNROTP1lI2b4XtnYhK0eCxJkfH+9+izoXUzFTJAv2J6OOPLxGpl+IHjv6xfztnzKRk+fjreEmEmHH8Xewt\/ZhW2a99vf\/pZ69eqlzvLhy3TTPP8vvjjEDN8\/SDwFnX3kP5vJ5Nebf4AKYhMUKfFuLBgmZvydetjKF+8AFbaHiYlvs60KC9rE0G\/TdCM8\/axBSctBwjQbay7vn1oLagdBLP3Pr9uv6Xv57+MdTP1tIcw\/\/i+1ib9cBusg0e\/93tp+F\/322NYxxxxz0Ko0k++YP+rrXUSQaSWc5ha0ci3bxopinSgM54QAxExOMKf7JiadUroJ4O3iIhBlpQkGqri8kFw7psvnM71B0J5ByX1bPJkNgYIXM9z5TZkyJfRQuUxTDlEPYbOBndayEDNp9azMe3kHpEyJomEbyWV6Mi2GXOvLvDGs2hLwRj29EZioydRoH7aeKJzyBS1mtEAxOSHXn6FfOC5K\/pNCzCTfR0l7wmw5IPysrqca2\/QJSWOC5\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\/OPfdci5oHis6dO5ceffRRmjp1qnVdiQqIzEhQ\/cEmOi45vmALtnIE5Cyj3cqxhZixYPvdd98R\/6lUqZJFreQWhZiR9Q06Ljm+YAu2cgTkLKPdyrGFmLFgy9NMnTp1osaNG1NxcTG1atWKDjnkEAsLySoKMSPrD3RccnzBFmzlCMhZRruVYwsxY8GWVzH96U9\/opdffpn27t1L9erVoxtuuIE6duxItWvXtrCUjKIQM7J+QMclxxdswVaOgJxltFs5thAzDmx37txJs2fPpvHjx9P777+vLJxyyinUq1cvOu+88wpmGgpixsH5FlXQcVnAsiwKtpbALIqDrQUsy6JgawnMsniax7Ry+\/fv32\/Jw6o4JwVPnz6dJkyYQPzf1apVo6uvvpq6du1KDRs2tLKV68JpdnyuWQbdDx2XnBfAFmzlCMhZRruVYzt45mqa\/O562j7+WrVimLdBSdMlLmY0LNZMH3\/8MY0dO5b+9re\/0a5du6hp06YqWnPFFVckMloDMSPb1NFxyfEFW7CVIyBnGe1Wjm3vycuVmKk14yaImbgwcz7NggUL6K677lImp02bRvXr14\/LfGx2IGZiQxloCB2XHF+wBVs5AnKW0W7l2ELMxMhWixiecmIxw8u4eXO9+++\/n\/gYg6RdEDOyHkHHJccXbMFWjoCcZbRbObaXP7aE5q3aisiMK2IWLEuXLqXJkyeXTC8VyioniBlXr5vVQ8dlxsmlFNi6UDOrA7ZmnFxKga0LNbM6EDNmnEqV4vwYPp+JIzAvvvgiffXVVyon5pJLLqEePXrQcccdR+XKlXOwnNsqEDOyvNFxyfEFW7CVIyBnGe1Wji3EjAXb3bt301NPPUUTJ06kdevWKcFSVFSk9pr55S9\/qVYyFdIFMSPrLXRccnzBFmzlCMhZRruVYwsxY8FWHzS5devWgll+ne31IGYsnO9QFB2XAzTDKmBrCMqhGNg6QDOsAraGoByKtRi0gD7b\/A1yZkzYcWRmzZo11KxZs0QutTZ5B28ZiBlbYnbl0XHZ8bIpDbY2tOzKgq0dL5vSYGtDy64sxIwFLx2ZGThwoFqlZHvNmjWLBg0apNbAJ+GCmJH1AjouOb5gC7ZyBOQso93KsYWYsWALMWMBC0UJHZdcIwBbsJUjIGcZ7VaObZ0+s5VxbJpnwFiLGV7J5HrxFsu5iMyMHDmSpkyZknXDPkRmXL1oVg8dlxknl1Jg60LNrA7YmnFyKQW2LtTM6kDMmHFSpfjU7FGjRqm\/XS\/ePG\/AgAGu1Y3qLVu2jLp06aI26su2+zDEjBFO50LouJzRhVYE21BEzgXA1hldaEWwDUXkVIATf3maCZEZJ3zJrMQnenfr1o0WLVqkDtqCmMmfn9BxybEHW7CVIyBnGe1Whu3bK7fSFSOXQMzI4M2PVZ5e4mmsli1b0iuvvAIxkx83qLui45KDD7ZgK0dAzjLarQxbiBkZrnmzyqul+vXrpzb1mzdvnnHOzDPPPFPquPQkHoqZN6gRboyOKwK8kKpgC7ZyBOQso93Gy5bzWPl6+sN9NHjmGkRm4sWbH2s6Obljx45UXFxMNgnA\/ifmfJvrr78+Py+Sorvu2bOHNm7cqE5Mr1ChQoreLP+vArZyPgBbsJUjEK\/lJ598Uv1433VqV9r747MhZuLFmx9rffr0UUcsjBs3Th2rYCNmhgwZQg0bNix5cB58EZ2J7kfeZHHDhg0q6gUxE52n1wLYxssTbOV4gq0cW\/4Rz3+KX9xIa745cJwQlmbL8Ra37J1eat68ubqfjZjhHBsecHHFSwAh5Xh5eq2BLdjKEZCzjHYbP1vvSiaImYh8+RTtLVu20HfffUd16tSh8uXL0759+3J25AFHZWbMmJHxLfr27aumnvwXlmZHdHxIdXRccnzBFmzlCMhZRruNny3ETAxMWcS8\/vrrdNddd6ncCL0UunLlynTTTTdRq1atiIVEpUqVYribnQlEZux4SZRGxyVB9YBNsAVbOQJyltFu42c7eObqkuTf8rs2UY1Z\/dWK3rTNNpTbz4pD6PrHP\/5BPXr0oFNPPZWOPfZYmj17tloKzTkrd9xxB82cOZP4HCdOqM31BTGTa+IH3w8dl5wPwBZs5QjIWUa7jZ+tPpOJLR\/9\/ae0+cX7IGZsMHPGf8+ePen777+n0aNH09y5c9UhknqTur179xJP\/2zatKkkKdfGftSyEDNRCUavj44rOsNMFsAWbOUIyFlGu42XrXd\/Gbb8h5\/uocfuKoaYscGsl0J3796dOnfuTPpEbO+Ou5MmTaLHH38868Z1NveMuyxyZuImWtoeOi45vmALtnIE5Cyj3cbL9vLHltC8VVuV0R\/X+RGNvrAcderUCWLGBrMWM9dddx3dfPPNgWKGoyPTp0+nqVOnUr169WzM56QsxIwsZnRccnzBFmzlCMhZRruNj+3kd9dT78nLSww+dk0RNf1uDcSMLWI9zbRr1y4aO3YssTDwTjPxydo33HADNW7cWE1DcVJw0i6IGVmPoOOS4wu2YCtHQM4y2m08bP0rmDgqs3TgWWocRmTGgfGSJUuIp5k4+Zf\/vPrqq\/T73\/+eVq5cSc8++yxx3syYMWPo5z\/\/uYN1+Sppdrw8vfA7oOMKZ+RaAmxdyYXXA9twRq4lwNaV3A\/1WMhwREZPL\/EnLxa3pHOOqQUxEwXv\/Pnz1dLs1atXlzLD00p\/\/vOf6cILL4xiXrQuxIwoXiwfFsSLQUEOLtiCrRyBaJaDhEyP1o3ogbbHKsNpHtNEl2Zrt\/Dqb1619K9\/\/Yt4+unEE09UxwEccsgh0TwnXDvNjhdGZ2Qeg4IRJqdCYOuEzagS2BphcioEtk7YVKUgIaOnl7TVNI9pOREz7u7Jb800Oz6\/ZA\/cHR2XnBfAFmzlCMhZRrt1Y8tC5oqRS5Sg0ZdfyCAy48aWtm3bRqNGjVJ\/h11169ZV000nnXRSoqI1EDNhnov2OTquaPyy1QZbsJUjIGcZ7daerX8vGbbAQobzZPhv75XmMU0sMsPHF9xyyy304YcfEq9o4otFC1885RR0nXvuuTRixAiqUaOGvUcFaqTZ8QK4rE2i47JGZlwBbI1RWRcEW2tkxhXA1hhV4LSSFjK8cinoSvOYJiZmGOSiRYuod+\/e6riCG2+8UR1jwBevYvq\/\/\/s\/dXL1o48+Sj\/5yU\/oxRdfVEu3uextt91m7lHBkml2vCA2Y9PouIxRWRcEW2tkxhXA1hiVdUGwDUfGU0kPzlpDzyz6olRhjsI80uEEan1s7YxG0jymiYmZnTt3Urdu3ei4446je++9l8qVK1cKMCcF33333fTpp5+qfWaqVKlCgwcPVtnWzz\/\/fLhHc1AizY7PAb7QW6DjCkXkXABsndGFVgTbUETOBcA2MzoWMUNfX0OTFpYWMToaEzSt5LeW5jFNTMzoHYA5MtOhQ4dAD\/HOv4899ljJcQbPPfccPfzww2qr5SRcaXZ8Evii45LzAtiCrRwBOctotwezDVqlpEtxNObRjkVqDxmTK81jmpiY2bJlC3Xt2pWaNWtG999\/P1WqVKkUa55quvPOO2nVqlU0fvx4ql27tsqX4Y31XnvtNRO\/iJdJs+PF4RncAB2XASTHImDrCM6gGtgaQHIsArY\/gBv+90\/p3pc\/CSTJIob3jjmxQbWDknyzoU\/zmCYmZhgoHyI5dOhQuvrqq1UyMO8twxdHbThXZsqUKSo\/plevXiq\/5o477lB70LCoScKVZscngS86LjkvgC3YyhGQs1zW2+1L72+ku15cWWqJtZf2gUjMCfTjOlWsRIy2keYxTVTMcPSFxcwTTzxB3333XalvAG+Yx0nBffv2VfuN8GGU69atUwKoqKhI7ttiYTnNjrfAIFa0rHdcYmCxh48kWuyPJEi3rPUJvKx6wSdb6YG\/ld4h34\/Ydjopk4vSPKaJihkNlJdpv\/nmm7Rs2TL1Txx9Of\/886lhw4bq\/3fv3q2iNQ0aNEjUgZNpdrxgf2Rsuqx1XMZgYigItjFAzGACbMHWhQDnvrAoyZYD44\/C3HfFMdSiUXWnKEzQM6Z5TMuJmHFxfBLqpNnxSeCLQUHOC2ALtnIE5Cynrd3qHXkHz1xNk99dHwqOxc6NP2tIV7Y4PDYB471pmsc0UTHDy68\/+eQTmjFjRuBGeXxO0+eff07Dhw8vyacJ9XYOC6TZ8TnEmPFWaeu4ksBUPwPYynkDbME2EwEWL3xaNe8B4z21OhsxFjA9zm1El55cT0TAQMzE0F55ZVKfPn3UJnlBF69watWqlUr45dVMSbsgZmQ9gkFBji\/Ygq0cATnLhdJu9ZQR57y8vXKLEi424uXsZrXomtOOVOLFf+SAHF2cmu3E9uuvv6YePXqopN5HHnmEjjrqKJXk27JlS7Ur8OTJk2nChAlqw7yTTz7Z6R7SlSBmZAkXSsclS0HGOtjKcGWrYFs22bJw+WLbHpr4zjpj4cKkWKw0rv0j6n9x05yLF7+n0jymiU0z6U3zeFn2rbfeqpjec8899OWXX6ppJb44alO5cmUaMmTIQTsEy31dzC2n2fHmFORKYlAAWzkCcpbRbtPL1htxeeWDjfT\/1u20Ei5avJx7TG3q0Kp+3sULxEwMbVWLmd\/\/\/vfUrl07ZXHSpEn00ksv0dixY9Vhkvz\/Tz\/9ND311FMlh1DGcOvYTEDMxIYy0BAGBTm+YAu2cgTkLOe63XK0ZdGabTT7o83WokULF\/574CVHU\/0alRMnXiBmYmir27dvV9NKp556KvXv319ZfOONN1R0hkVMkyZNlKhhITNt2jQkAMfAvNBM5LrjKjQ+UZ4XbKPQy14XbAuLrV5RtHD1NjVF9J8t32TclC7bm+nclnYtDqdfnHBY4oVL0Luk+Qe62DQTg3zooYfUhnm33347dezYUeXPdO7cWe0I\/Ktf\/YoGDBhAvOJJH2cg9xVxs5xmx7sRibcWBoV4eXqtgS3YyhGQsxyl3bJo4T\/vfrqN\/r7CLdKi30znufAqo1pVKxakcIGYibGdcnSG82X4bxYstWrVokGDBqnEXxYxfJI2n5zdpUuXGO8anymImfhYBlmK0nHJPlnhWwdbOR+Cbf7YasHy\/f79NGTWGucoi1e08H\/\/4vg61K7lEcYHNsoRkLWc5jFNNDLDbmHRsmPHDqpevboSL3yswfz582nu3Ll08cUXq2ko\/vckXml2fBJ4Y1CQ8wLYgq0cATnL3G7f++g\/tK9ybSpX\/hAa+voadTPTZc+ZnkxPEf3qxLpqPxe+cr0sWo6aueU0j2niYiYbZhY227Zto5o1axKf1ZS0K82OTwJrDLhyXgBbsJUjEN0yJ96ymHh60Rc0f9XWyBEWb6SFl0Ff0bweFdX\/4UTpXO7lEp2OnIU0j2liYkavZho4cCC1adMm0DuvvPIKDR48GAnAcm030ZYx4Mq5B2zBVo5Adss64Zb\/\/nzrNzRp4RexRFe8goX\/+6Kiw+jXzQ8vs1EWF\/9CzBhS4+MJeApp165dtHXrVnr44YfpyiuvpObNmx9kgctyHg2XnTp1KtWrdyD0l6QrzY5PAmcMuHJeAFuwlSNAJauB1ny1W00F7d8ffSrI+7ylp4XqElG5MjktFLcP0zymxR6ZmThxIt17770qVybs4qmlfv36qSXcScybSbPjw3yTi88x4MpRBluwdSHg3TSO6y9bu4P+9uGmWCMr3ggLTwld2fJwOu6xjnJ8AAAgAElEQVTwQ5VY4XZbcc8Waty4MVWoUMHlFVAnC4E0j2mxixk+h4nzYDZs2EA9e\/ak3\/3ud3TuuecehJfPZeLzmJIoYvTDptnxSfjGY8CV8wLYgm0mAnoa6KMNX9MrH2yiVRt3xZaz4o+usFhpclgVuu2io2jtlj0l5xBlymFBu5Vrt2w5zWNa7GJGu4KTezdv3qx2+uUjCwrxSrPjk+APdFxyXgDbssfWG1X59vv99OaKr2jpf3aICBWmq\/diOf6IQ+nEBtXo2MOrhoqVMK+g3YYRivZ5mse0WMWMFjD8t+nFU0116tTBaiZTYCkqh45Lzplgmz62egXQyo276M0Vm+n9tXJCxStWzmhak35+XG0q99+8Ff2ZBGG0WwmqP9iEmDHkq1cwrV271rAGUaNGjbCayZhWugqi45LzJ9gWDlu9ERxHOnj32jkfb6FPNu0Wi6h4xQiLFJ4G+k3LI1RSr57+yddSZrRbuXaLaSYLtrt376Z58+YRr1QyvXgK6uyzz6YqVaqYVslZuTSr2JxBzHIjdFxyXgDb\/LPVuSn8JJ9t3k0LPtlGc\/69RT1Y1E3gsr2dFiKcr3LxiXWpRaPqqnghbBKHdivXbiFmZNkm2jrEjKx70HHJ8QVbebb7D61H67Z\/S7w8eeri9eqGrocYmj6tV6hwVOXMprVKoineiIupvaSVQ7uV9Uiax7RYc2YyuWHLli309ttv03vvvUe8iqlly5Z05plnqtVMSb7S7PgkcEfHJecFsHVny7kpfP39o8307pptOREpWohwNIUFC58VdESNAwsn8j31407SvibarT0zmxppHtNExQwnAj\/++OM0bNgwdSaT9+LE3969e1NxcbESOEm80uz4JPBGxyXnBbD9ga13R1qi\/bRozXaa\/dHmnIoUvhkLlQtOOIx+elR1OqpOFZWjcs4xteQaQQFaRruVdVqaxzRRMTN9+nTq378\/tW7dmm677TZq0qSJ8tSaNWvooYceUrsFDxkyhK644gpZDzpaT7PjHZHEWg0dV6w4SxkrK2x18mzVSuVp6uIN9K8vdioOkjkpGrR3yuekBtXo\/OPrUNVKB86YK4T8FLnW5265rLRbd0LRaqZ5TBMTMzt37qRu3bpRtWrVaMSIEQcl+HKy8K233qoiNqNHj07kXjRpdny0r0Q8tdFxxcMxyEqhsi2dNPsNrdq0i559bwPxfuLS+ShBIuW4I6qqaEqNHx3YjRa71Mq1WbZcqO1Wlkp81tM8pomJGb1Mu3v37tS5c+dAb0yaNElNQ02bNo3q168fn8dispRmx8eEKJIZdFyR8GWtnCS2foGydus3NPffW9Q0S64EihYi\/DdP95zSqLqa4qleuYJ1Am2S2Mq1oPxYBltZ7mke08TEzMaNG6lDhw7qoEmOwARdHLF5\/vnncdCkbPtNrHV0XHKuyQVbvS8J\/73kP9tp7r+3Em+Rz1cupnmCIikX\/eQwOrVxjZIoir9MHMRzwTaO5yxEG2Ar6zWIGQe++\/btoz59+tDy5ctp7Nix1LRp01JWVq9erQ6YLCoqUgnCFStWdLiLbJU0O16WnJl1dFxmnFxKubLVOSh8zwrly6klx7zjbD4FCm+Vf+4xtahmlQN9hHdDt3xs7ubK1sWPZa0O2Mp6PM1jmlhkhl3ywQcfUNeuXdUmehdddJHaHE91ivPm0euvv67yZHiaiZdqJ\/FKs+OTwBsdl5wXNFveC4VPH9ZTPbwnyqI122i18A6zQW\/mTZg9qs6PqFWTmnRMvaqJECg2nkC7taFlVxZs7XjZlk7zmCYqZhj0Rx99RHfeeSctXbqU9u\/nND5SJ2W3aNGC7r\/\/fjr++ONt\/ZGz8ml2fM4gZrkROi43L3i3v+edZRd\/ul3ticJfr1zmoHijJHp\/FI6inNKwuoqepHXpMdqtW7s1qQW2JpTcy6R5TBMXMxo7R2d48zy+eLO8QjhJO82Od\/86xFcTHdcPLL0C5csde9X0zor1B\/JP8ilQmtWrSocdWpGuP6tBIs7uia\/1uVtCu3VnF1YTbMMIRfs8zWOamJjhBODx48erPWSOO+64RJ6KHdYs0uz4sHfPxedp7ri8G7VxRHLxZ9vp7ytyt1Gb33\/eKZ6mdavQCfUPLRVBKUu7zEZt22lut1HZRK0PtlEJZq+f5jFNVMzwaibeIO+www5TouaGG25Qp2TzNFMhXGl2fBL4F1LH5U2M5YH\/H\/\/eQks+207\/\/nJXziMn2nelNm1rWI1+dnQtql31QJLs4VWJ+AfFkUceSUcfXi0J7k7NMxRSuy006GAr67E0j2liYoZdsnfvXlqwYAFNmDBB\/c3\/z6ua9JLtevXqyXouovU0Oz4imliq57Pj0suK9Tk8m7\/eR++s3koffH5gB9lcT+3wPb3ihP\/\/vOPr0FUtj1DP4v3cZAVPPtnG0jgSbARs5ZwDtnJs2XKaxzRRMeN1i1\/Y8NJtXpbNq50uu+yyRJ7PlGbHy35lzKzH3XF58044YrL4022kxUo+xIlfoPzkyEOpcZ0q1KJRdWtxYkb0h1Jxs7W9f5rLg62cd8FWji3EjADb7du30xNPPKEiNtWrV8cOwAKMC8Fkto7LHzlpXOdHNH\/VVpq3cgt9muOdY70sSy0vPqyKOt348Oo\/HJSa7z1Q9LNiUJD7BoAt2MoRkLWc5h\/oOYvM8FlNs2fPVsKFgfJ1yimnqMgM70GTxJOz0+x42a9Mdus6glK78vf05Nuf0vIDebF5n9rhVK6f\/riGOjCw\/H\/zugo1MRYDrlwLB1uwlSMgaznNY5qomPELGD5UknNmbrzxRrr00kvVEm2Xi3cWnjFjRklVPqiyTZs2WU0tW7aMunTpQjt27ChV7vTTT6dx48apAzH9V5od78I9Ux1\/FOWDz3fQqk276aP1X+dcoHgjI7z3yZlH16RWR9WgzV9\/q87kSUrkJE7+QbYw4MoRBluwlSMgaznNY5qYmNEHTa5du5Y40fe6665T5zQ1bNgwkrdYyCxevLhkamrWrFnUs2dP6tu3LxUXF2e0zeX69etHEydOpObNmxs9Q5odbwTAU0gvNeaN2Ua89VlOhYp\/aqf1MbWVSNHPxJ+bJMXavnMhl8eAK+c9sAVbOQKyltM8pomJGd4gb\/LkyXTxxRfT0UcfHctybB1dGTJkSKlIDAucdevWZYywcPMYOXIkzZkzJ2uZsh6Z0eLg069207T3NhBvfS91YCCLj2+\/\/ZaOrF6BzjqmLjWpW5WOO+JQ5YJCndqR7YbsrGPAteNlUxpsbWjZlQVbO162pSFmbInluLyJmOEyfPGhlqZXmh3PDHilz4vvf0nLv\/g6NtGihcjZzWrRaUfVpMoVy2ec3kHHZdoS7cuBrT0z0xpga0rKvhzY2jOzqZHmMU0sMmMDOEpZjrgMHTqUsuXN6CkvzotZsWJFye3atm2bVdykzfGT311Pzyz6wlm4eCMm5zSrTT9rVitSJAUdV5SWn70u2IKtHAE5y2i3cmzZctrGNC+tghUzOleGXyZMlOjpqQsuuKBEvGiB06BBg9AE4GeeeUbtXKyv+vXry7a4GKyv2\/6tyimZ\/s8NNPGddVYWG9SoQOc0q0XtWh5BlSqUp3OOOSBa4r7QccVN9Ad7YAu2cgTkLKPdxsuWxznvxTmsnTp1UikX3jEt3rvmx1rBihmNi1dMdevWTeXM8LJvG6GhBVGmqI5WsX7X8Kqo66+\/Pj8eM7jrowu20BOLt4WWZNHCV7uTqtMp9Sur\/BX9b6GVYyjAh4\/ylvvsswoVDjwLrngIgG08HIOsgC3YyhGI1\/KTTz6pFr34L4iZeDnHZk1HXnr06JF1RZP\/hmH1tJjhhGPvKiwefG1EU2wvmsGQXhrd6i\/vEkdkMl08TcS5LFe3rKeSbHMpXIKeaffu3bRhwwb1CwFiJt5WArbx8vRaA1uwlSMQr2WOzHijMwsXLqThw4cjMmODedu2bTRq1Ch1VMFJJ50UWPW9996jhx9+WE39RDmnKUyUZHrusHqFML\/ISbxXjFySVcA837MFHVK+XOKWLyOkbPONsisLtna8bEqDrQ0tu7Jga8fLtnQhjGm276TLi00z6ZyUgQMHBm5oxxvoceLuK6+8Yjw9lGmvmLDpokyfh+09k1THcySGl0z3nrw80O8cdXm2RwtqVq+Ka7vIST10XHKYwRZs5QjIWUa7lWPLlpM6psXx1rGKGT5M8s4776TnnnvO+Nl+8Ytf0IgRI6hKlfCBV+fHsHG9a6+OrvChlZl28g3KqwlKCvY\/dBIdz0KGIzF6TxjvM7OIebRjkVjCrrFTDQui4zIE5VAMbB2gGVYBW0NQDsXA1gGaRZUkjmkWj5+1aKxihu+0evVq4qQj3jSPz2Jq0aJF4K6\/fMBky5Yt6ZxzzlGHTdpc\/uMM\/Lv\/8nLtMWPGHLTbb1i9JIsZFi+fbNpN7UYvPQgVi5gXi1smbhopzKfouMIIuX8Otu7swmqCbRgh98\/B1p2dSU2IGRNKvjImOTMOZnNaJSmOzxSNKVQRo52IjkuuOYMt2MoRkLOMdivHli0nZUyTeMvYIzMSD5kvm0lwPAuZFoMWlEJQaNNJmfyHjkuuZYMt2MoRkLOMdivHFmLGgq2OxnCVa6+9lp5++mnif8t21axZk3r16kX8d9KufIsZ3rHXn+Rb6NEYr4\/Rccm1eLAFWzkCcpbRbuXYQsxYsNUrmLgKJ\/XeeuutxDsOZrt4jxHbze4sHilS0XyKmSAh85crj6Xu5\/6wE3Gkl0tAZXRcck4AW7CVIyBnGe1Wji3EjCzbRFvPl5jxC5m0TCv5nY2OS675gy3YyhGQs4x2K8cWYkaWbaKt50PMBAmZa06rT\/0vbppoVi4Ph47LhZpZHbA14+RSCmxdqJnVAVszTq6l8jGmuT6rbT2xBGCdP4OcGXOX+JN90xqR0UTQcZm3DduSYGtLzLw82Jqzsi0JtrbE7MpDzNjxUqV1\/gxyZszgBS2\/fuyaIuKoTFovdFxyngVbsJUjIGcZ7VaOLVuGmImR7\/79+2nTpk00ZcoUeuGFF9TZTJnOborxtk6mcuV4FjK3TFlOfM6Svvpf3CSVU0teR6DjcmqWRpXA1giTUyGwdcJmVAlsjTA5F8rVmOb8gBEqik0zhT0Ti5q7775b7RTMB01WrFgxrErOP8+V4wfPXE2DZ64peb+bz2lEg9sdm\/P3zfUN0XHJEQdbsJUjIGcZ7VaOLSIzgmynTp1Kjz32WJlemh2UJ7N04FmC1JNjGh2XnC\/AFmzlCMhZRruVYwsxI8RWH0r54Ycf0lNPPUV169YVupO72VxEZi5\/bIk6AZuvtCf8+j2Bjsu9bYbVBNswQu6fg607u7CaYBtGKNrnuRjToj2he22xaaaw1UyffPIJLV68mLp06UJ\/\/OMfqVy5cu5vIVRT2vH+qEzaE34hZoQaaoBZDApyrMEWbOUIyFqWHtNknz67dTExE7aaqVKlStSuXTsaMGAA1ahRI58MMt5b0vFleXpJA8egINfswRZs5QjIWUa7lWPLliXHNNknD7cuJmbCb538EpKO7zv9Yxo\/\/3MFIU3nLdl4FR2XDS27smBrx8umNNja0LIrC7Z2vGxLS45pts8Sd\/mcixlexbRjxw6qXr16IqeWvIAlHV+nz+ySW\/32\/B\/TPZc3i9u3ibeHjkvORWALtnIE5Cyj3cqxRWQmAltO8v3rX\/9KnOQ7cuRIqlatmlqK3bVrV1q3bh394Q9\/oMsvvzyxokZKzPiTfl8sbqmiM2XtQscl53GwBVs5AnKW0W7l2ELMRGD7+OOP09ChQ1VuzMCBA5WY2b17N82cOZPGjRtHK1euVHvMXHLJJRHuIldVQsz4c2XKwuZ4mTyEjkuu7YIt2MoRkLOMdivHFmLGka2OwBx77LH05z\/\/+aBN8Thq06dPH7UbMAsbFjpJuyTEjD8qU1b2lAnyLTouuRYPtmArR0DOMtqtHFuIGUe2ejVT7969qUOHDoFWnnvuOXWcwbRp06h+\/eSdQRS3mOHjCq4YuaSEBZ+7xMuxy+qFjkvO82ALtnIE5Cyj3cqxhZhxZMsRl+uuu47OO+886t+\/f6CVwYMH01tvvVVmNs379cglNPe\/5y9xjkxZjspwg0DH5fjlMqgGtgaQHIuArSM4g2pgawApQpG4f6BHeJTYq4qtZuJVS\/fddx+9\/PLL9OCDDypRozfG489YxNx+++102WWXlYlN8\/y5MiM6nkDXnn5k7A4tJIPouOS8BbZgK0dAzjLarRxbRGYisF27di11796dVqxYoTbG4+XYfO3Zs0flypxwwgnEScKNGjWKcBe5qnGq2NkfbabfjFmmHras7ivj9xQ6Lrm2C7ZgK0dAzjLarRxbiJmIbHn1EufE8J\/t27crayxs2rdvr\/5UqVIl4h3kqsclZvxRmQtOqEP\/17253IMXiGV0XHKOAluwlSMgZxntVo4txIwjW55KeuGFF+jEE08kXtFUiJeUmOF9Zc45plYhIon1mdFxxYqzlDGwBVs5AnKW0W7l2ELMOLLduHGjWsV01VVXUXFxsaOV\/FaLS8xgOXawH9FxybVvsAVbOQJyltFu5dhCzDiy1auZeIffsi5mvEcXlOVN8vxNCR2X45fLoBrYGkByLAK2juAMqoGtAaQIReL6gR7hEcSqiq1m4id+44036J577lERmksvvZSqVq160IsccsghVKdOHeK\/k3bF4fjBM1fT4Jlr1KthOXZpD6PjkmvxYAu2cgTkLKPdyrFFZMaRrd40j1c0Zbt4JVOaN83zTjGd3awWvdS7pSPR9FVDxyXnU7AFWzkCcpbRbuXYQsw4suVVTPPmzVPLsLNdlStXprPPPjuRq5qiRmb8q5iQ+IvIjOPXyboaBgVrZMYVwNYYlXVBsLVGZlUh6phmdbMcFxadZsrxu8R+u6iOf+vjzdRu9A97y5T1HX\/9DkLHFXuTLTEItmArR0DOMtqtHFtEZmTZJtp6VDHjnWK6qOgwmnrzKYl+31w\/HDouOeJgC7ZyBOQso93KsYWYsWCr82S4yogRI+jWW2+lspozgymm8IaDjiuckWsJsHUlF14PbMMZuZYAW1dyZvWi\/kA3u0t+SsU6zbRt2zYaNWqUepNrr72Wnn76aeJ\/y3bVrFmTevXqRfx30q4ojp\/87nrqPXm5eiWsYgr2LDouuRYPtmArR0DOMtqtHFtEZmTZJtp6FDHjnWLi3X45+RdXaQLouORaBNiCrRwBOctot3JsIWYisOUjDV577TWaM2cO3XXXXXTooYeq85l4E72KFSvSgAED6Pjjj49wB9mqrmIGU0xmfkHHZcbJpRTYulAzqwO2ZpxcSoGtCzXzOq5jmvkd8lcy1mkm\/2u8+uqr1KdPH2revDmNHj2aateuraadHnzwQXrppZeIl2XzqdktWyYzauHqeO8UEzPZPOz8\/Hk4wXdGxyXnHLAFWzkCcpbRbuXYIjLjyHbnzp3UrVs3tX8MJwNXq1atlKUtW7ZQz5491a7ALHRY2CTtchUz2CjPzJPouMw4uZQCWxdqZnXA1oyTSymwdaFmXsd1TDO\/Q\/5KikVm9Mqm7t27U+fOnQPfcNKkSSoyk6YdgP1TTI9dU0TXnFY\/fx5O8J3Rcck5B2zBVo6AnGW0Wzm2iMw4stWnZl955ZVqiXbQNXLkSJo+fTpNnTqV6tWr53gnuWouKvbtlVvpipFLSh4Ku\/5m9g86Lrm2C7ZgK0dAzjLarRxbiBlHtvv27VP5MsuXL6exY8dS06ZNS1lavXo13XzzzVRUVETDhg1TCcFJu1zEDA6WNPciOi5zVrYlwdaWmHl5sDVnZVsSbG2J2ZV3GdPs7pC\/0mLTTPxKH3zwAXXt2pV27NhBp556Kh111FHqTTdt2kRz586l6tWr0\/jx4+nkk0\/OH4Esd3ZxPPJlzF2JjsuclW1JsLUlZl4ebM1Z2ZYEW1tiduVdxjS7O+SvtKiY4df6\/PPP6YEHHqA33niD9u7dq960UqVKdOGFF9Idd9xBDRs2zN\/bh9zZxfF1+swusdr\/4ibU\/+LSEanEvmweHgwdlxx0sAVbOQJyltFu5diyZZcxTfaJ4rMuLmbie9TcW7J1PPJl7HyEjsuOl01psLWhZVcWbO142ZQGWxta9mVtxzT7O+SvBsRMFva2jv\/Hx1voytFLlUUcYRDeqNFxhTNyLQG2ruTC64FtOCPXEmDrSs6snu2YZmY1GaUgZmIUM958mZ8fW5ue79UiGV5O6FOg45JzDNiCrRwBOctot3Js2TLEjCzfxFq3cbx\/fxnky4S7FR1XOCPXEmDrSi68HtiGM3ItAbau5Mzq2YxpZhaTUwqRmZgiM8iXsW\/U6LjsmZnWAFtTUvblwNaemWkNsDUl5VYOYsaNW8HXsnG89zwm5MuYuR4dlxknl1Jg60LNrA7YmnFyKQW2LtTM69iMaeZWk1ESkZmYIjPYX8a+QaPjsmdmWgNsTUnZlwNbe2amNcDWlJRbOYgZN26qFu\/0+7e\/\/Y0+++yzQCs1a9akXr16Ef+dtMvG8S0GLSDOm+EL+TJmnkTHZcbJpRTYulAzqwO2ZpxcSoGtCzXzOjZjmrnVZJQUjcy8+uqr6kgDvVle0Cs3atSo4A+a9Cf\/4jwms8aNjsuMk0spsHWhZlYHbM04uZQCWxdq5nUgZsxZlZTcuXMndevWjb788kt65JFH6MQTT6Ry5co5WMpfFVPH+5N\/Nw87P38PXUB3Rscl5yywBVs5AnKW0W7l2LJl0zFN9ilkrItFZtavX0\/t27en6667Th0omYSLo0QzZswoeZTRo0dTmzZtMj6aqeNxuKSbd9FxuXEzqQW2JpTcyoCtGzeTWmBrQsm9jOmY5n6H\/NUUEzNbtmxRh0xecskliRAzLGQWL15cMqU1a9Ys6tmzJ\/Xt25eKi4sDPWDqeCT\/ujVgdFxu3Exqga0JJbcyYOvGzaQW2JpQci9jOqa53yF\/NcXEDL\/S448\/Tq+99pr6u169enl7y2XLllGXLl1oyJAhpSIxLHDWrVtH48aNo2rVqh30fKaOx+GSbq5Fx+XGzaQW2JpQcisDtm7cTGqBrQkl9zKmY5r7HfJXU0zM7N69m9566y164oknaMWKFdS6dWuqXr36QW+az9VMcYgZJP+6N150XO7swmqCbRgh98\/B1p1dWE2wDSMU7XOIGQd+Omdm7dq1WWvnazXTyJEjaejQoZQtb0Y7\/plnniF+Tn3Vr1+\/5L\/fWbOTrhi5pOT\/v3zwXAdaZbMKOi45v4Mt2MoRkLOMdhsvWx6HvRePx506daI5c+aUGtPivWt+rIlFZvLzOuF31bkyXLJt27Y0bNiwjJW0mPEX4Cmr66+\/Xv3z8x\/uoEF\/\/0r9d4MaFeil638QPeFPU7ZL7NmzhzZu3EgsDitUqFC2YcT89mAbM1CPObAFWzkC8Vp+8sknaeLEiQcZhZiJl3Nereml45wzM23aNDWg+i8tZjjXpmHDhiUfc1ld\/soxH9DCT3eqzy44vjY9df0JeX2vQro5T0Vu2LBB\/UKAmInXc2AbL0+vNbAFWzkC8VrmyIw3OrNw4UIaPnw4IjMumFetWkWDBg2iBQsW0OGHH66EQ9WqVWnAgAH0y1\/+ki6\/\/PK87T+jE4N79OgRuKLJZH4Ryb8ureJAHYSU3dmF1QTbMELun4OtO7uwmmAbRija5yZjWrQ75K+26DTTBx98oJZnly9fnpo0aUJffPGFEjOVK1dW\/758+XK1oV62vV4k0UQVM0j+jeYddFzR+GWrDbZgK0dAzjLarRxbtgwx48B337596igDTjjipdlLlixRERo9pbN9+3a1\/wxHaTgJlwWO1MV5Mv369VNzh82bNy+5jc6fyZQEHOZ4\/86\/SweeRXxiNi4zAui4zDi5lAJbF2pmdcDWjJNLKbB1oWZeJ2xMM7eUvJJikRlO7OzQoQNdddVVagqHhYNXzDCKsWPH0lNPPSV+NpPOj+F76j1ldFSmqKjIeZ8Zr5hhEcNiBpc5AXRc5qxsS4KtLTHz8mBrzsq2JNjaErMrDzFjx0uV1kuzu3fvTp07dw4UM5MmTVJRm0wJuA63zVrFf5xBtt1\/2VCY47HzbzQPoeOKxi9bbbAFWzkCcpbRbuXYmoxpsneXtS4WmdHRkLp166rlz7yBXtA0U8WKFWnMmDF06KGHyr6pg3UbMdP\/4ibU\/+KmDncpu1XQccn5HmzBVo6AnGW0Wzm2EDMR2L766qvq7COOzBx11FE0atQolR\/DeTSPPvqoSgAePHiwmopK4pVNzPiTf5EvY+9BdFz2zExrgK0pKftyYGvPzLQG2JqScisX9gPdzWoyaolFZvj19u\/fr5JueZ+WXbt2lXrjQw45RCUI8zQU\/3cSLxsx82JxSzrnmFpJfI3EPhM6LjnXgC3YyhGQs4x2K8cWkZkY2PLKpUWLFqk\/e\/fupZ\/+9Kd0zjnnUO3atWOwLmfCRsxsHna+3IOk1DI6LjnHgi3YyhGQs4x2K8cWYkaWbaKtZxMzg2eupsEz16jnx0omNzei43LjZlILbE0ouZUBWzduJrXA1oSSexlMM7mzU1NNH3\/8MT377LO0Y8cOZYmTgtu1a0dNmyY7YTab47GSKUKj+G9VdFzRGWayALZgK0dAzjLarRxbRGYisOXppT\/84Q\/EicAsarxXuXLlqGPHjnT33XdTpUqVItxFrqqpmHm04wnU6fQj5R4kpZbRcck5FmzBVo6AnGW0Wzm2EDOObFm8cOLv+PHjqXfv3uqU6Ro1aihrLHJ4wzzeY+a2225TScBJvDKJGf9KpseuKaJrTjv4oMokvlOSngkdl5w3wBZs5QjIWUa7lWMLMePIdtOmTXTdddfRaaedRvfee+9Bh0my2OGoDJ\/fxIInicnA2SIz3gMmsZLJrZGg43LjZlILbE0ouZUBWzduJrXA1oSSexnkzDiw0zsAc1SGjzUIul566SUVvcnVDiz6C8MAABtJSURBVMC2r5HJ8f4zmbCSyZbsgfLouNy4mdQCWxNKbmXA1o2bSS2wNaHkXgZixoHdnj176NZbb1X5MLwDsD8vhpdo33nnnWrKacSIEaIHTTo8vqqSyfFYyeRKtHQ9dFzxcAyyArZgK0dAzjLarRzbbGOa7F1zY1100zze6ZfzYRo3bky33347NWnShMqXL6\/ObeIdgPnwyQceeIBOOumkkrflDfTq1auXm7cPuUsmMXPb9I\/oifnrVO2zm9Wil3q3TMTzFtpDoOOS8xjYgq0cATnLaLdybCFmHNnqaSYWNDZXo0aNaM6cOTZVxMpmEjPeZdk4k8kdPzoud3ZhNcE2jJD752Drzi6sJtiGEYr2OaaZHPjt3r2b5s2bRzzdZHNVrlyZLrzwQpsqYmUzOd6b\/Asx444fHZc7u7CaYBtGyP1zsHVnF1YTbMMIRfscYiYav4KtbSJmsJLJ3b3ouNzZhdUE2zBC7p+DrTu7sJpgG0Yo2ucQM9H40ZYtW+jtt9+m9957TyUCt2zZks4888xELsf2vmqQ4\/0rmSBm3BsHOi53dmE1wTaMkPvnYOvOLqwm2IYRivY5xIwjv++++05tjMermfi\/vRcn+vKy7eLi4oLaAdgrZnAmk2PD+G81dFzR+GWrDbZgK0dAzjLarRxbtgwx48h3+vTp1L9\/f2rdurXa6ZdXM\/G1Zs0aeuihh2j+\/Plqn5krrrjC8Q6y1YIcP\/nd9dR78nJ1Y4iZaPzRcUXjBzEjxw9swTY\/BGTvCjHjwHfnzp3UrVs3qlatmtpHpkqVKqWscIIw70PDEZvRo0cXzD4zOGDSoTFkqAIxEx9LvyWwBVs5AnKW0W7l2CIy48hWL83mfWY6d+4caGXSpElqGqqQdgDGsmzHBhFQDR1XfCwhZuRYgi3Y5o6A7J0QmXHgu3HjRnWMwZVXXqkiMEEXR2yef\/55mjp1amI2yvM+Z5DjWwxaQHzQJF9Ylu3QMDxVIGai8ctWG2zBVo6AnGW0Wzm2iMw4st23bx\/16dOHli9frk7Ibtq0aSlLq1evpptvvpmKiopUgnDFihUd7yRXzS9m\/KdlLx14lsqbweVGAB2XGzeTWmBrQsmtDNi6cTOpBbYmlNzLIDLjyI5PxO7atavaOO+iiy6is88+W1nizfRef\/11lSfD00y8VDuJl9\/xWJYdr5fQccXL02sNbMFWjoCcZbRbObaIzERk+9FHH6kDJZcuXUr79+9X1sqVK0ctWrSg+++\/n44\/\/viId5CrHiZmcFp2NPbouKLxy1YbbMFWjoCcZbRbObYQMzGx5egMb57HV+3atRO5esn\/qn4xg2XZMTWG\/5pBxxUvT0Rm5HiCLdjmhoDsXTDNJMs3sdb9juf9ZVjQ8IXTsqO7DWImOsNMFsAWbOUIyFlGu5Vji8iMLNtEW\/eLGewxE6+70HHFyxPRAzmeYAu2uSEgexdEZmT5JtZ6NjGDZdnR3QYxE50hIjNyDMEWbHNPQPaOEDOyfBNr3e\/4On1mlzzrY9cU0TWn1U\/ssxfCg0HMyHkJbMFWjoCcZbRbObaYZpJlm2jrXjHzfdW6xBvm6QunZUd3HTqu6AwRPZBjCLZgm3sCsndEZEaWb2Ktex2\/5ptqdMXIJSXPig3zorsNYiY6Qwy4cgzBFmxzT0D2jhAzsnwTaz2bmMEeM9HdBjETnSEGXDmGYAu2uScge0eIGVm+ibXudfzTH+6jwTPXqGflIww4MoMrGgGImWj8stUGW7CVIyBnGe1Wji1bhpiR5ZtY617HPzB3B\/aYidlT6LhiBuoxB7ZgK0dAzjLarRxbiBlZtom27hUzvV7YSPNWbVXPiw3z4nEbOq54OAZZAVuwlSMgZxntVo4txIws20RbzyRmsMdMPG5DxxUPR4gZOY5gC7a5JSB7N0wzyfJNrHWv408Z9u+S58QeM\/G4DGImHo4YcOU4gi3Y5paA7N0gZmT5Jta6dvykGbPosgn\/KXlO7DETj8sgZuLhiAFXjiPYgm1uCcjeDWJGlm9irUPMyLoGYkaOL9iCrRwBOctot3Js2TLEjCzfxFrXjn\/giReJE4D1hT1m4nEZOq54OCJ6IMcRbME2twRk7wYxI8s3sda143s9NI14aTZf2GMmPndBzMTH0m8JbMFWjoCcZbRbObaIzMiyTbR1LWYu\/cNE4k3zIGbidRc6rnh5eq2BLdjKEZCzjHYrxxZiRpZtoq1rMdPqt2Ppjc\/Kq2fFHjPxuQwdV3wsEZmRYwm2YJs7ArJ3wjSTLN\/EWteOr9VxOPFBk3xdc1p94qXZuKITgJiJzjCTBbAFWzkCcpbRbuXYIjIjyzbR1oPEDDbMi89l6LjiY4nogRxLsAXb3BGQvRMiM7J8E2tdO35r2\/8teUZsmBefuyBm4mOJAVeOJdiCbe4IyN4JYkaWb2Kts+M7dvstbW8zuOQZsWFefO6CmImPJQZcOZZgC7a5IyB7J4gZWb6JtQ4xI+saiBk5vmALtnIE5Cyj3cqxZcsQM7J8E2s9aJpp6cCz1F4zuKITQMcVnWEmC2ALtnIE5Cyj3cqxhZiRZZto634xgw3z4nUXOq54eXqtgS3YyhGQs4x2K8cWYkaWbaKt+6eZIGbidRc6rnh5QszI8QRbsM0NAdm7YJpJlm9irfsjM9gwL15XQczEyxMDrhxPsAXb3BCQvQvEjCxfK+s7d+6kbt260aJFi0rq9e3bl4qLi7PaWbZsGXXp0oV27DhwxpK+Tj\/9dBo3bhxVq3ZgUzzvBTFj5RrrwhAz1siMK4CtMSrrgmBrjcy4Atgao3IqCDHjhC3+SuvXr6f27dtTgwYNSgSIFikXXHABDRs2LONNZ82aRf369aOJEydS8+bNjR7OL2awYZ4RNuNC6LiMUVkXBFtrZMYVwNYYlXVBsLVGZlUBYsYKl1zhTIJk5MiRNGXKFJo2bRrVr18\/8AG4zJw5czJGYYIqQczI+ZIto+OS4wu2YCtHQM4y2q0cW7YMMSPLN7J1FipjxozJGnXp06ePuk+26I3\/QfxiBrv\/RnZVKQPouOLl6bUGtmArR0DOMtqtHFuIGVm2sVhnobJ48eKMkRk9PcV5MStWrCi5Z9u2bbOKG7+Ywe6\/sbirxAg6rnh5QszI8QRbsM0NAdm7IDIjyzeSdZ566tmzJ2VLAg7KqwnKvwmLzIz6dT3iFU2ZprIivUgZrAwxI+d0sAVbOQJyltFu42XL45z3Wrt2LXXq1EmlXDRq1Cjem+XZWrn9+\/fvz\/MzON9ei5SioiKrXBh9Qy2ERo8eTW3atDnoOfyRmVozblJleFXU9ddf7\/zcqHiAwJ49e2jjxo1KHFaoUAFYYiQAtjHC9JkCW7CVIxCv5SeffFKlX\/gviJl4OUeyFlXI8M21jR49egQu7faKGd4wb\/SF5dQz8+CL6Ewk96nKa9asIf6y3XTTTan7lRCdTjQLYBuNX7baYAu2cgTitcyRGW90ZuHChTR8+HBEZuLF7G5NR1Sy7RFjYt1WzPC5TLjiI5Dm+dv4KLlZAls3bia1wNaEklsZsHXjZlorzXwLbppJC5mw5F2vczNNJ4XtPeONzGD3X9Ovi3m5NH+xzCnIlARbGa5sFWzBVo6ArOU0t92CEjOmG+T5m4PeNXjdunUlK55MbHnFTJMf7SRezYQrPgI6Ge2ZZ57BNFN8WJUlsI0ZqMcc2IKtHAFZy0gAluVrbJ2XYM+YMSNjeZ3Im2nfGX\/9sGMQ2PG8a\/DMI7tTpc\/mUdV\/jjd+VhQEARAAARAAgaQROOOMM2jy5MlJe6zIz1NQkZnIb+tggAUN\/8EFAiAAAiAAAoVOgJdkp21ZNvsEYqbQWyaeHwRAAARAAATKOAGImTLeAPD6IAACIAACIFDoBCBmCt2DeH4QAAEQAAEQKOMEIGbKeAPA64MACIAACIBAoROAmCl0D+L5QQAEQAAEQKCME4CYKeMNAK8PAiAAAiAAAoVOAGKm0D2I5wcBEAABEACBMk4AYqaMNwC8PgiAAAiAAAgUOgGImUL3IJ4fBEAABEAABMo4AYiZDA1An+e0aNEiVYJ3TJw2bRrVr1+\/jDcZ89f3Hh9RvXp1mjhxIjVv3jyrAX0oqC4E7sG4XNh6La1fv57at29PHTt2pOLiYnOnlpGSLnz9fQaj0keslBFsRq\/pwla3V70bO\/oFI9SBhZg\/X8OGDXM3ksCaEDMBTtGdUoMGDUoczg1g8eLFEDSGjZh58cGe48aNo2rVqlGm87K85rjM0KFDSw0AbOfNN980EkKGj1bwxVzY+l9aDyhh55MVPCyHF3Dhqwdb7jO8bd7fnh0eJ1VVorBt1aoV+uOIrUH3sW3btoWYiciyIKoHDbz4JWvuOh1d8f4qDRKIXouZPgf30txd2Po9541+QczEw5f7jClTppT6sRPW5s2\/Ueko6dp2g\/rjZcuWUZcuXahHjx6ILBo0D3\/UEGLGAFoaivh\/Peh3yvTvaXjnON8hqGNn+5n+Pdu9tZjx\/iqL81kLzVZUtppnt27dVAQB00ylW4ALXz1QtG7dGgNrli+UC1vdb4wZM6ZUdBZixrzn0u2TI+UTJkygO++8k7yzDuaWkl0S00w+\/2T7NYWpJrPGnEn0mUw1+e+ATqs0kShsvW379ttvR85MQHN24asF4sCBA2nlypVqqpQv0zwxs29V4ZdyYctvHfSDBtPPbu0hzdFCiBkLMeMSWXBrcoVdK1OnxWHmfv36Gee\/eH9RIPn6QJuIwtbbftkWEoAP\/p658NWCe8eOHeQN3wflgBX2Nzva07uw9d7Rmzh8+umnl+QmRXuqslUbYqYM+TubsyFmzBpC1E5L30V3XlgR8gN3V7Z6wB0yZAi1adOm5NcuppnMIl\/ZhLhmW1RUVGqA1X0J30EnBZt9g9JZyrXt6lwbb34XhKJbG4GYceNWkLUwzRTdba7h5KBfYRAyZoNttim8oDaNxOrgdu7SdrWYueCCCw5aIYIfQOFC3KTtBglC5DDa99UQM\/bMCroGEoCjuc810Y\/v6s26h5A52A8ubL3TIEGexZ4dP1Bx4ZstSR1iJhpbRMqj9cX+2hAz8fJMvLWgDgi\/ZM3dFhSSN\/kS6TLLly83zqsxf6p0lHRl6397tOfg9uDKN2hxgEmbT0erNHsLF7bZpuoQmTHj7i2V5jaJBOCA9hC0ARZWMpl\/cYISd01WMmGFQjhjV7YQM+FsvZFBXsaqk85N2m7QVJNJPbOnSkcp17aLnJn4\/A8xEx\/LgrGE4wyiu8q7+iBomapXIG7YsEFtgsUrQoIurF4oTcWGbdARHIjMZG\/fLnz9W+5jaXYwYxe2\/qlSsHXrnyFm3LihFgiAAAiAAAiAAAiIE8A0kzhi3AAEQAAEQAAEQECSAMSMJF3YBgEQAAEQAAEQECcAMSOOGDcAARAAARAAARCQJAAxI0kXtkEABEAABEAABMQJQMyII8YNQAAEQAAEQAAEJAlAzEjShW0QAAEQAAEQAAFxAhAz4ohxAxAAARAAARAAAUkCEDOSdGEbBEAABEAABEBAnADEjDhi3AAEQAAEQAAEQECSAMSMJF3YBgEi+u677+iFF16g3bt307XXXmvN5PPPP6dx48ZRcXEx1atXz7p+lApvvPEG3XPPPcRnFTVs2JCmTJmi\/ra9sp0sbWsrCeX9Rxf07dtX+Scfl3+rf5w2nw8v4J75JgAxk28P4P6pJxD1HKSgU9xzAW379u108803KyHTs2dPOuKII+jss8+mKlWqWN8+rWLmyCOPVGeKHXvssXTcccdZc4mjwpYtW2jhwoX0z3\/+U4leiJk4qMJGoRGAmCk0j+F5C45AoYqZOAVInLaS0ACS+D76dGmImSS0EDxDrglAzOSaOO6XKgL79++nl156iR566CFau3YtVaxYkVq0aEH33nsvHX\/88ZTttF8+wfbRRx+lGTNm0JdffknlypWjRo0a0W233UaXX365+n\/vCcMMrm3btjRs2DDF8KOPPqI\/\/vGP6hc5X6eccgrdfvvtdMYZZ4Qy5qmrESNG0Msvv0y7du2iBg0a0E033aSmwSpVqkR6YPQayjaVwhxee+01+utf\/0qffPKJ4nDWWWfRwIEDqVmzZqQH\/5YtWxKfgP7II4\/Qxo0b1X35mfX78v327t1LTz\/9NE2YMEExZduHH344de3alW644Qb1fHzpiNWNN96o7vv999\/TfffdR+3ataNVq1ap\/54\/fz4dcsghdM0119DJJ5+s\/DJx4kRq3ry5ssFTf3yf8ePH01dffUWHHXYY\/eY3v6FbbrmFqlWrlpFjkJjxnkh83nnn0Z\/\/\/GfatGmTagd33nmnimqxTzNdmSJwppE5iJnQZo8CKSYAMZNi5+LV5Am88sorSnz8\/Oc\/p4svvpi2bt1K\/\/u\/\/6sGrWeeeYaqV6+uhMHQoUPpZz\/7Gf3qV79SYuPQQw9VQuWtt96iq666ik477TRavXo1TZ48WQ2qY8aMUTbfe+89NfjyoPz73\/+eTjjhBPrpT3+qbHL9Jk2aUOfOndWLTpo0iVauXKnEziWXXJLx5VkgsChgMcF\/H3XUUUpQzZs3j9q3b68GfB6EZ8+eTQ8\/\/DAdffTRWadSWGxwNIDfsaioSD0PD\/b8PHXq1KEnnnhCCQq2zf\/OUzPXX389Va5cWYkIfg5mxgzY1pAhQ2js2LFK4LRu3Zq2bdumbLFIuvvuu9WzaDHDIrJ27drUvXt3dY9f\/OIXSkjxe7Ev+Flq1KihBAsLFxZKWsyw+GDRsnjx4hIfvPvuuzR9+nQlSFlEcN2gK5uYWb58uXqGq6++WrHl+\/Gzs4Br06YNxIz81xJ3KIMEIGbKoNPxyvERYEHBUQ4efPXA9\/bbbyuhMWjQIDV4BU0zcVSFE0Y5CuBNHOXBlCMknKOi\/93\/y5xzWXiwrl+\/vooI6RwWHqj5vpzjwoN30EDMYoEFAScks5BgYcQXJymzGOGIiBYWplMpn332GXXq1EkJABZSOnLyzjvv0K233kp33HGHEnIsZjh68tRTT1HTpk3VffX7cm4Ol2UR1aNHD5V\/wpEVFgV86XtwVEdHppgLP\/MDDzxAHTp0UOX0+3GU6PHHHyeOBPHlFXBazLCA48gW2znnnHNKGgU\/N4sjvj9HwmzFzNKlS0sJSs5p4agSC9xMftHijBOsp02bpnyrL0Rm4vu+wlJ6CUDMpNe3eLMcEGAxwUKGB2OOGAStNrLJmdFlL730Uurfv796A\/9gtmTJEhXZuOyyy9TUhffiiA1He7xTKd7PWSxcd911SkwMHz68RCxwGY4o8DtwpIjvbSpmeMUTCy+eMssUefBOM3G0R0+3mN5DT+FwpEc\/N3PhCJb3XbO9H0+rcYIsl+cpOY50ceSLozMcQdMXR3T4Gc8999wS4eRvStkiM1yW7+OdpuJn5XbCQu6kk04KbJmYZsrBFxa3SC0BiJnUuhYvlgsCHJXRgyLfj3M7eCqJRYGOPmQTM3v27FFTECwkFi1aRBzV4ciKNzfGP8gF5bN435WFAg\/yF1544UEIgsSSLuQfoE2FBg\/SLGQyCSi2n8lWpn\/nKBOz\/de\/\/qUEx9y5cxUnnorSQiFIzHAdjtLwH470+IVev3791HNyHk+3bt0U80wXR5OYI08J2ogZzgPS0SNdzySfBWImF99Y3COtBCBm0upZvFfOCPDUBg+0U6dOpZkzZ5YkAuvclSAxw4M1T5FwPglP8fCUEO\/fwompr7\/+eqmoQCYx47JqRULMBIkKk8E\/SOQwSxYbnDfDiclVq1ZVOTatWrVSwoMjX9nEzJo1a1QSc8eOHY3ETFAUxaThhCUAB4mZXr16ZRSZfE+IGRPyKAMCwQQgZtAyQCBmAh9\/\/LHKeeHkTxYcnDPB+SI8wOo8mL\/\/\/e\/EgxvnUvTu3btkSkIPxpwk7M0N8eZS8KDOOTM81aSnokxfwWSaiaNKHNUwjcxkmmbixF6e0rrooouUwGAGLEq8A73\/Hnqqi9\/\/T3\/6k0ru5UvnnXDScDYxk+39vKLrJz\/5Cf3ud79Tq804T4gTqW2ubGKGc5jY7\/ys+uJ7s3Dle2XajyZTGV7txXk8\/lwa\/\/OaRH9s3hFlQaCQCEDMFJK38KyJIrBjxw76n\/\/5HxVZ4YFIJ+Ju3rxZCY26detmFDNcnge8J598siRJlaMSPGDxMt5f\/\/rXGcWMTgDm3A6u37hxY8WFoz1cl5dqcxKvnubyQgtLAOYBlxODeYrFVMxkSgB+8cUX1bJrXsXDOSomYkYPyJzfwnlD+uLpNxaCLEKyiZlMCcBaDHEEzZsAzCvReJqQ82Z0Hg9Pa3ESMi\/n5s+DrrDVTN7kal52z+2BpyDZ55k2HeQVWyzgRo0apVZl8fWf\/\/xH1f32228hZhL17cfDJI0AxEzSPILnKSgCvGKGp4t4VRALEL6ef\/559YtfL8XV0QL+jKMxnFj6\/vvvqwGTp5Y4OsMDHK8w4qgLr\/jhgVxHMPQgx4MaL\/\/me7366qtq5RInmbIdTmDl+jwQs8DiyFCmPU1MlmbziiRTMeNdms0RFU4g\/ve\/\/62SXU899VQl9DiB10TMcGSG35MZ8HvxtNKcOXPUUvR9+\/Yp4ZdNzDBjjm5xRIiXYvPSbLbB4oLf+0c\/+lGJmGFRyAJpwYIFSjwwc87Ree6556hWrVpq5RHv+2MrZtiHnKjsvTffy7u6Sos27949+t15BRevaOOLxSo\/C4sxHZnRfuHcHG+iMSIzBdV14GFjJgAxEzNQmCtbBDgqw1NAPFDxYMmXf5M0Hux5YGfR8\/XXX6u8iQsuuEBttvfggw+qhF\/ODeF9ZVjs8Aop3ltFL+PlpFYWLrwfCm9Ep5NSeQt7rs\/CiAd\/Hnh56oRFFe+5ku0K2jSPp5auvPLKkqXVpmKG7+PfNI\/FGYsXLbhsEoB5Tx1e1s7L1\/UmhJy4y5GeN998U+3f8+Mf\/1iJJP9qJv3OvGmejlLpTfP4mXiax5uorDcufPbZZ9X+PuyH888\/nwYMGJD1DKpskRlmwb7kKBdHz1jQ8TJvbhf6ChIz\/Jn33VkQ8So59utf\/vIXiJmy1bXgbS0JQMxYAkNxEACBwiTAU1ecpM1\/XA7L9L617dLsXBBDZCYXlHGPpBKAmEmqZ\/BcIAAC1gR4qTvnv5QvX15FuPReL\/rQTJ568+8BY32TDEvN9V44bC+Oe9g+F8SMLTGUTxMBiJk0eRPvAgIgoKaReJrqzDPPVPv1cGK06VEPpvh0ZMZ7ajbnsPDeNbkWMzg129RrKJdmAhAzafYu3g0EyiABzmPiZGjeKZjzmDhKw6upOO+GE5SzHfZoikuLGZ0nxYm8vKQ9H2LGf5ipy\/5Dpu+NciCQVAL\/H0WcdgOBsGCBAAAAAElFTkSuQmCC","height":271,"width":451}}
%---
%[output:4e40d541]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:563fc97c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:6761ce4b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
