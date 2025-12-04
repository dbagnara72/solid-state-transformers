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

model = 'sst_spdab_npc_inv';
rpi_enable = 0;
rpi_ccaller = 0;
%[text] ### Settings voltage application
application400 = 1;
application690 = 0;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 2.5e3;
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
ubattery = 750;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 750;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:1dd96712]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:174c5e41]
%[text] ### AFE simulation sampling time
dead_time_DAB = 3e-6;
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
% single phase DAB
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8) %[output:6e4cc00d]
f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:01f62bda]

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
% iph_grid_pu_ref = 3.75;
iph_grid_pu_ref = 2.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:231dc037]
Iac_FS = I_phase_normalization_factor %[output:1bc16b1f]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:26989b55]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:75c58cb8]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:6a06269e]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:0c76ff32]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:1ba8c347]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:16217f48]

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
figure;  %[output:28c3084b]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:28c3084b]
xlabel('state of charge [p.u.]'); %[output:28c3084b]
ylabel('open circuit voltage [V]'); %[output:28c3084b]
title('open circuit voltage(state of charge)'); %[output:28c3084b]
grid on %[output:28c3084b]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:2330891c] %[output:55c0aace] %[output:6d4591c7]
%[text] #### DEVICES settings
% danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
wolfspeed_CAB760M12HM3;

dab_mosfet.Vth = Vth;                                  % [V]
dab_mosfet.Rds_on = Rds_on;                            % [Ohm]
dab_mosfet.Vdon_diode = Vdon_diode;                    % [V]
dab_mosfet.Vgamma = Vgamma;                            % [V]
dab_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
dab_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
dab_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
dab_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
dab_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
dab_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
dab_mosfet.Rtim = Rtim;                                % [K/W]
dab_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
dab_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
dab_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
dab_mosfet.Lstray_module = Lstray_module;              % [H]
dab_mosfet.Irr = Irr;                                  % [A]
dab_mosfet.Csnubber = Csnubber;                        % [F]
dab_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
dab_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
dab_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]

danfoss_SKM1400MLI12BM7; % 3L-NPC Si-IGBT
igbt.inv.Vth = Vth;                                  % [V]
igbt.inv.Vce_sat = Vce_sat;                          % [V]
igbt.inv.Rce_on = Rce_on;                            % [Ohm]
igbt.inv.Vdon_diode = Vdon_diode;                    % [V]
igbt.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.inv.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.inv.Rtim = Rtim;                                % [K/W]
igbt.inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.inv.Lstray_module = Lstray_module;              % [H]
igbt.inv.Irr = Irr;                                  % [A]
igbt.inv.Csnubber = Csnubber;                        % [F]
igbt.inv.Rsnubber = Rsnubber;                        % [Ohm]
igbt.inv.Csnubber_zvs = 4.5e-9;                      % [F]
igbt.inv.Rsnubber_zvs = 5e-3;                        % [Ohm]

% wolfspeed_CAB760M12HM3; % SiC Mosfet for 3L - NPC
% inv_mosfet.Vth = Vth;                                  % [V]
% inv_mosfet.Rds_on = Rds_on;                            % [Ohm]
% inv_mosfet.Vdon_diode = Vdon_diode;                    % [V]
% inv_mosfet.Vgamma = Vgamma;                            % [V]
% inv_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
% inv_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv_mosfet.Rtim = Rtim;                                % [K/W]
% inv_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
% inv_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
% inv_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
% inv_mosfet.Lstray_module = Lstray_module;              % [H]
% inv_mosfet.Irr = Irr;                                  % [A]
% inv_mosfet.Csnubber = Csnubber;                        % [F]
% inv_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
% inv_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
% inv_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]
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
%   data: {"layout":"onright","rightPanelPercent":34.7}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.278409090909091e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     4.953480089180958e-04"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.265986323710904e+02"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ldrso_pll","rows":2,"type":"double","value":[["0.005131660589376"],["1.166353750716398"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000400000000000"],["-39.478417604357439","0.993716814692820"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ld_fht","rows":2,"type":"double","value":[["0.006220353454108"],["1.086643444559938"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.142908164917381"],["7.327264123984933"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfEAAAErCAYAAADOl7o0AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ1wXtV55x+SbmOlgdiySYwiXDtEJuTLBheiOAaSdYHN7tqk2WQkOTtlXJEqAYeZxa4kk8IMgdiSxtClQBpvVlHoTG1r20Jib5uh1CQEarwJH1bbjVML\/IURJAbhEBIRxot3nmsf+ejovu89\/\/ve9773vvf\/zjBG0nnOPff3PO\/zv+fznnHixIkTwg8JkAAJkAAJkEDuCJxBEc+dz9hgEiABEiABEggIUMQZCCRAAiRAAiSQUwIU8Zw6js0mARIgARIgAYo4Y4AESIAESIAEckqAIp5Tx7HZ0wmMj49LZ2dn8IfBwUFpbGysGqbR0VFZvXq1LFmyRPr6+qShoaFq17IrTvMefW7IcPjyl78sbW1tPiaZLtPf3y+bN28O2tjV1SU9PT1Qe4eHh2X9+vWycePGTPLQ+9u9e3fVvx8QNBauiABFvCJ8NM4SgTQFrpSIa5K8\/PLLpbW1tSpoSt1jta9b6mbiiIKKyCOPPAIJZBwb1AF6jVWrVk2a1aOI19tDF+rjeixPEa9Hr\/KeUicwMTEhvb29smPHDtmyZUtqIq4jAGlcNwyoEYQVK1Z4C7LpqSICGccmTgAYEUfa5l4n6z1xE6eHDx9mbzxOkGTQhiKeQadkoUn2sKK2xx4etHuhixcvlttuuy1osiZze2jZ9BpHRkam\/d2u4+qrr5Zrr702KNPU1CRDQ0PS0tISisEWS7e820vVv5vh9eXLl8udd94pixYtCpKXLX5R9eiwvLnuk08+GbRPP2Y4\/ZZbbpGvfvWrgYCbj8tCf2+Y2iJvhMMub4TA1GWLin2P99xzjwwMDIRe98iRI0H7xsbGJttk+9DmqMzvvfde+da3viXm\/pS\/y9qwM9MUYYJl\/Gpf19yve1\/G183NzZMPIi6\/7du3B8PT5mPHh1tf1MOTG4\/l6ioXh+V67DYTbbNpu\/tg4H6\/bLamjhtvvFF27twp+v0xvrPvWX9nrmH7NopLWBxmIeewDfEIUMTjcatbKzdx2zdqElFYonaTr9ajAmoE3P17mMiUE0D9W6m2mYQ7e\/bsKXPiRsTtNqhYhomuLeRuPUmJeFhPz02obnIvxVV\/X0rEu7u7Zc2aNdPYlxNN\/dvZZ58tR48eDR5SwoRVr2mLjdv2UnFhrvvUU0+FCvL9998\/OQ9tx5stUq6Iu3WZv5cS8nIxqzaHDh0q+bBgt8kVcPdBywjopZdeKo8++uiUPBEmxGHfL1eEtUxYG\/X35jpRddtcsj5aULfJtUo3RhGvEti8VmuSlN0TMQlQ78nuhWpvyyQHO0naCcfugdhJX4XS9BRNHebabo\/PsAz7u52QrrjiipIibvdU0HqiRFxHH\/QTNaxdbqRARwdefvnlaUzs3qNyWrhw4ZR79BlOd4f6DXvjT+11u37Xtuj8cNgIgbJcuXJlcL92z91nsZ\/P0Lhbxv3ZMDEPHNr+qGub2Au7H\/M7fdjTey41nG5zNPHk+vShhx4KHgbCetal6nVHY8zog11HuWubnrqJ\/yguSUwb5DW\/1WO7KeL16NUK7qnUU7pJgpq8LrrootCV2XaZgwcPhvautGl2Hdr7MyvJoxamRfUgSomkndT0+mg9SYm4fW0VZP3YolEqudoi9oUvfMFbxMNGLsKuq+1wpwtK9XS1rIqRaYfNNux67rRCOREvNY3g2pTrVYc9ALr3ZqZq3IcB8+BSSmyj4tP2r11HKb+6vXrDyoh4qWkUe+eFHcvme2lPZZhUYHMJm8KpIGXQtMYEKOI1dkDWLp+2iNtbtKKSJCq+yjZsy5lvPbZAuQlf67a3mPn0xLWMvRhMf9btTO5IhCsiqIi7HN3euvvwkJSIm1gu9fCgK\/bDRNwdlo\/qiedBxMNGfoxf3fgr1RO36yj13aCIZy2Dpt8einj6zDN9xbjD6e6wr5ljLNWrCRv+jBLxcsPgdu9QAWtvpZSI+9ajw5SuwJppBlfEVSh9Fgy5Amf3VN0pCRW9qOF0HSWIEkG3jrjD6XbglurdusHtCrLbKzX3bI\/ImPsxsePahA2nR32pqj2cbh74zAhGKREPG8EwjNyeeKmFiO5Qfrnh9DAuHE6PipZ8\/Z0ini9\/Vb211V7YVk4Eo0S8XNvC5otLiXhUPTr0aOa3XeA+Iq42YavTTV3uCmP7kBRkYZsZVrVt9Lqf+cxnglGCsI9yCrs\/34VtWqd5sHEfHkot+rJt7DJ6zbvuuktuv\/32aYvw1MYVcf1dqUVy5l6jHhrDhpqjRkJsjqXusZwA26J5ww03lIytcnVoG8IWvPkubLO5RI1EVT3J8AKJEqCIJ4qzfirz3WJmbw+L2mIWtlgOGU5XuuWGaqMWjtknuJWrR6\/jbke6+eabZc+ePZMLucJ64naCL7U4z67bnasPE3lbzGxb\/X8j4mHX\/eY3vznl5DE9gMaef4+zxcwWY1tUwrYfltra5nK15+i1TuW2adMmWbduXYDDHlExuwxKbVmL2t9dbouZXsu3h1pqLltHY8IEstTogzIK294X1psv9QCov3dPiCu1tsDU4TNiVD+ZrP7vhCJe\/z5O\/A6jVgInfkFWmCiBsGF7dwdCqX36dkPiHPaS6I0UqDL7ocs8rIStWI9CwsNeogjl7+8U8fz5rOYtpojX3AUVNaDcdEK5aYCwi8Y5drWixhfYOGw4XXFEHZAU9uBVL2fdFzgcJm+dIs4ogAlQxGFkmTNwh5a1gaiAqw3P4k7Xte40FyLg2lI+dKXrrzSuRhFPgzKvQQIkQAIkQAJVIEARrwJUVkkCJEACJEACaRCgiKdBmdcgARIgARIggSoQoIiLyHvf+94qoGWVJEACJEACWSegb4pbsGBB1ptZsn0U8VMivn\/\/\/tw6Me2G60MPeWHUyQzjpaXJjMxwArhF3uOMIs5kAUd93oMevuEEDMgMh0hmZIYTwC3yHmcUcYo4HPV5D3r4hhMwIDMcIpmRGU4At8h7nFHEKeJw1B84cCDXc0jwDSdgQGY4RDIjM5wAbkERx5llziLvTkwbKJMrTpzMyAwngFswznBmec\/\/7ImzJw5HPRMFjEzIjMxwArgF4wxnRhHHmWXOIu9OTBsoEwVOnMzIDCeAWzDOcGZ5z\/\/sibMnDkc9EwWMjD1xHBmZkVkMArgJRRxnljmLvDsxbaAUcZw4mZEZTgC3YJzhzPKe\/9kTZ08cjnomChgZe5U4MjIjsxgEcBOKOM4scxZ5d2LaQCniOHEyIzOcAG7BOMOZ5T3\/syfOnjgc9UwUMDL2KnFkZEZmMQjgJhRxnFnmLPLuxLSBUsRx4mRGZjgB3IJxhjPLe\/5nT5w9cTjqmShgZOxV4sjIjMxiEMBMHnvmmHy29x558W\/+FDPMUGmKOEUcDkeKOIyMgoQjIzMyi0EAM9n64xfl+q17ZfzOT2KGGSpNEaeIw+FIEYeRUZBwZGRGZjEIYCb9Dx6U\/gcPUMQxbNkrnfc5kbSJUsRx4mRGZjgB3IJxhjGjiGO8MluaIo65hokC46WlyYzMcAK4BeMMY0YRx3hltjRFHHMNEwXGiyKO8yIzMotHALPS+XCdF+ecOMYtc6Up4phLKOIYLwoSzovMyCweAcyKIo7xymxpijjmGoo4xouChPMiMzKLRwCzoohjvGpSenR0VLq7u2VgYEBaWlpC20ARx1xDEcd4UZBwXmRGZvEIYFYr731aHnv2GIfTMWzplZ6YmJDe3l558sknZWhoiCKeEHqKOA6SzMgMJ4BbMM4wZhRxjFfqpXfv3i39\/f3BddkTTw4\/EwXOkszIDCeAWzDOMGaLb39cDo+\/zp44hi2d0uPj43LrrbdKR0dHIOQU8eS4M1HgLMmMzHACuAXjzJ+ZireKuH64Ot2fW2olh4eHg2tddNFFXnPipmE7d+5MrY15vdCRI0ekubk5r82vSbvJDMdOZmSGE\/CzWL58ubz59jny6pUnR2op4n7cUiuli9nuu+8++cpXviKaCLiwLVn0fNrHeZIZmeEEcAvGmT8zffnJyq8\/TRH3R5ZeSR0+v\/zyy6W1tVW4Oj157kwUOFMyIzOcAG7BOPNnRhH3Z5VqSZ0L7+zslJGRkWnX3bJlSyDs7odbzDAXMVFgvLQ0mZEZTgC3YJz5MzMr09\/y65fkpW98zt8wYyXr\/i1m7IknH3FMFDhTMiMznABuwTjzY2YvaqOI+zGrWSmKePLomShwpmRGZjgB3IJx5sfMHkqf8dPtMvb3f+ZnmMFSdd8T92HO4XQfSqfLMFFgvDicjvMiMzKLR8DPyhy3qqXf8diAHP7R9\/wMM1iKIi4iFHEsMiniGC8KEs6LzMgsHoFoK3sofV7jDHn1W5+X\/fv3RxtmtARFnCIOhyZFHEbGhW04MjIjsxgEok301aPaE9fPd69bLNdcuYQiHo0t2yXYE8f8QxHHeLFXifMiMzKLR6C8lfbCdW+4\/qu98O3XXSif+L0PUMSrATvNOiniGG2KOMaLgoTzIjMyi0egvJXdC++5aoH0XDU\/99OpHE7ncDr8XaGIw8g4NIwjIzMyi0GgtIm9Il174Xv+9GNB4bx34ijideDERCPdozKKuAckpwiZkRlOALdgnIUzs4fRtcS9HRdIx8VzKeJ4iGXTIu9PYmlTZaLAiZMZmeEEcAvG2XRmroD\/2efOl2s+1jRZMO\/5nz1x9sThTMFEASPj0DCOjMzILAaBqSaugH\/+knPk7vb3TylEEa8Yc+0ryLsT0yZIEceJkxmZ4QRwC8bZaWb2HLj+dtl5M2X79RdOg5r3\/M+eOHvicKZgooCRsVeJIyMzMotBQILtY7oKvf\/BA5P2A59ZKNcue09ofRTxWJizZZR3J6ZNkyKOEyczMsMJ4BZFjzPtfa\/ZtjcQcv3oKvR72i+QZe+bWRJm3vM\/e+LsicOZouiJAgbGV5HGQcaeeAxqRf1uqngPPHhAHnv22CQ1c5iL\/lvuQxGPEWhZM8m7E9PmWdREUQlnMsPpkRmZRREoJd5RvW+73rzn\/6r1xMfHx6Wzs1NGRkai\/DDl74sWLZIHHngAsqm0cN6dWOn9o\/ZMrigxYa8SR0ZmZFaSgDtsbgrq6vM\/uXJ+MIzu+8l7\/q+6iPf09Ehra6sXz927d0t\/fz9F3ItW7QpRxHH2ZEZmOAHcot7jrPeBUfkfjx6ZBkYPbtFjVBHxNpVQxEvEmemJU8TxL2LWLeo9UVSDP5nhVMmMzHSB2gkRufrUS0tsIirYqy45R9p\/b24s8aaI4\/GVWYu8P4mlDZbJFSdOZmSGE8At6iHOVLjfPHFCbtj20ykL1QwN+9xznNB0i7zn\/6oPp+uceFdXl2iPPKufvDsxba71kCjILG0C+PUYZ8VhpsL9rX96Xv78+4dDb1qF+9plzbLyI2dX1OsOqzzv+b9qIm5g6Rz35s2bgx+bmppkaGhIWlpa8OisokXenVhFNKFVM7nixMmMzHACuEWe4kyFe83WvaG9bb1zFe7\/8ME58p8\/fHbZfd44pakWec\/\/VRdxg8tdrZ6l3nnenVhpEKP2eUoU6L1VqzyZ4WTJrL6YhW0Hc+9QhfuPL22W6y4\/F7\/5mBZ5z\/+pibjNd3R0VFavXi1jY2OZ6J3n3YkxYze2GZMrjo7MyAwngFtkJc7MiWnletqmtz1v1gzpPrWyPM7qcpwSe+KVMptib7aVDQ4OSmNjY6J1+1ZGEfcldbJcVhIF1uraliYznD+Z5YeZivZf7h6THx34RcnhcSPa+u8dnz1flr+\/NvnepZr3\/F\/znrgC3bJli\/decjysoy3y7sToO0y2BJMrzpPMyAwngFukEWcq2E8c+oV8e9dYWcG2e9rrrpwv82c3JL4oDSc03SLv+T81EZ+YmJDe3l7ZsWNHQHHFihXS19cnDQ0NSfihojry7sSKbj6GcRqJIkazMm1CZrh7yKz2zFSwfzj6ivyvJ16MFGwj2h8\/b6Z0XHxOINi1GB5HqeU9\/1ddxO3V6VnodYc5OO9ORIO20vJMrjhBMiMznABuETfOzBz2XQ8fktGf\/dpbsHU+e8MftMixXx\/PjWi7VPOe\/6sm4vZq9Cz1uinieGJwLeImisqvnN8ayAz3HZlVh5muEv\/N8Tflrp2HvMTaHhZfe8V8eetbzqjqli\/8riuzoIiX4PfMM8\/IDTfcILfccov3fDfPTq8sGNOyZnLFSZMZmeEEcAuNs7e+8+RQ9sm561fl27uel8OvvD75ju2oWoNh8Fkz5O7298tzr\/ymrgS7HjtxVe+J8+z0qK9M\/v5OQcJ9RmZkhhOItjDD4H\/z1M\/kB\/82LvuPviZjrx6PNjx1mIoW\/PTid8kfLX1PYJOHOWyvmwMKsSdeAhZfRQpEUc6KUpBwh5EZmeEETluoWOt\/W3\/8gjw3\/rr3MLgtzJe+b5a0WS8LKaJgsydeSRRm2DbvT2Jpo6Ug4cTJjMxKEVBxNsPfwWrwZ16R3c8eg4TaiPXx48flqg+9S\/5g8buDyy1730wcfMEs8p7\/qzacnqc4yLsT02ZNQcKJkxmZKQEz\/P3tx8fkiYO\/gOaqDUEzZ71i0dlywdx3TFkVzjjD4yzv+Z8iLiJ5dyIetpVZMFHg\/MisOMx09bcK7Z3\/eEj2H\/XbruXSMUL94fecKV2XNQfi77PvmnGGx1ne8z9FnCIORz0TBYyMR9XiyDLLzPSm\/3XsNfnGI88Fd\/bYs8fgOzRz0p9Y2Cg3\/v7vBvaVzlPzuwm7IfedOIo4RRyOeiYKGFlmBQm\/k\/Qs0o4z09sNRPmZY9I8621Bb\/rgSxOxhr2NKOt2rQVnN8jnLppb9XnqtJmlFw3VuxJ74tVjm1rNeXdiaqBOXYiJAidOZtliZnrTf\/WjF+TxZ49VJNKBWM+aIf\/pI2fLB885OUedRK8aJ8aXE8Vhlvf8n2pPfHh4WNavXx9w1peeHDp0SHbt2lXzM9Tz7sQ4gVuJDQUJp0dm6TIzW7K+s+fnsu9nv4ot0nZvev4c7U2\/W8444+SJZXbPHb+76lgwznCuec\/\/qYm4nqGu7w\/v7u6WNWvWiB4Cs2jRouClKE1NTcHPtfrk3Ylpc2OiwImTWXLMzMKxfz7yS\/ne\/30p2DeNnEjmtsT0nC9rmSXXfKxJJt54s6a9aZzUaQvGGU4v7\/k\/FRE3B7+oUC9cuFA6OzsD0W5tbRW+TxwPulpbMFHgHiAzf2ZmqHv4sX3y6HP\/LzCMs3DMXNGItL5d6\/cvmC1L5p0V9KLrcQ8148w\/zkxJirgHM4q4B6QcFWGiwJ1FZieZmV70k4dflX\/c+3LFvWh7uPvcxhly9aJ3yfvn\/s6kgypd7Y17urYWjDOcP0Xck5nOh+v8tz2cbnrl7e3t0tbW5llT8sXy7sTkiZSvkYkCJ14EZmYe+px3\/rbc+4Pn5Jmf\/7qiYW4j0MG\/s2bIh5vPlE99cE4Avx570XhUTbcoQpwlwcmuI+\/5P5XhdANMh85XrVo1xQcbN26sqYBrY\/LuxKSDOqo+JoooQvWXXM0Q9\/E3T8ifP6yHmMTfdmXTMYeaaC\/6yg\/MkQvPPTP4s\/7+n0ZG5eOLWnDYBbbgdxN3ft7zf6oijuNNxyLvTkyH0umrMFHgxLPMzAj0G8ffFN1y9eShVyvuQbu96P+y5N1y3py3QwvGsswMj4B0LMgM55z3\/E8RZ08cjnomChhZzQ57MUPcP3v1N\/Lwv40nMgftCvRHF7xTLl\/YONmDTmoemnGWnzjDW5odC4q4hy98X0va1dVVk61meXeihwsSLcLkiuOsBjOzSGz4iRfl0dFXgkZVstXK3NXkYSWzZkjre2fKf\/3oOZOrudPcG10NZrjn8mVBZri\/8p7\/U+uJ68K2bdu2yeDgoDQ2nnxqN+KuC9tWrlxZsz3jeXciHraVWTBR4PwQZkacf\/LCa\/J3\/\/KSHHo5mflnuwet260+9t6ZMn92AzTEjd95fAuEWfyr1JclmeH+zHv+T0XE7S1mujfc\/tj7xPft2yd6KMwDDzyAe6ICi7w7sYJbj2XKRIFjU2Zvfec5J3vL46\/L7gPH5If7qtN71kViV1wwWy46tR+6lseA4qROWzDOcHpkhjPLe\/6niHNOHI56JoqpyMzCMP335V+9IQ\/\/dFwOVPDSDNch9vD2JQveKX\/Y2jTlyM+k5qDhQKiyAeMMB0xmODOKuCezqOF03Sdu9pLfddddnrUmUyzvTkyGgn8tRUkURpyVzPPHXpfv7Dkqe194LZF5Z3toW\/dAa+\/5k+c3ytyz3hY4Iqtnc\/tHSeUlixJnlZPi6EUlDPOe\/1PpiRvAYfvE9UUoOsSuAn733XfL0NCQtLSkuzc0706sJIDj2OY9udri\/NJrb8j9T\/9c9BzuJBaFGZ527\/myhbNkfsOEzJ17DsUZCLi8xxlwq4kVJTMcZd7zf6oijuNNxyLvTkyHUvaf9m1xPvrLN+SBPdUV5w+\/50z51IdOniAWNe\/M5IpHKZmRGU4At8h7\/qeIc04cjvq0k6stzvr\/\/\/tfjspPxpIb1naHts+f+zty3eXnyvPHfhMpzr7w0mbm264slyMz3DtkhjOjiHsyGx0dldWrVwevI3U\/+kpSe+uZZ5WJFcu7ExMD4VlRUonCFudHRl+R\/3PgF3I4we1UYeL8xcua5d+99S2Td5rWorCkmHm6qC6KkRnuRjLDmeU9\/6fSE5+YmAj2gC9dunRyP3hHR8e015Li+JOxyLsTk6HgX0tUorDF+Qf7xuVHB16Vw+PJ7XV2xfkDTe8QFee3nHFG6uLsSy2KmW89RSpHZri3yQxnlvf8n4qIu\/vEdS\/4\/Pnzgxef6GK3rVu3Sl9fnzQ0NOAeSMAi705MAIFXFUacn3vuOZl422z5+389Ks\/8rPI3VdkXtxeELZ53llz78fdMaVtaPWcvIEAhJlcA1qmiZEZmOAHcIu\/5vyYirivRDx48GByxah\/2Yk5yw91QmUXenVjZ3U+11tPCmma+Te5++LA8ezQ5gQ7b6xwm3kneS5bqoiDh3iAzMsMJ4BZ5z\/+piLhi1d63flzhfuihh4L3jEf1xM2Q\/I4dO4J6yp2z7paNKp93JyJha16Isf2ffy4\/feFXFW+rssV50blnyheWNddFzxlh6lOWguRDaWoZMiMznABukff8n5qI2\/PiOoyuor5582Zpamry2htuPwTYZ65rXe5H\/7527Vq56aabvPac592JYWGrPWr9bP3xC8Gbqx579uTPyMcW6FUfPUeaZ84IVmvrcDrf84yQlJq9xQxrZbZKU8Rxf5AZzizv+T81EcfRlrewRd0tqSvhN2zYIHfcccfky1bK1ZZ3J5q56jVb90JibUT6yg\/Mlk99cE6walt\/FzXvzESBRzOZkRlOALdgnOHM8p7\/UxFx3xeg+M6Jl6tPXYjOs+fRiVt\/\/KJs\/dELXqKtorzsvFnSfdX8yVdK4qF+2oKJAqdHZmSGE8AtGGc4szzmf\/sucyfiZhh+xYoVJefRdeHc+vXrJ+8zah+6OtF8du7ciUdBChZjrx6X42+ekNsfflmefP710Cs2nfVbwe8\/v\/gsaZnz23LOmb8l5ndJNvHIkSPS3Dx17jvJ+uuxLjLDvUpmZIYT8LNYvnz5lIL79+\/3M8xgqaqKuCumpe6\/3CK1UjYq5npwTNiCOPdv5cpq\/Vl+EtO57YEHD4T2uIOh71kzpPuqBV7D4EnFH5\/2cZJkRmY4AdyCcYYzy3L+97mbqoq4aUDU8LdPQ90yOu\/d3d0tAwMDkYvXospm0Ykq3mu27Q2Gv+2PCvdnLny3\/PvzG4OXadTiw0SBUyczMsMJ4BaMM5xZFvM\/chepiDjSIN+yyLx31EK3rDjRbP9a+fWnpwn3x8+bKR0Xn3wLVq0\/TBS4B8iMzHACuAXjDGeWlfyPt\/ykRW5E3F6Nbrar6fY03Xduf9ytbOXKGrssOFEFXMXb7nlrr\/s7X1ocHCcatWI8bgDEsWOiwKmRGZnhBHALxhnOLAv5H2\/1aYuqibgZQh8ZGYlsX9TCM63APcDFXthm\/qbnseu7ycuVDWtMLZ2oov3Q3pflT\/5232TTVLDvab8gE73uMF5MFJEhPa0AmZEZTgC3YJzhzGqZ\/\/HWTreomogn0bi06qilExff\/viU3reeeHb9J87NVM\/b9QMTBR6ZZEZmOAHcgnGGM6tl\/sdbSxEPZVYLJ7rD51nvfdvgmCjwrx6ZkRlOALdgnOHMapH\/8VaWtki1Jx625Wzjxo3B28xq+Unbibry3F68pgK+\/boLM937pohXFqFMrjg\/MiMznABukXb+x1tY3iI1EVcB37ZtmwwODk4ehRp1BnrSN1uqvjSd6Ar48Bc+IldcMDutW03kOkyuOEYyIzOcAG7BOMOZpZn\/8dZFW6Qi4kkfuxp9W1iJtJyoQ+g6B24+93ZcIB0Xz8Uam4HSTBS4E8iMzHACuAXjDGeWVv7HW+ZnQRFP6cS2ehFwDSsmCr8vl12KzMgMJ4BbMM5wZhRxT2ZFHk53F7Hp\/HcWDm3xdN20YkwUODkyIzOcAG7BOMOZUcQBZkVd2Lby3qcnzz7X4XMdRs\/zh4kC9x6ZkRlOALdgnOHMKOI4s8xZVNOJ9kI2XYW+508\/lrn7RxvERIES4xQETozMyCwOAdymmvkfbw1ukeqceHt7e823k4UhqpYT7XnwvG0jKxdKFHH8i0ZmZIYTwC0YZzizauV\/vCXxLFIRcW2aO5S+ZcuW4IjULHyq5UR7GD2vK9HD\/MNEgUctmZEZTgC3YJzhzKqV\/\/GWxLNITcTt5unLTDZv3hz8Sl9iMjQ0FPk60Xi352dVDSfaw+jLzpsp26+\/0K8xOSjFRIE7iczIDCeAWzDOcGbVyP94K+Jb1ETEXUHX14rah8DEv514lkk70R1Gr4d5cJssEwUeZ2RGZjgB3IJxhjNLOv\/jLajMoiYibvfEfd5gVtktRlsn7cRv\/PCI3PSd0eDC9TSMbkgyUUTHlFuCzMgMJ4BbMM5wZknnf7wFlVmkJuJZG0K3sSXpxHqVT2nqAAAb7ElEQVTvhSs3Jgr8S0dmZIYTwC0YZzizJPM\/fvXKLVIR8XLHrlZ+C5XXkKQTN3zvgGx66GDQqLwf6lKKLBMFHnNkRmY4AdyCcYYzSzL\/41ev3CIVEa+8mdWtIUknNt74\/aCx9bInPIw8EwUej2RGZjgB3IJxhjNLMv\/jV6\/cgiKe4Nnpw0+8KF\/asjfwSj3OhZtwY6LAv3hkRmY4AdyCcYYzo4jjzDJnkYQTizAXThGPH7pMrjg7MiMznABukUT+x6+anAV74gn1xLf++EW5fuvJXni9zoVTxON\/8ShIODsyIzOcAG5BEfdgVoT3iZvT2ep5Lpwi7hHsJYpQkHB2ZEZmOAHcgiLuwazeRdw+ne2LlzXLhk+3eFDJbxEmV9x3ZEZmOAHcgnGGM6OIl2EW9urRsOJdXV3S09OD00\/IolInfvov9sgPR18pxFC63iQTBR54ZEZmOAHcgnGGM6s0\/+NXTNYilTnxet4nXqQFbRxOj\/\/lY3LF2ZEZmeEEcAuKOM4scxaVONFe0NZz1QLpuWp+5u4v6QYxueJEyYzMcAK4BeMMZ1ZJ\/sevlrxFKj3x5JudbI2VONFe0Kar0nVhW71\/mChwD5MZmeEEcAvGGc6skvyPXy15i6qJuD2EvnDhQuns7JSRkZHQO6j1S1DiOtEeSq+3142WCzUmCvyLSGZkhhPALRhnOLO4+R+\/UnUsqibi1WludWqN60RbxPV1o0XohasHmCjwOCQzMsMJ4BaMM5xZ3PyPX6k6FhTxCg57KdLecDv8mCjwLyOZkRlOALdgnOHMKOIezMzQej0Npxd1KJ09cY+ADynC5IpzIzMywwngFhRxnNmkhYr72rVr5aabbpKWltodkBLHifYBL0VZlW4cx+SKBz2ZkRlOALdgnOHM4uR\/\/CrVs6j5cPru3btl69at0tfXJw0NDdW70zI1x3GiGUrXauv9rHQXHRMFHqZkRmY4AdyCcYYzi5P\/8atUzyITIt7f3y+Dg4PS2NhYvTtNWMSL8N7wUsiYKPAwJTMywwngFowznBlFHGc2xUIFfGxsLFc9cXso\/f4vLpJPLKzNw0eF6GObM1Hg6MiMzHACuAXjDGdGEfdgVm5hW1NTkwwNDeVqTrxIrx0Ncy8ThUfQO0XIjMxwArgF4wxnRhH3ZDYxMSG9vb2ydOlSaWtrE\/dnz2qqUgx1YlG3lhn4TBR4GJIZmeEEcAvGGc4Mzf\/4FaprkdqceNiwuemht7e3B8Jeqw\/ixCJvLaOIx49QJlecHZmRGU4At0DyP1579S1SEfGo94nnaXW6PR9+b8cF0nHx3Op7KWNXYHLFHUJmZIYTwC0YZzgzirgHsygRz9Pq9OEnXpQvbdkb3HXRtpaxJ+4R7CWKMLni7MiMzHACuAVF3INZufnv4eFh2bVrV25Wpxd9PlzdzeTqEfROETIjM5wAbsE4w5lRxD2Z6aEu69atm7ISfXR0VFavXi2bNm2S1tZWz5qSL+brRM6Hn2TPRIHHIJmRGU4At2Cc4cx88z9eczoWqcyJm1tRIV+1atWUO9uyZUtNBVwb4+tEW8SLdtSq7TQmCvzLSWZkhhPALRhnODPf\/I\/XnI5FqiKezi3hV\/F1Yv+DB6X\/wQPBBYr06lGXKBMFHmNkRmY4AdyCcYYz883\/eM3pWKQi4mZOvKOjo+a97jCsvk7kfPhJekwU+JeTzMgMJ4BbMM5wZr75H685HYtURLzc6vR0brP8VXycyPnw0wyZKPCoJTMywwngFowznJlP\/sdrTc8iFRHX28nCKvRSWH2caIt4UfeHG35MFPgXlMzIDCeAWzDOcGY++R+vNT2LVES83NnpequLFi3K\/FvMin5euh2STBT4F5TMyAwngFswznBmFHGcWeYsfJz41b\/bL\/9956Gg7UVe1Kb3z0SBhzCZkRlOALdgnOHMfPI\/Xmt6Fqn0xNO7nXhX8nEiF7WdZstEgccZmZEZTgC3YJzhzHzyP15rehZVE3F7MdvChQuls7NTRkZGQu8s68PpXNQ21W1MFPgXlMzIDCeAWzDOcGYUcZxZ5iyinMhDXijilQYtkytOkMzIDCeAW0Tlf7zGdC2q1hN3byPP7xPnIS8U8Uq\/lhQknCCZkRlOALegiHsyy\/P7xK\/fuld0dbp+ir6oTRkwuXoGvVWMzMgMJ4BbMM5wZhRxD2ZRryLN+vvEuaiNPXGPMC9bhMkVJ0hmZIYTwC0o4h7MokQ8y+8T56K26Q5mcvUIeqcImZEZTgC3YJzhzCjiHszy\/D5xW8S3XfsRufIDsz3uuL6LMFHg\/iUzMsMJ4BaMM5wZRdyTWZrvEzcPDTt27Aha19XVJT09PSVbWs6Jjz1zTFZ+\/enAdvt1F8qy9830vOP6LcZEgfuWzMgMJ4BbMM5wZhRxgFla7xPX4Xn9qHCbofz29nZpa2sLbW05J9rHrXJR20l8TBRA0J8qSmZkhhPALRhnODOKOM4sdQtb1MMuXs6JXNQ2nRgTBR7CZEZmOAHcgnGGM6OI48xStfB5DWo5Jy6+\/XHRefF5jTOC7WX8sCceJwaYXHFqZEZmOAHcgiKOM0vNQnvgmzdvlhUrVkhfX580NDRAw+lcmR7uKiZXPITJjMxwArgF4wxnRhHHmaVuEXbQjN0IdaL57Ny5c\/L\/x149LivuOxL83HXJTPnjj3JRm7I4cuSINDc3p+7HPF+QzHDvkRmZ4QT8LJYvXz6l4P79+\/0MM1gqtWNXa3nvo6Oj0t3dLQMDA9LS0jKtKaWexLgynT3xpOKWPSScJJmRGU4At2BPHGeWuoWuii93oEwpJ9pnpnN72Wm3MbniIUxmZIYTwC0YZzgzirgnM+0Nr169WsbGxqZZJP0qUns1utkz3tTUVHKveCkncmU6e+Ke4R1ZjMk1EtG0AmRGZjgB3IIi7sHMR0g9qvEu4h72EndhG0WcIu4ddBEFKUg4STIjM5wAbkER92Dms83Lo5qqFQlzIleml8bN5IqHIpmRGU4At2Cc4cwo4h7MTM+4o6NDWltbPSzSLRIl4j1XLZCeq+an26gMX42JAncOmZEZTgC3YJzhzCjinsyiFpd5VlOVYmFOtFem39txgXRcPLcq185jpUwUuNfIjMxwArgF4wxnRhH3YGaG00dGRkJLJ72wzaNJU4qEOdFemc4z06cSZaJAI4yn3OHEyIzM4hDAbSjiOLPMWYQ58fqte0VffqIfijhFvNKg5YMPTpDMyAwngFtQxHFmmbMIcyJXppd2E5MrHsJkRmY4AdyCcYYzo4gDzIaHh2X9+vWBxZYtW+TQoUOya9eusueaA9XHLhrmRPPik2XnzZTt118Yu+56NGSiwL1KZmSGE8AtGGc4M4q4JzNzfrkef7pmzZrg4BWdC+\/t7ZVyB7F4Vl9RMdeJ3F5WHicTBR5uZEZmOAHcgnGGM6OIezCz94kvXLhQOjs7AxHX7WZZWLVeTsS5Mn26g5koPILeKUJmZIYTwC0YZzgzirgHs7yJOLeXsSfuEdZQESZXCFdQmMzIDCeAW1DEPZnpfLjOf9vD6aZX3t7eLm1tbZ41JV\/MdSJffEIRTzrKKEg4UTIjM5wAbkERB5jp0PmqVaumWGzcuLGmAq6NcZ34Px97Xrrv3xe0c\/zOTwJ3WIyiTK64n8mMzHACuAXjDGdGEceZZc7CdSK3l7EnnnSQMrniRMmMzHACuAVFHGeWOQuKOOYSJleMl5YmMzLDCeAWjDOcGUUcYGbvEzdmul+81i9FcZ3YeOP3g+Zxj3i4c5kogKA\/VZTMyAwngFswznBmFHFPZirg27Ztk8HBQWlsbAyszKr1LC1ss\/eI8+1lFHHP8I4sxuQaiWhaATIjM5wAbkER92BW7n3iWdsnbm8vo4hTxD3C26sIBckL05RCZEZmOAHcgiLuwSyvIs4Xn1DEPcLbqwgFyQsTRRzHRGYVMqOIewLUHve6detkaGhIWlpaMjuczj3i0Q6lIEUzckuQGZnhBHALxhnOjCLuwSzqfeJ2FXqe+gMPPOBRa3JFbCfyFaTRXJkoohlRxHFGZEZmlRPAa6CI48wyZ2E7kXvEo91DEY9mREHCGZEZmVVOAK+BIo4zy5wFRRxzCUUc46WlyYzMcAK4BeMMZ0YRB5iF7RPP2rGr3CMe7VAmimhG7FXijMiMzCongNdAEfdklpd94kbEOy6eK\/oaUn6mE6CI41FBZmSGE8AtGGc4M4q4B7O8bDHjHnEPZ3Jo2A+SU4rJFcdGZmSGE8AtKOIezPIo4toL1944P+yJJxEDFCScIpmRGU4At6CIezLLw3A694j7OZPJ1Y+TXYrMyAwngFswznBmFHGAWdYXttkiztPaSjuWiQII+lNFyYzMcAK4BeMMZ0YRx5llzsI40T7oZfzOT2aunVlpEBMF7gkyIzOcAG7BOMOZUcRxZpmzME7kQS9+rmGi8OPE4XScE5mRWWUEcGuKOM4scxbGiYtvf1z0VaTzGmeIDqfzE06AIo5HBpmRGU4At2Cc4cwo4jizzFkYJ\/KgFz\/XMFH4cWKvEudEZmRWGQHcmiKOM8uchTrxB0\/8RLQnrh8e9FLeRRRxPITJjMxwArgF4wxnRhHHmWXOQp34l\/\/wlKz8+tNB23quWiA9V83PXDuz0iAmCtwTZEZmOAHcgnGGM6OI48wyZ+GK+PbrLpRl75uZuXZmpUFMFLgnyIzMcAK4BeMMZ0YRx5llzkKd+LXhXaJbzPRDEedwetJByuSKEyUzMsMJ4BYUcZxZ5izUiV1\/8bD0P3iAIu7hHSZXD0hOETIjM5wAbsE4w5lRxHFmmbNQJ3564Hvy7V1jQdt4Wht74kkHKZMrTpTMyAwngFtQxHFmmbNQJ35o7d\/KY88e4x5xD+8wuXpAYk8ch0RmZFYxAbwCijjOLHMWFHHMJRRxjJeWJjMywwngFowznBlFHGeWOQt14ll\/9FfBaW3Lzpsp26+\/MHNtzFKDmChwb5AZmeEEcAvGGc6MIo4zy5yFOvHYpweDdunWMl2dzk9pAkwUeHSQGZnhBHALxhnOjCKOM8ucxfwPXSKvXtkftIsHvUS7h4kimpFbgszIDCeAWzDOcGYUcZxZ5izmXfIpeW1ZN0Xc0zNMFJ6grGJkRmY4AdyCcYYzo4jjzDJnYYv4vR0XBGen88Ph9CRjgMkVp0lmZIYTwC0o4jizzFnYIs7T2qLdw+QazYjD6TgjMiOzygngNVDEcWaZs2j6j\/9NXn\/\/yqBdFPFo91DEoxlRkHBGZEZmlRPAa6CI48wyZzH3s7fLG\/M+HrSLp7VFu4ciHs2IgoQzIjMyq5wAXgNFHGeWOYt3\/eE35Pic84N2jd\/5ycy1L2sNoojjHiEzMsMJ4BaMM5wZRRxnljkLI+LzGmcEPXF+yhNgosAjhMzIDCeAWzDOcGYUcZxZ5izmfPGv5c23z+G56Z6eYaLwBGUVIzMywwngFowznBlFHGeWOYvGG78ftIlHrvq5honCj5NdiszIDCeAWzDOcGYUcZxZ5iyMiOv+cN0nzg+H05OOASZXnCiZkRlOALegiOPMMmdhRJxHrvq5hsnVjxN74jgnMiOzygjg1hRxnFnmLCjimEso4hgvLU1mZIYTwC0YZzgzijjOLHMWRsR55Kqfa5go\/DixV4lzIjMyq4wAbk0Rx5llzsKIOE9r83MNRdyPEwUJ50RmZFYZAdyaIo4zi2UxPj4unZ2dMjIyEtivWLFC+vr6pKGhYVp9ExMT0tvbKzt27Jj8W1dXl\/T09IRemyKOuYQijvHS0mRGZjgB3IJxhjOjiOPMYAsjykuXLpW2tjYxPzc1NYUKswr+2rVr5aabbpKWlpbI6xkR55GrkaiCAkwUfpzYq8Q5kRmZVUYAt6aI48wSsRgeHpZdu3aF9sZHR0dlw4YNcscdd0hjY2Pk9YyI88jVSFQUcT9E00rxwQcHR2ZkhhPALSjiOLNELMqJ+O7du6W\/v18GBwe9RZxHrvq7hcnVn5UpSWZkhhPALRhnODOKOM6sYgszP97e3h4Mr7sfFfj169dP\/nrRokVlBV174m\/59Uty1j\/0yM6dOytuX71XcOTIEWlubq7320z0\/sgMx0lmZIYT8LNYvnz5lIL79+\/3M8xgqTNOnDhxIoPtKtkkMx+uBUotbNNe+NjY2OTf3Z\/dylXEeeSqfxTwad+fFXviOCsyI7P4BHBL9sRxZrEtfAQ8rHKdI+\/u7paBgYHQhW4UccwlFHGMl5YmMzLDCeAWjDOcGUUcZxbLImpFerlKoxa6qYjz3HR\/tzBR+LNirxJnRWZkFp8AbkkRx5nFsogaEjeVotvR1E5FnOem+7uFIu7PioKEsyIzMotPALekiOPMYAv3oBdTgVmwpge+6OEuHR0d0traOrmP3Bz2Uu5gGIo47A4ODePIyIzMYhDATfiAjTOjiOPMMmcx54t\/LXd3XhoMqfMTTYCJIpqRW4LMyAwngFswznBmFHGcWeYs8u7EtIEyUeDEyYzMcAK4BeMMZ5b3\/J+7LWa4i6It8u7E6DtMtgQTBc6TzMgMJ4BbMM5wZnnP\/xRxEcm7E\/GwrcyCiQLnR2ZkhhPALRhnOLO853+KOEUcjnomChgZF7bhyMiMzGIQwE0o4jizzFnk3YlpAyUvnDiZkRlOALdgnBWPGXvi7InDUc9EASPjlA2OjMzILAYB3CTv+YwifkrEcdfTggRIgARIoB4I8AUo9eBF3gMJkAAJkAAJ5IwAe+I5cxibSwIkQAIkQAKGAEWcsUACJEACJEACOSVAEc+p49hsEiABEiABEqCIMwZIgARIgARIIKcEKOI5dRybTQIkQAIkQAIUccYACZAACZAACeSUQKFFvL+\/XzZv3hy4bsuWLcG7yPk5SWB0dFRWr14tY2NjEvU+dptjU1OTDA0NSUtLS+FQIswMnImJCent7ZWlS5dKW1sbmfX1SUNDQyiH8fFx6ezslJGRkUJ\/Z5E4s8sW+btZ7otlvoMdHR251IDCivju3btFxWdwcFD27ds3+f+NjY2FS6TuDdvCsnLlyrIiMzw8LLt27ZK+U8lXf962bVvAtUgsEWY2b+W1fv162bhxY+FEHGFmyqoQ9fT0BA+Z3d3dMjAwUKgHRoSZeehRXtpBKep300fAd+zYkduOXGFFXAVcPxrgeX8SS\/qpw02Q+sCzdevWSaEud72iJtc4zDTJrl27Vo4dOybt7e2FE3GEmZbdsGGD3HHHHYV6OHS\/aygz+0GnqN\/NUvnKjFIsWbJEDh8+HGhBHkdjCyni7hBm0Yc03SC3Rym0N+3+TBGfTiAOM32QvPjii+W73\/1uIYfTEWbuiE\/SD655qQ9hFtYTt0fN8nLP1Wrn888\/H1St0zc6TUMRrxbpKtQb1vPWhDp\/\/vzC9YbC8Lo9b6QXpBx1Ht0Mr1fBfZmsEmWmTO+77z658cYb5dZbby2siNsjPOXiTEX84MGDge+LvI4FjTOT63S4uKurKxAqfqYScB928san0D1xeyEDRfx06KKJwlhqor377rsLubANYaaJ9Wtf+5pcc8010tzcXNiFbQgzs3bALEBV23Xr1hUu1hBmZrh406ZNwTAxMqKWNyGrpL0U8Uro1ciWw+nlwSNDdhTwkwQQZlr2kUcembIeo4ir0xFm7nB6UafAyCx50aCIJ880lRrtnjcXtk1F7g5rRi1s46rXk1vy7IVX5ZjZW\/Js8kUb7kSYuTyL+p1FmPHBx09KKOJ+nDJXilvMSrsE2cZS1GFNlx7CzLYtao9SGSDM3ERb1KFhhFnYcHoRpyCixIciHkUow3\/nYS+lnVPuQAmzyEgXyZTqVRbx8BxfZhTx0wQQZvZhL0U+uARhpg87q1atCoAXmVk5GaKIZ1ik2TQSIAESIAESqGcChVydXs8O5b2RAAmQAAkUhwBFvDi+5p2SAAmQAAnUGQGKeJ05lLdDAiRAAiRQHAIU8eL4mndKAiRAAiRQZwQo4nXmUN4OCZAACZBAcQhQxIvja94pCZAACZBAnRGgiNeZQ3k7yRJ49tlnZdasWd6vv9Q9p6+88oqcd955yTbEqs3szV+0aBH03va8vIrS3F9a+5rTvl7VAoMVF5IARbyQbudN+xBATwVL49CISoS4ElsfXkmVUVHVT5pv3MoLm6QYs576IUARrx9f8k4SJpBFEUfbZCPJi1BRxBMOZFZX1wQo4nXtXt5cFAH7KE8ta4ao9+3bN3lcpf7eHCPrHjNrhnxnz54tnZ2dMjIyElzSvMzEfp+zXX9jY2PJptnXsIeUzes4jeHGjRulra1tWj2lyhkRX7lypdx2222BnTtkbR\/T6V5HWa1du1Yuu+yywN6wevnll2X16tXBe+T1c\/PNN8v27dtlYGBAWlpagt+VuqcwCK6I68+\/\/OUvg\/\/0vdg23zB798UfWibsd3l8wImKZ\/69eAQo4sXzOe\/4FIGwl4\/YAuL2eku9FUqr6+vrC17ooUKuw8D6\/mbzgNDe3j4ptuXe+GbaY+praGgIxMd+R3tUT9wtb78EQx80VGyXLFkStNfUv23btmBuXcW4u7t7ivja9ZkHlXnz5k3au\/dofj569Gjwrm\/zvnR9WDDD41EvzQkT8c2bN4t5aAnjagc1RZxf8SIRoIgXydu81ykEosQgSjDdHp4r4mGvIy331rKw4W63fLk2Rb0RzX2rlbY\/aojd\/rsRcfehZNeuXZOirnXaIq0\/269oNQ4oN2QeJuLayzcPHuYaWk4fPtxRDYo4v+hFIkARL5K3ea\/TCNhDz+5q71KC6Q45r1ixIrQn7g5r2xcPGwovdT37rXHlRDxqYV2YYIf9zp1icKcMzEiD3k+YGNt1au\/evEXLhV\/q\/elhIq629kK3cg8fFHF+0YtEgCJeJG\/zXksSsIXLnhe3e3tGlN15atMTdXviauv2IMu5oJRAlxvit+urVMS1LjO3bR4ywnriiIg\/9dRTYobry60DsO+DIs4vKgn4E6CI+7NiyQIQsIXQ9DR1yFbnj3t7e2Xp0qVTFpPZQu2KeLn57zCUaQynu3Pe9jVVcMsNjZvhdFvEw3q99nC69sTXrVsXzI+bRW5RYVSN4fSoB6qoaYWoNvPvJFArAhTxWpHndWtOIGxO3O4N2wu9whZomZ65GU7XG7KF3tSvi9zMUHDYvLQBkdTCNrvna8+TX3TRRdMWrrkibtuatmr7VITDRNx3YZvWYea0o9YiVLqwzUx3mB0F5j7sBX1u8FHEa\/51ZANiEqCIxwRHs\/ogYBK82R5lD5Xb28N0ePmKK66Yso1Mxfvqq6+WW265ZbKn6Qq76Z2brWdKzYhLKYLltmP5LrZbv379ZPVhQ+OmV+yKl3vtTZs2BfPeupjN3L\/dE9eLuAzdLWbuNju1KbU9zox+6L\/mwSdsi5ltHybA9noE9dPixYtlz549wYOE+7Bl7sEdpaiPCOdd1DsBini9e5j3RwIpE1BRDVuR7tsMnzlx37p8y7En7kuK5bJGgCKeNY+wPSSQIwLuvL\/pddv7wtHboYijxFi+yAQo4kX2Pu+dBBIg4J5iV2rrmO+l3BeS3H\/\/\/YFptc5S5wtQfD3Dclkk8P8BDsdAfUQ8QV0AAAAASUVORK5CYII=","height":0,"width":0}}
%---
%[output:2330891c]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:55c0aace]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:6d4591c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
