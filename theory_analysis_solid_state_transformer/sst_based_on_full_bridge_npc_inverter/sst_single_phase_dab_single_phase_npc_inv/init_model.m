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
application400 = 0;
application690 = 0;
application480 = 1;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 4e3;
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
omega_rso = 2*pi*f_grid;
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
omega_f = 2*pi*f_grid;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*f_grid;
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
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:16217f48]

freq_filter = f_grid;
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

wolfspeed_CAB760M12HM3; % SiC Mosfet fpr 3L - NPC (two in parallel modules)
parallel_factor = 1.75;
inv_mosfet.Vth = Vth;                                           % [V]
inv_mosfet.Rds_on = Rds_on/parallel_factor;                     % [Ohm]
inv_mosfet.Vdon_diode = Vdon_diode/parallel_factor;             % [V]
inv_mosfet.Vgamma = Vgamma/parallel_factor;                     % [V]
inv_mosfet.Rdon_diode = Rdon_diode/parallel_factor;             % [Ohm]
inv_mosfet.Eon = Eon/parallel_factor;                           % [J] @ Tj = 125°C
inv_mosfet.Eoff = Eoff/parallel_factor;                         % [J] @ Tj = 125°C
inv_mosfet.Eerr = Eerr/parallel_factor;                         % [J] @ Tj = 125°C
inv_mosfet.Voff_sw_losses = Voff_sw_losses;                     % [V]
inv_mosfet.Ion_sw_losses = Ion_sw_losses;                       % [A]
inv_mosfet.JunctionTermalMass = JunctionTermalMass;             % [J/K]
inv_mosfet.Rtim = Rtim;                                         % [K/W]
inv_mosfet.Rth_mosfet_JC = Rth_mosfet_JC/parallel_factor;       % [K/W]
inv_mosfet.Rth_mosfet_CH = Rth_mosfet_CH/parallel_factor;       % [K/W]
inv_mosfet.Rth_mosfet_JH = Rth_mosfet_JH/parallel_factor;       % [K/W]
inv_mosfet.Lstray_module = Lstray_module/parallel_factor;       % [H]
inv_mosfet.Irr = Irr/parallel_factor;                           % [A]
inv_mosfet.Csnubber = Csnubber;                                 % [F]
inv_mosfet.Rsnubber = Rsnubber;                                 % [Ohm]
inv_mosfet.Csnubber_zvs = 4.5e-9;                               % [F]
inv_mosfet.Rsnubber_zvs = 5e-3;                                 % [Ohm]
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
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     7.990056818181819e-06"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     3.095925055738098e-04"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["4.040205933898089e-01"],["1.129961081535149e+02"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-1.421223033756867e+05","-1.884955592153876e+01"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.866106036232337e+03"],["3.911916400415777e+05"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.500000000000000e-04"],["-3.553057584392168e+01","9.952876110196153e-01"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["4.665265090580843e-01"],["9.779791001039442e+01"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["9.111998627203048e-02"],["4.708907792220440e+00"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAh8AAAFICAYAAADwPpwDAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ+UltV9568ujdbMKGqikwHSGEkam9TRlozxTyfx0I7ppkmIPWWRzQ6usGUyYo7lwNpj2XOyWw57XHCONEigO5hAGiTTbUpMmiq7NGdp0DCSIjWpSQuKFUcIauRPizSp7P4evC937jx\/7n2e7\/O8z3vn+57jUee993fv8\/nd597v+7v\/zjl9+vRpxQ8JkAAJkAAJkAAJVETgHIqPikizGBIgARIgARIggYgAxQcbAgmQAAmQAAmQQKUEKD4qxc3CSIAESIAESIAEKD7YBkiABEiABEiABColQPFRKW4W1mwCixcvVlu3blWzZs1Sg4ODlVbn0KFDavbs2ergwYNq3bp1qre3t1H+tm3b1EMPPaSGhoZUW1tbJfVKq8\/atWvVvn37KmeU9ODit+3bt6tNmzaprq4uLz5F2J44cUItWLBA3XHHHWP85VWBnIml3v39\/Y3cRdrs3r17VV9fX2QrD8OcjwDLVsT\/sErQEJQAxQcUJ43VnUAdxYcM9KtWrVLd3d21EB\/NZBTXfvTAedVVV3nzKcI2TZyV3c7NsnVZS5YsUQMDA7mKbnXxoYVY1e9ILtjM5ESA4sMJExORQHkEigyQRWqVNLjWTXzo+uQZfIuwrYv4sKNkeXze6uJD++Lo0aMtGbnJ47PQ81B8hO7hHM\/n8qvL7MxWrlypli9fHk0nyCcuPGzbnDp1qhoeHlYdHR1RHrujf\/TRR6PpEfm0t7c7dTg6RD4yMtJ4arsu9sBqlvu5z30u+mUtz2H+wtIDmDZq1z1pgLP\/LvWzp110fUw3xQ2ySYNH2gBp27Z\/Ndp5b7jhhmiKweQn9dKDX1y7iPO1ruvx48ejx5L82p92eput\/expg479fHY7yWJrf2\/61X4GeQ7z+6z2nPbaxdk2BYY93WKXHWfbzmOzsNvPxo0bG++X3Z7t91GXZ\/rO5b0x+Urb++xnP6s+85nPRObMqR\/7vU163+sminN0rcxiEKD4YHMYQyCu44vrfOI6UNOQOdAlpTU7mbiBzbQX10Ga36flNzvNNPERJwDiBjB7MKhCfJgdtDlAx5WdxiJpAJXBL018XH311Q3hZL8yJt+sdhHnizR7uj3a\/nfxS5r4yMp\/+PDhaI2EFlCmz+O+cxXJttiKa3O+4iPJpvl+meLjwgsvVC+++OIY7Ob76vIuJaXRbTOJr80prSw74lMkisVuvn4EKD7q55Om1cgc4MzOyOwMdYdgDjJJUQJ7QIsbNPVgZHZC5kATV3YcIN0xmR2uzmv+LU182AOc+Yy67mY99d+KiA95FtdO1U4neXWkIo5tkriLY2529HG\/MHXZceLB9L\/Om+RDnT9OVMRFOeLYxAmxOL8ksdV+veiiixrRt7i2EhdVyhKBaYtCTXtmuri26zrlE2fTrKMuJ+t9NduKi6\/NKJ7Le2PWKe19FJ8lre9IEqJN6zBZcCECFB+F8IWVOSm0n9aZCYGkEKoMiLfeemviL2bz16T8t56SMAdS107YNSSbJj7skL9rZ1eV+LAHZ2Fm72BIGhzNgVgPFibzLPFhtnT7l7YWH0liSP5uc0+LAEh6+xd00vSZrlfSepAsYWdHapIEW9rUk8kmLUKX1J7ihJNru3ddy2E+p+nrrPxJvjbFh+t7Y5d15ZVXxk7zaZ5ZU0e+u57C6q1b\/2koPlrfh7AnSBts7cHDRahIp3TjjTeOC1\/HddZJA6FLJ5w24NpwfMRH1sClbVclPsxBXNjKx94lk8bL9q+P+EibTtHiI25Ni2Zkc08Ly7uIj7j1Pbosc3DNipzY7SNLfGRNK6WJj6R2khXJSVtw6iqQk97XuL\/7+toWH0nPaZd1+eWXp\/4wofiAde21NETxUUu3NKdSLoIiLoxrdo52R2pGPtI60aRB00V8xP2yTiKYJj7izt6Qcxay1pvozjZpXULW4OwqcuSZ9GAj207l88wzz4w5M6SMyIdp03xGzbJI5CNr66RrRMv8hR43DZg2NSjnrbhOu7i2x7j2V4fIh9TLjFSmRSPSfJ0mNJOeM60sl51MWVGa5vSaLDUvAYqPvOQCzGd2rGWs+YhbG5C1\/sC1s4+bN48LNfuIj7g1H3FTUFllI8WHyy4L\/YyoNR9Z0wJF1nzIa2SvIzIHyKwFtXFrXcxBMy6\/LZrkULcsZrqOSeuiXESS2Z7KXPNhinHtm\/3798ceMpYWjYhb55TVlqXsrPcmbs1Hls9M4Z31YyDArjnIR6L4CNKt+R8KtdvF7FyTbGaFuOUpXMVH2qr5PKLHni5IC89nheKzOmx7Xj3rJEszvcu2ZrPuabtd9Imr9pSIDEJy2qne+myzyNrZJH7WOyx0fdOmTeLaTtL25riWHidIdDqx\/dGPfnTMyaGmjbQdWPo72e1injyq87tsCXfZ7eLT7iVtms00YWcKBVPwpU2JZbXltPdmypQp6tixY1ESHX1Je3e42yV\/P94KOSk+WsFLFdcxbiC3OwLzF9MXvvAF9Ud\/9EeN8yFczn6wO+qi0y5JnZ7POR9J00K2eIr75RV3FPb06dPHrMlIClXbA3GW+EhaPGg3E3sQse26MBebMpjLwlbzDBCxNW\/evNRf07JNVftZnyuRtXA0qY5xh0vFiVo7fJ\/E1hywxZ9f+tKX1L333hu1YdOGWUbc1lW9FddFeGj\/pC1y1WlcRberTZ81H3HMbF+b6zWS3huz\/aW1F5f+Rp5T23OZoqm4y2RxOQi0vPiQBjk6Opp57HKcms+ab87Bc8Jk4fzrhHG114NmTV\/kGTg46Hi5oOmJs6aX8kybaEEka5xa8W6apjulhhVoafHhet6\/brg9PT2570aooe+aWiWKj6bir23hadN2PtEB8wF1W8tzt0ttQQVcsbQpNXnsrMheHBq2gfAaTMuKj6TFkXEu0mmXLVtW+c2U4TWZM09E8RGqZ4s\/V5wAySs8dG0k+pH3VtviT0QLvgSSBEge4SFl6+gX4p4b32dh+nIItKz4kMaoP1nTLjJQynzuhg0bGneJlIOTVkmABEiABEiABLIItKT4kF9WS5cujeb+ZCFblviIWw3O9R5ZTYPfkwAJkAAJkEA5BFpOfNjrN1wWnNohW21DkMotprLPnx8SIAESIAESIIFqCLSc+JAoxo4dOxqiwUV8xKHUaxbkOnh9voGZTl8PX40bWAoJkAAJkAAJJBOQXUIhfVpKfIhgWLRokVqzZo3SlwrlFR96EeqcOXPG7YAR4SHTOrt27QrJ13wWEiABEiCBFiVw3XXXKfmxHIoIaSnxkXaSn+9q+jTx8d3vflfNnTs3crScysdPMQIi4lavXk2exTA2cpMnCKRhhkyxTMmzHJ4S9af4wLLNbS0r8qHXd3R2dqrBwcFGOXFRFP2lFh8hOTo3YEBG8gRANEyQJ5anWCNTLFPyJM8sAi0V+Yh7mCzxIXn0uQN6j7iOesyYMWOMIKH4yGou+b5nR5SPW1Iu8sTypPggTzwBrMUQ3\/kgxYc+A8SMdMTdvWF+bzaVEB2NfRX8rB04cCC6O0MOeZs0aZJfZqYeR4A88Y2CTLFMyRPLM8QxqeXFB9bFZ6yF6OgyOLnafP3119VLL72kpk2bRvHhCi0lHXkCIFomyBTLlDyxPEMckyg+YtpIiI7Gvgp+1tgR+fHKSk2eWYT8vydTf2ZpOcgTyzPEMYnig+ID+5bEWGNHhEVMnlieYo1MsUzJE8uT4gPLs7bWQnR0M2GzI8LSJ08sT4oP8sQTwFoMcUxi5IORD+xbwsgHeZZOAF8ABR2WKXlieVJ8YHnW1lqIjm4mbHZEWPrkieXJyAd54glgLYY4JjHywcgH9i1h5IM8SyeAL4CCDsuUPLE8KT6wPGtrLURHNxM2OyIsffLE8mTkgzzxBLAWQxyTGPlg5AP7ljDyQZ6lE8AXQEGHZUqeWJ4UH1ietbUWoqObCZsdEZY+eWJ5MvJBnngCWIshjkmMfDDygX1LGPkgz9IJ4AugoMMyJU8sT4oPLM\/aWgvR0c2EzY4IS588sTwZ+SBPPAGsxRDHJEY+GPnAviWMfJBn6QTwBVDQYZmSJ5YnxQeWZ22thejoZsJmR4SlT55Ynox8kCeeANZiiGMSIx+MfGDfEkY+yLN0AvgCKOiwTMkTy5PiA8uzttZCdHQzYbMjwtInTyxPRj7IE08AazHEMYmRD0Y+sG8JIx\/kWToBfAEUdFim5InlSfGB5VlbayE6upmw2RFh6ZMnlicjH+SJJ4C1GOKYxMgHIx\/Yt4SRD\/IsnQC+AAo6LFPyxPKk+MDyrK21EB3dTNjsiLD0yRPLk5EP8sQTwFoMcUxi5IORD+xbwsgHeZZOAF8ABR2WKXlieVJ8YHnW1lqIjm4mbHZEWPrkieXJyAd54glgLYY4JjHywcgH9i1h5IM8SyeAL4CCDsuUPLE8KT6wPGtrLURHNxM2OyIsffLE8mTkgzzxBLAWQxyTGPlg5AP7ljDyQZ6lE8AXQEGHZUqeWJ4UH1ietbUWoqObCZsdEZY+eWJ5MvJBnngCWIshjkmMfDDygX1LGPkgz9IJ4AugoMMyJU8sT4oPLM\/aWgvR0c2EzY4IS588sTwZ+SBPPAGsxRDHJEY+GPnAviWMfJBn6QTwBVDQYZmSJ5YnxQeWZ22thejoZsJmR4SlT55Ynox8kCeeANZiiGMSIx+MfGDfEkY+yLN0AvgCKOiwTMkTy5PiA8uzttZCdHQzYbMjwtInTyxPRj7IE08AazHEMYmRD0Y+sG8JIx\/kWToBfAEUdFim5InlSfGB5VlbayE6upmw2RFh6ZMnlicjH+SJJ4C1GOKYxMgHIx\/Yt4SRD\/IsnQC+AAo6LFPyxPKk+MDyrK21EB3dTNjsiLD0yRPLk5EP8sQTwFoMcUxi5IORD+xbwsgHeZZOAF8ABR2WKXlieVJ8YHnW1lqIjm4mbHZEWPrkieXJyAd54glgLYY4JpUS+Th06JCaPXu2OnjwoLcHpk6dqoaHh1VHR4d3XlSGEB2NYpPHDgfLPNSS85AnlifFB3niCWAthjgmlSo+li1bpnp7e529sG3bNrV8+XKKD2dirZGQgyXWT+SJ5UnxQZ54AliLFB+OPHXkg+LDEVjgyThYYh1MnlieFB\/kiSeAtUjxgeVZW2shOrqZsDlYYumTJ5YnxQd54glgLYY4JpUy7YLFXr21EB1dPcWzJXKwxNInTyxPig\/yxBPAWgxxTCpFfJw4cULJP81cNFrE9SE6ugiPonk5WBYlODY\/eWJ5UnyQJ54AzuLp00o98fQ+9elZvWrHjh1KNmWE8ClFfJi7Xeqwe8XXURQfvsTS03OwJE8sAbw1tlEsU\/LE8bzz4WfUw08eUpO3zqf4cMW6du1atWrVqkby7u5uNTQ0pNra2lxNNCUdxQcWOzsi8sQSwFtjG8UyJU8cT4qPgiwXL16stm7d2rAya9YsNTg4WNBqOdkpPrBc2RGRJ5YA3hrbKJYpeeJ4fvzBPWrn\/tcY+UAgtYXIkiVL1MDAAMI0xAbFBwRjwwg7IvLEEsBbYxvFMiVPHE+KDxzLMZZkambLli1NP1jMrBTFB9bZ7IjIE0sAb41tFMuUPHE8KT5wLBUjH0CYLWCKHRHWSeSJ5SnWyBTLlDxxPCk+CrJspcWnjHwUdLaVnR0ReWIJ4K2xjWKZkieO5zXLn1D\/+OrrXPPhg9QWHK2y7Zbiw8fL2WnZEWUz8klBnj603NKSqRsn11Tk6UoqOx3FRzajRgrznI\/29na1adMm1dXV5WGh3KRygd3SpUsT60XxgeXPjog8sQTw1thGsUzJE8eT4sODpYiPJ554Qn3qU5\/yyFVNUi2Mjh49SvFRDXLOp4M5s2MHA+WaDzhQtlEc0ksWfzsyxkPGHJjW+VZbvdg1LSLDyIeDkz2SsCPygOWQlDwdIHkmIVNPYBnJyRPDU9Z6SOSD4sORZ13Fh0y3LF++XC1YsEDdf\/\/9jHw4+rNoMnZERQmOzU+eWJ5ijUyxTMkTw5Piw5OjuebDM2t0ac7w8DD8Ujpdpzlz5qjp06dzzYevYwqkZ0dUAF5MVvLE8qT4IE88AYzF7+x7TX1i7R5GPjA4m2NFdt7IjYByt8zjjz\/uJD42b9485gbBVr2ltznEz5bKwRLrAfLE8qT4IE88gWIW5ceyfORY9c98\/QjFRzGczcu9d+9etWjRIrVmzZpo143rbhe7xn19fWrevHnNe5AWLfnUqVPqyJEjUTRr0qRJLfoU9ak2eeJ9QaZYpuRZjOfGjRujZQGvv++T6vX3fYLioxjO5uQ+ceJEtMajp6encX+Mq\/hYuXKlmjJlSqPiMngy+uHvx5MnT6rDhw9HUSSKD39+dg7yLM6QTPEMTYtso8X4SuRD\/hl45Ig68PqZW+C526UY08pzS9RDIhbHjx+PLTvuUjvudsG6idME5IklgLfGNoplSp4YnnqbLcUHhmfTrbhGPmSNiPxa56cYAXZExfjZuckTy1OskSmWKXkW52nudKH4KM6zFhYoPqp1AzsiLG\/yxPKk+CBPPIHiFu977Dl132MHIkPn\/vPL6sJt90SbJkL5QXzO6dOnTxfH1FoWKD6q9RcHSyxv8sTypPggTzyB4hb1bbZiadLLP1Jt3\/kfFB\/FsdbbAtd8YP3DwZI8sQTw1thGsUzJsxhPe8rlD371lHrwvwxQfOTBah481t3drR544AF19913j9mFksduGXkoPrBU2RGRJ5YA3hrbKJYpeRbjaUY93nnJ+Wrdr5+j5s6dS\/Hhi9Wc5ti5c2fjsK\/9+\/dHO1EWLlzY2Abra7uM9BQfWKrsiMgTSwBvjW0Uy5Q88\/M0TzUVK\/fc8i714YsOU3z4IrXP2TBPGm1ra1P2\/\/vaLyM9xQeWKjsi8sQSwFtjG8UyJc98PGW6RY5Tl3\/LR6IeTy27XoU4JpW+4NS8U2VgYGCc2BDxsWXLllLuc8nnfhWko\/OyQORjR4SgeNYGeWJ5ijUyxTIlT3+eIjjufPiZ6Eh1\/Xlk4Fp10\/TJQY5JpYsPHfno7OxUg4OD48SHXHE\/Ojoa3bkikZA6fEJUmc3kyo4IS588sTwpPsgTT8DPogiPPS8cU\/9x4w8aGUV0iPiQT4hjUuniQ8DJmo\/+\/n61bt06tW\/fvsaaDzm7ftWqVdHfe3t7\/bxVYuoQHV0irkzTHCwzEXklIE8vXE6JydQJk3Mi8nRGFU2xPPzkS40zPSSnTLeI8JB\/U3y4s4xNae520Qna29ujy3Pksrc6fSg+sN5gR0SeWAJ4a2yjWKbk6cbTXuMRJzwoPtxYBpGK4gPrRnZE5IklgLfGNoplSp7ZPB9+8lC0xsP82BEP\/V2IY1Il0y7ZbqhXihAd3UzC7Iiw9MkTy1OskSmWKXmm85QdLbKl1kV4MPKBbZu1tkbxgXUPOyLyxBLAW2MbxTIlz\/E8ZYrlL54+ov7g6\/vGfflQ3\/vVrGsuS3RCiGNS6ZGPuLUeSYTrsvA0REdjuxY\/a+yI\/HhlpSbPLEL+35OpP7O0HOR5lo6IjtV\/9Y\/qi4+\/OA6ZTLPcc8sV6rYPdqQ6IMQxqXTxIUT1bpclS5aMOclUzvjQu10uv\/zy2px2GqKjsV2LnzV2RH68slKTZxYh\/+\/J1J8ZxUc6M5lWkZtpzXM7dA4RHWvmXBWd4eHyCXFMKl182Od82KDNcz5k54tcGdzsMz9CdLRLAy8rDTt2LFnyxPIUa2SKZTqRef7xXx9Uv\/\/n\/xALVETH7dd3qluvvbyxjdaFfIhjUuniwz7h1AZtnnD6ta99rRannYboaJcGXlaaidwRlcGUPPFUyRTLdKLwlCkVERR6LYc+Ft2mKWm+cscvq\/d35jtIM8QxqXTx4Rv5qMNR6yE6Gtu1+FmbKB2RH5X8qckzP7uknGSKZRo6T5lSefLAUfWH33o2FZzv9EqSsRDHpNLFh8B0WfNxww03qAULFih9DDv2VfCzFqKj\/QhgU4feEWFpZVsjz2xGvinI1JdYevoQeaat4TBpiOD4\/Y9eoW5492SvqZU0oiGOSZWID4Eat+tl6tSp0YVycqeLCA\/5NHu9h9QhREdjuxY\/ayF2RH4EsKnJE8tTrJEplmkIPOMuekuiJIJj3oc61W\/\/it9aDlfqIY5JlYkPV8h1SBeio5vJNYSOqJn87LLJE+8NMsUybSWeep3Gc6+cVKu2HYjdnRJHRwTHfbe+V13V8VZYhCPJCyGOSRQfMd4O0dHYrsXPWit1RH5P1pzU5InnTqZYpnXlqReIyhTKN\/72iPq7l054iY0br5wcncshH33pG5ZcvLUQx6RKxEfWQWN6+qWjI\/2glSqczGkXPOW6dkT4J63GInniOZMplmkdeJpC48\/+5rDad+SfnYWGFhciMP5z7xWR0KhSbNjeoPjI2T7tszz27dunBgcH1d69e9WiRYvUmjVranWzbYiOzuk6SLY6dESQB6mJEfLEO4JMsUyr5qmnTvT19HKwV9K216QnFXFx8y9eon77zTM4mik2KD4A7dE+50N2vjz00EONhaVyzocWI4DiICYoPiAYG0aq7oiwta+fNfLE+4RMsUzL5inC4isjL6nH97\/mFc3QTynCYtrF56u7Z\/6COm\/SuU2PbGTRD3FMKn3axRYfEu2499571YYNG5RMs9j\/n+WEKr4P0dFVcEsqo+yOqJnP1oyyyRNPnUyxTBE8deRi61M\/Vv\/7mVdyiQw9fSJCY\/B3flEdPvYvzkeaY4kUsxbimFS6+NCHjPX09ET3uogYmT9\/vlqxYkU01ULxUaxRtkJuREfUCs9ZVR3JE0+aTLFMfXiKyHj+lZPqf+05rJ49cjK3yNBCQ6Ia\/25Gh\/q16RdHD1Wn6ZO8lCk+cpIzj1CXaIesAZk+fXokRuS7OtznYj5aiI7O6TpINp+OCFJg4EbIE+9gMsUytXnK7hIRAbLw869+9GohgWFGM+Rcjelvv6Aloxk+xEMck0qPfGjA5qJTiYbMnj1bHTx4UNVtp4vUN0RH+zR0dFp27Fii5InlKdbItDhTPU3y5V2j6rvPHlXPHTmhRo\/9rJBhvTbjt65+u3r\/O9pqvzaj0MOmZA5xTKpMfJTllDLshujoMji52mTH7krKLR15unHySUWm2bS0uNh\/5J\/VI3uPeG9dTStBiww5tOvVf\/rphBUZSYxCHJMoPmK8HaKjs7uW8lKwY8eyJU8sT0Y+zvA0t6vqA7he+Mnr3ltWk7yjBcZvfuBt6uop7dFUiT6LA+\/RsCyGOCaVLj7sBaZ2k7DXg9ShyYTo6GZy5WCJpU+eWJ4TRXyY4kK2qP71vp8opLgQjlpgdE25QP3mFeeqn553sXr3ZWemS\/jJTyDEMYnig5GP\/G+EY04Olo6gHJORpyMoj2QhMDXFxfYfvqJ2P3+sFHEhWOWY8dtvmKJO\/fSN2CmSEHh6NJ\/Sk1J8OCLW22tHRkaccixZsiTa+VKXT4iObiZbdkRY+uSJ5dkqkQ+9Y2THP\/xE\/en3Dqk3TqvCu0biSOroxcd++e3qA51tURKZIvH5sI360MpOG+KY1PTIRzb26lOE6OjqKZ4tkR0Rlj55Ynk2W3yYEYujr\/9MPfaDl9VzL5+ERy00NS0u5Cjx7nddlEtcZHmAbTSLkN\/3IY5JpYsPP8T1SB2io5tJlh0Rlj55YnmWKT5EWMg9IzJN8bcHj6u\/\/MHL0SJL9FoLU1jIf8uJnnO73xH9O0\/koihhttGiBMfmD3FMoviIaSMhOhr7KvhZY0fkxysrNXlmEfL\/Pi9TPRXyg9ET6ptPHylVWMhT6ajFL1x6vvrUNZdH95I0Q1xkEc7LM8vuRP0+xDGpFPGh73ORQ8RcPnU7aCxER7v4oaw07IiwZMkTy9OMfJx+69uj3RlaVPzrG6fVFx9\/Ue154XhUqEQxyvroHSESJfnIL16i3nHheVFR+u+ttGOEbRTbSkIck0oRH1js1VsL0dHVUzxbIjsiLH3yzM\/TXF\/x5IGj0VHf8ilrGkTXVAuHm\/7\/9Mut116u3vJmxKIVhYULfbZRF0ruaUIckyg+YvwfoqPdmzk+JTsiLFPyjOcp0Qr5\/OjwP6m\/+cdjpU+BmFEJWVvxnssuUO\/reKv66Pvf1jg8q5WiFchWyjaKpBnmlR8UHxQf2Lckxho7IiziicJTn36pp0C+s+8nasuTh9TpCiIVtrCwt53qCMpEFRdZLXqitNEsDqjvQ\/xBXJn42Lt3r+rr61PHj5+ZO5VPe3u72rRpk+rq6kL5CGInREdDwOQ0wo4oJ7iEbCHwNNdUfHnXS2r380fV6dPlT3\/YokIiFb9x1aXq3Df+Rb322mvqV947TU2aNIknchZssiG00YIIoNlDHJMqER\/btm1T\/f39yj5MTI5WX7VqlVq3bp3q7e2FOquIsRAdXYRH0bzsiIoSHJu\/jjztKMXzr5xUf7bnx0ouISt7PYWmo6MQMgXy3ssvUNMve6v62AfcpkDqyBTbaqq1Rp5Y3iGOSaWLD33aaWdnpxocHBznkcWLF6vR0VE1NDSk2trOnKbX7E+Ijm4mU3ZEWPpV89RRip\/+6xvqT3a9pL73\/LHKpj7sSMW7Lv159W8\/8DbVfv6kCCrqcrKqmWJbRP2skSfWJyGOSaWLD73tds6cObFHqPNiOWwjraM1dkRYrxTlae74mHbJ+erbP3pVfW3P4cqmPeIiFTN+4UL1Hz7UqSade86Ym06rWlNRlCnWw61vjTyxPqT4yMGTkY8c0ALLwo4I69A4nqagkAF75MDRSFSUeZpm3FOZUx\/y372\/dKm69K1vGROlMKMZWDL5rbGN5mcXl5M8sTwpPnLy5JqPnOACycaOKJ8jRTjotRTy7++PHldPv3gi+ttzR06o0WM\/y2c4Ry5TVEiU4sPvvVi9+20XNCVKkaP6mVnYRjMReSUgTy9cmYkpPjIRJSfgbpcC8Fo8KzuisQ7Uayief\/WkeuqF42rb370SJahqYaaujSkoZC3F7Td0qpP\/8kYUa5FPAAAgAElEQVTtoxRlvA5so1iq5InlSfGB5VlbayE6upmwJ0JHpA+4uvD8f6M2P3lIyV0fzRYUso30uisuUh0Xnjdu62hVayma2e58yp4IbdSHR9G05FmU4Nj8IY5JpS84xbqgGmshOroacvGltEpHZE5x6CeRv\/39j\/8p2uFR9fqJuAiFnKI5bfLPqe7L\/lX99LyLo3tIKCSKt+5WaaPFn7QaC+SJ5RzimFS6+NC7XWSrbZ2206Y1jRAdjX0V\/KzVoSPSCzKl5nL89pMHjqknnj1zHHfV0x1Spjnlcc20dnX9uyerC9\/cPpp130cdePq1gPqnJlOsj8gTyzPEMaky8WHfcFu3g8XMphKio7Gvgp+1Mjoie3fH7uePRbs7DrxysuliQujILo9fntIebR01IxOIKEUZPP08Gl5qMsX6lDyxPEMck0oXH7YL9M4X8+\/d3d21ioqE6Gjsq+BnzbUjMgXFxRdMUt98+mUl93nUITox\/bIL1K9Ma1cffu8lUX30FI0ZxfCjkj+1K8\/8JUy8nGSK9Tl5YnmGOCZVLj5Ml+gzQOSE0+HhYdXR0eHkMTkVdevWrVFa1\/thzDy6kCTRE6KjncCWkEgG6Wd\/fEL93KmfqKPnXKgef+642nvwzP0+VU93mFEHOYJbtozKIVu\/\/r5LW+oWUnbs+IZKplim5InlGeKYVLn40Pe5mK6x73xJc5t9Iqr8\/\/r161MvqNMip6enJ\/aUVbu8EB2NfRXO\/PLXEYBzz1VqePfhSu\/x0M9jH2r18avfrtrOO3P0Nnq6A80wrz127HnJJecjUyxT8sTyDHFMKl186AWn5poP12iF7b64o9pdhIXOt2zZMqcL7EJ0tO+roMWFrKH41vdfjraOVhWpMAWFLMT80BUXqbdMOpfbRd90Ijt239acnZ5Msxn5pCBPH1rZaUMckyoTHzNmzIi9WC4be3qKrLtjJLcccHbvvfeqDRs2OE3thOjoJIpyPoUcdvXEs0eV3ES6c\/+ZHSDoT+eFk6KrymW649fec7HqvOg81fOei1tqugPNJK89dux5yTHygScXb5FtFEs6xDGpdPGBdcF4ay4X08VN9aQtcg3R0UJOhIZEMr66+xBMZEiUQgSF\/PvGKyerd17y85GTzO2i7IiwbwF5YnmKNTLFMiVPLM8Qx6SWFR\/mrpmsNSOy2HT79u2NdSF6qkaaR9zZI6E4WsTG8PcORdeg5\/mY0x8DH5425hpzH3vsiHxoZaclz2xGvinI1JdYenryxPIMZUwyqbSs+NAPkXVrblIT0HfNrFy5ctw6EO3ozZs3q6lTpzZMuO7GwTY7P2uP\/O3L6otPvOQV2ZApEYla3HrtZeo9l711TOTCr3SGYBG8smywY88i5P89mfozS8tBnsV4ynIC8yNrJufOnat27NgxZkwqVkpzc7e8+BB8EgVZunRp6o4XG3PaWhEtPuw8fX19at68ec31WEzpu144qZb\/1SuZt5yKyJDPp6+9UE2\/9C3qHe2TlP5bmQ916tQpdeTIkWi9jaz74KcYAfIsxi8uN5limZJnMZ4bN26MxjP7Q\/FRjCs8d1niQ6IiU6ZMadRXBs+6RD9kN8qdDz+jvvfimS2vcR+9HuP+T10RDfpVCI24epw8eVIdPnw4UuwUH8WbP3kWZ2hbIFMsU\/IsxlN+HJvRj127dqnVq1cz8lEMa\/7cSVMlaYtOk6ZlxNaiRYvUmjVrVFdX15hK1Xl+TdZx3PfYc4nTKiI4Ftw4Vcl9ITdNn5wfNjAnQ7BAmFwciYX5pjW2USxW8sTyrPOYlPdJS592EfU2f\/58tWLFinGDvFTaZbeK+XCyeHT37t2NE1G1IFm4cGHiAWJ6caq+T0ZPuSRt\/62jo3WkI24rrAiOW37pUnXnR95ZyxtO2RHlfT3j85EnlqdYI1MsU\/LE8qzjmFT0CVtOfMgD20el25fUyffyGRwcbPCx75SZNWtW4rkjdXO0RDs+sXbPOF+L6Fgz56raRDiSGiM7oqKv6dj85InlSfFBnngCWIt1G5MQT1eK+NBTHSMjI051zNoq62QEmKgujpZoh4gO8zp4ecxWER3aJRwsgY2Tv9KxMN+0xjaKxUqeWJ51GZOQT1WK+DArmDXtgnwYlK06ODou2tFqooPiA9UiGfkoh+RZqxwssYTJE8uzDmMS9omUKl18oCtchb1mO3rzyEtq0ZYfjnlUER6PDFxbyzUdWT5hR5RFyO978vTj5ZKaTF0ouachT3dWLimbPSa51NE3DcVHDLFmOVqmV7759BG17Ov7xtTq6wPXqF+bfrGvb2uTnh0R1hXkieUp1sgUy5Q8sTybNSZhn2KstVLEh95N0tnZGe1yuf3225V5q639QHL+w\/DwcG3O0GiGo0V4yFHoK\/7yuQaeVp1msf3Ljgj7CpMnlifFB3niCWAtNmNMwj7BeGuliI+yK122\/WY4eu3\/fWFMxKOVp1koPsptoRQfeL5kimVKnliezRiTsE9A8eHEs2pHy8Vvn\/3q2TUeIQkP\/qp0anJeidixe+FySkymTpicE5GnMyqnhFWPSU6VKpiIkY8YgFU6+uEnD0XHpOtPaMKD4qPgGxqTnR07meIJYC2yjWJ5VjkmYWuebK108aHXf3DNx3gnyDqPa5Y\/EbTwoPjAv8rs2MkUTwBrkW0Uy5PiA8szuo12+fLltVpsKo9YhaPtA8RCWVwa10TYEWFfHPLE8qRAJk88AazFKsYkbI2zrZUe+ciqQtxR6Fl5yv6+Ckff87W\/V\/\/zOy82HuXB265St32wo+xHa4p9DpZY7OSJ5UnxQZ54AliLVYxJ2BpnW2u6+PC9WC77kYqnKNvR9umlcvusHCAW6oeDJdaz5InlSfFBnngCWItlj0nY2rpZo\/iI4VSmo+OmW1r15FK3JsYDnFw5uaaj+HAl5Z6OTN1ZuaQkTxdK7mnKHJPca4FN2VTxkXW1PfZR3a2V6Wh7W23I0y2aODsi97bnkpI8XSj5pSFTP15Zqckzi5Df92WOSX41waUuXXxk7Xbp7u5WQ0NDqq2tDfdUBS2V5Wh7d8uNV05W37gz3OkWio+CDTEhOzt2PFcyxTIlTyzPssYkbC39rJUuPnR1ZG3Hjh07GkJDdrosXbpUbdq0SXV1dfnVuuTUZTn6E2v3KFnvIZ+Qd7fY7mFHhG2w5InlKdbIFMuUPLE8yxqTsLX0s1aJ+BDhsX79+nFCQwRIf3+\/Wrdunert7fWreYmpy3C0HfVY\/+9\/Sf3Or15e4lPUxzQ7IqwvyBPLk+KDPPEEsBbLGJOwNfS3Vrr4OHHihFqwYIGSS+YGBwfH1VC22o6OjtZq6qUMR3\/8wT1q5\/6zUY+nll3v760WzcHBEus48sTypPggTzwBrMUyxiRsDf2tlS4+9JqPOXPmqIGBgXE1nAhbbe2ttffc8i51zy1X+HurRXNwsMQ6jjyxPCk+yBNPAGuR4iMHT4oPpX7nj\/eq7T98NaInaz0mUtSDHXuOlyYjC8UHmeIJYC2yjWJ5Unzk5Jk0tZI1JZOzuMLZkI6213ps\/cw1quc9FxeuYysZYEeE9RZ5YnlSIJMnngDWInJMwtYsv7XSp12kanv37lV9fX1q5syZY9Z9JC1Ezf84mJxIR8uNtXJzrY56hH6gWJwHOFhi2qW2Qp5YnhQf5IkngLWIHJOwNctvrRLxIdXTUY6RkZFGbadOnVq7S+WkcihH21GPu25+p\/qvH78yv7daNCcHS6zjyBPLk+KDPPEEsBZRYxK2VsWsVSY+ilWz2twoR0vEQyIf+iNRD7nHZaJ9OFhiPU6eWJ4UH+SJJ4C1iBqTsLUqZq108aEjHnfccUetzvJIw4ZytLm9dqKcZhrHlYNlsZfUzk2eWJ4UH+SJJ4C1iBqTsLUqZq108ZG126VY9cvJjXC0vb12okY92LHj2yjFB5niCWAtso1ieSLGJGyNilsrXXzoyEdPT0\/sOR\/FHwFvAeHo\/\/YXz6oHtj8fVW4ibq81vcKOCNtGyRPLkwKZPPEEsBYRYxK2RsWtlS4+pIqy22XRokVqzZo1tbvHJQ5hUUfbC03v\/Mg09YefmF7cWy1qgYMl1nHkieVJ8UGeeAJYi0XHJGxtMNZKFx9Zt9rKY9Rt10tRR9viQw4Vk+jHRP1wsMR6njyxPCk+yBNPAGux6JiErQ3GWuniA1PNaq0UdfREP9HU9hYHS2z7JU8sT4oP8sQTwFosOiZha4OxRvERw7GIo+2ox4O3XaVu+2AHxlstaoWDJdZx5InlSfFBnngCWItFxiRsTXDWKD7A4oO7XMYD5WCJe2E5UGJZamtso1iu5InlSfHhyFOv8+js7FQrVqxQt99+uzp48GBi7pDWfHz6oafVt77\/cvSsE32XCzt2xxfGMxk7dk9gDsnJ1AGSRxLy9IDlkJTiwwFSCEmKOPqSxd9uIPjT371azXzfpSEgKfQM7IgK4RuXmTyxPBlNIk88AazFImMStiY4a5VNu8glcjt27FBDQ0Oqra1Nbdu2TS1dulRt2rSpdttv8zqaUy7xDZODJe6F5UCJZcnoHHmWQwBrNe+YhK0F1lol4iPp9loRIP39\/WrdunW1Ono9r6PNu1w45XK2oVJ8YF9a8sTypKAjTzwBrMW8YxK2FlhrpYsPfcKprP8YHBwcV\/vFixer0dHRRkQE+3j5rOV1tHmXyye73q6+OO8D+SoQWC4OlliHkieWJ8UHeeIJYC3mHZOwtcBaK118ZN3tIlGRLVu2qOHhYdXRUY8tqXkczYPFkhsmB0vsS0ueWJ4UH+SJJ4C1mGdMwtYAb43iI4ZpHkfb6z1eHbwZ760WtcjBEus48sTypPggTzwBrMU8YxK2BnhrpYsPqXLS1ErWlAz+cd0s5nG0earpjVdOVt+481q3wiZAKg6WWCeTJ5YnxQd54glgLeYZk7A1wFurRHzIxXJ9fX1q5syZY9Z9JC1ExT+mn8U8jr5m+RNKpl7k8+f916gPv\/div0IDTs3BEutc8sTypPggTzwBrMU8YxK2BnhrlYgPqbaOcoyMjDSeom6Hi+mK+TraXu\/xyMC16qbpk\/HealGLHCyxjiNPLE+KD\/LEE8Ba9B2TsKWXY60y8VFO9cux6uvo+x57Tt332IGoMtxiO94nHCyx7ZQ8sTwpPsgTTwBr0XdMwpZejjWKjxiuvo6e\/cd71f\/54auRJa73oPgo51U9a5XiA0+YTLFMyRPL03dMwpZejjWKD4D4MI9Uv+eWd6l7brmiHG+1qFV2RFjHkSeWJyMf5IkngLVI8YHlWVtrPo7m+R7ZbuRgmc3IJwV5+tByS0umbpxcU5GnKym3dD5jkpvF5qdi5KNg5MMUH1zvEd+g2RFhX3TyxPJk5IM88QSwFik+sDxra83H0eaR6lzvQfFRRaOm+MBTJlMsU\/LE8vQZk7All2eNkY+CkQ9TfHC9B8VHea\/qWcvs2PGUyRTLlDyxPCk+cvKMO+PDNFW38z5cHc3zPdwaBDsiN06uqcjTlZR7OjJ1Z+WSkjxdKLmncR2T3C02P2UlkQ85Xn337t21ujwuDb2ro+37XHi4GCMfVbzS7NjxlMkUy5Q8sTxdxyRsqeVaK118ZN1qW+7j5bPu6mgeLubGlx2RGyfXVOTpSso9HZm6s3JJSZ4ulNzTuI5J7habn3LCiI9t27ap\/v7+BvElS5aogYGBWA+4OpqLTd0aMDsiN06uqcjTlZR7OjJ1Z+WSkjxdKLmncR2T3C02P2Xp4qMON9fqi+1Wrlypent7lf7\/hQsXxgoQV0ebl8lxsWlyY2ZHhH3RyRPLU6yRKZYpeWJ5uo5J2FLLtVa6+JDq66jDunXrosG\/6o+sORkdHVVDQ0Oqra0tKl5u1N2xY8eYv+l6uTiai03dvciOyJ2VS0rydKHkl4ZM\/XhlpSbPLEJ+37uMSX4Wm5+6dPGh13wcPHgw8WmbsdslTpD4iA97selTy66PLpXjZzwBdkTYVkGeWJ6MfJAnngDWIsUHlmfTrNnTMHZFXBy94x9+omZ94akoK082TXclB0tsUydPLE+KD\/LEE8BadBmTsCWWb630yEf5j+BeghmF6e7ujp1yEWva0Zs3b1YSldGfjo6Oxn9\/av3Tauf+16L\/v+nd7epr\/de4V2SCpeRgiXU4eWJ5UnyQJ55AMYsyVpkfmTmYO3dutFTAHJOKldLc3BNKfJioZc3Hli1bYs8e0eLDdk1fX5+aN29e9OePbzyoRo\/9LPrv3+2erBZeN7m5nqxx6adOnVJHjhxRIt4mTZpU45q2RtXIE+8nMsUyJc9iPDdu3Kg2bdo0zgjFRw6udtThgQceUHfffbfq6elJ3PKaoxjnLGnnj2jxIbtjpkyZ0rApg6f8I4tNP7RqT+Pv311yLdd7pJA\/efKkOnz4cKTYKT6cm2hiQvIsztC2QKZYpuRZjKeMT2b0Y9euXWr16tWMfPhild0uS5cujZTczp07G7tM9u\/frySakLTl1bccn\/Qu4iNJZXKxqQ9pbmP0o5WdmtMu2Yx8U5CpL7H09OSJ5ck1Hzl46nM+dITD3uKatuU1R3HjsiSdM5K26DTL0TzZ1M8z7Ij8eGWlJs8sQv7fk6k\/s7Qc5InlmTUmYUurxlrpaz7sCEOc+Ehae4FCYJ8zogWJ2DfP\/tDlZTmaJ5v6eYYdkR+vrNTkmUXI\/3sy9WdG8YFllmYta0yqria4kkoXH3bkwRYfaedt4B7z7EFn2uasWbPU4OBgbBFZjjbFB082zfYSO\/ZsRj4pyNOHlltaMnXj5JqKPF1JuaXLGpPcrNQrVeniQx7XjDzs27evseZD1oCsWrVKNevk0yRXpDnaPtn0wduuUrd98OwW3Hq5tx61YUeE9QN5YnmKNTLFMiVPLE+KjwI84046bW9vjxahdnV1FbCMz5rl6EsWf7tR6CMD16qbpnObLUOw+HaYZJEdO541mWKZkieWZ9aYhC2tGmuVRD6qeRRcKWmOtne6vDp4M67gQC2xI8I6ljyxPBn5IE88AaxFig8sz9paS3M0d7r4u42DpT8zRpKwzLKssY1mEfL7njz9eGWlpvjIIhTI92mO\/uTaPeqv9505Vv3GKyerb9x5bSBPXd5jsCPCsiVPLE9GPsgTTwBrkeKjAM+4NR9p96sUKKpw1jRHc6eLP14Olv7MGPnAMsuyxjaaRcjve\/L045WVmuIji1DC9+YJp+biUtl2u379+totOk1ztLnYlDtd3BoEOyI3Tq6pyNOVlHs6MnVn5ZKSPF0ouaeh+HBn1Uipz\/m44447VG9v7zgLVZ3z4VP1JEfb22yfWnY973RxAMuOyAGSRxLy9IDlmJRMHUE5JiNPR1COySg+HEGZyWS6Zf78+WrFihWxW2rTbpfNURwkS5KjeadLPrzsiPJxS8pFnlieYo1MsUzJE8uT4iMHT\/tuF9tEK0U+TPHxzkvOVxL54CebADuibEY+KcjTh5ZbWjJ14+SaijxdSbmlo\/hw4zQulX23ik4gUY9WOuH0zoefUQ8\/eSiqPne6uDcGdkTurFxSkqcLJb80ZOrHKys1eWYR8vue4sOPV5Q6bpdLmpmpU6eq4eFh1dHRvCPLkxxt7nSRI9VlwSk\/2QTYEWUz8klBnj603NKSqRsn11Tk6UrKLR3Fhxunlk+V5Ohrlj+hZNGpfHihnLub2RG5s3JJSZ4ulPzSkKkfr6zU5JlFyO97ig8\/Xi2bOs7R9k4X3uni7l52RO6sXFKSpwslvzRk6scrKzV5ZhHy+57iw4\/XmNR79+5VfX196vjx442\/t9LFchQf+Z3Pjig\/u7ic5InlKdbIFMuUPLE8KT5y8tQLTpcsWaIGBgYaVlppwSkvlMvpfHbs+cEl5GTHDkdK8QFGyjaKBUrxkYOn3mrb2dmpBgcHx1lola22vFAuh\/PfzMKOKD87Rj6w7JKssY1iOZMnlifFRw6eerfLnDlzxkQ9tKlWOWSM22xzOJ\/iIz+0lJzs2PFYyRTLlDyxPCk+cvAMJfJhbrMd+PA0tfyT03PQmJhZ2BFh\/U6eWJ5ijUyxTMkTy5PiIyfPENZ8cJttTuezY88PLiEnO3Y4UooPMFK2USxQio8CPFt5twt3uhRwPMVHMXgxudmxw5FSfICRso1igVJ8YHnW1prtaIqPYq5iR1SMn52bPLE8Oe1CnngCWIsUH1ietbVmO5rbbIu5ioNlMX4UH1h+cdbYRrGMyRPLk+IDy7O21mxHc5ttMVexIyrGj+IDy4\/igzzLJ4AtgeIDy7O21mxHf+v7L6tPP\/R0VF\/eZuvvNooPf2ZpOcgTy5PTLuSJJ4C1SPGB5Vlba7ajzW22v3HVpeqr\/+nq2ta9jhXjYIn1CnlieVJ8kCeeANYixQeWZ22tpYkP3mbr7zYOlv7MGPnAMsuyxjaaRcjve\/L045WVmuIji1Ag39uOvmTxtxtPRvHh72R2RP7MKD6wzLKssY1mEfL7njz9eGWlpvjIIhTI96aj37jgbUoOGNOfp5Zdr955yfmBPGk1j8GOCMuZPLE8xRqZYpmSJ5YnxQeWZ22tpYmPRwauVTdNn1zbutexYuyIsF4hTyxPig\/yxBPAWqT4wPKsrTXT0Qdeb1OfWLunUddXB2+ubb3rWjEOlljPkCeWJ8UHeeIJYC1SfGB51tZakviQ6RaZduHHjwAHSz9eWanJM4uQ\/\/dk6s8sLQd5YnlSfGB51taa6eiv\/OCn6r7HDkR1pfjI5zJ2RPm4JeUiTyxPRj7IE08Aa5HiA8uzttZMR3\/m60fUzv2vRXXlAWP5XMbBMh83ig8sN\/5SJ8\/qCGBLovjA8qyttSTxwW22+VxG8ZGPG8UHlhvFB3lWRwBbEsUHlmdtrZmO\/q0vvaDkVlv5cKdLPpdRfOTjRvGB5UbxQZ7VEcCWRPGB5Vlba9rRf7J1mxLxoT8UH\/lcRvGRjxvFB5YbxQd5VkcAWxLFB5Znba1RfGBdQ\/FBnlgCeGtso1im5InlSfGB5Vlba0nig6eb5nMZO6J83Bj5wHJj5IM8qyOALYniA8uztta0o\/\/7Fx9RsttFf3jAWD6XUXzk40bxgeVG8UGe1RHAlkTxgeVZW2va0R\/7g01KzvmQD8\/4yO8uio\/87OJykieWp1gjUyxT8sTypPjA8qyttTjxwTM+8ruLHVF+dhQfWHaMJpFnNQSwpVB8YHnW1pp29OQ5q5Xc7SIfio\/87qL4yM+O4gPLjuKDPKshgC2F4gPLs7bW4sQHDxjL7y6Kj\/zsKD6w7Cg+yLMaAthSKD6wPGtrTTv6WO996o0L3hbVk+Ijv7soPvKzo\/jAsqP4IM9qCGBLofjA8qytNe3o12ZtaNTxwduuUrd9sKO2da5zxSg+sN4hTyxPsUamWKbkieVJ8YHlWVtr4ug5Cz6rJPKhPzzdNL+72BHlZ8fIB5YdIx\/kWQ0BbCkUH1ietbUWF\/ngAWP53UXxkZ8dxQeWHcUHeVZDAFsKxQeWZ22txYkPHjCW310UH\/nZUXxg2VF8kGc1BLClUHxgedbWmi0+eMBYMVdRfBTjZ+cmTyxPsUamWKbkieVJ8YHlWVtrFB9Y17AjIk8sAbw1tlEsU\/LE8qT4wPLMZe3EiRNqwYIFamRkpJF\/3bp1qre3N9Xe4sWL1datW8ek6e7uVkNDQ6qt7cxBYvpjiw8eMJbLVY1M7IiK8WPkA8svzhrbKJYxeWJ5UnxgeXpb08JDMmrRsG3bNtXf36\/SBIjO19PTowYGBjLLtcWHbLGVrbb85CPAjigft6Rc5InlKdbIFMuUPLE8KT6wPL2t7d27Vy1atEitWbNGdXV1NfJLVGN0dDQ2iiGJDh06pGbPnq2WLVuWGSGR9Lb44AFj3q4ak4EdUTF+jHxg+THyQZ7lE8CWQPGB5QmztnbtWrVjx45E8SGi5d5771UbNmxQHR3ZB4VRfMBcExmi+CBPLAG8NbZRLFPyxPKk+MDyhFjTUyqdnZ1qcHAw1qaIk1WrVo35Lmm9R1zkgweMFXMVO6Ji\/Bj5wPJj5IM8yyeALYHiA8sTYs1lzYdMy2zfvl1t2rQpmq6JWztiVsaOfFB8FHMVxUcxfhQfWH4UH+RZPgFsCRQfWJ6Frcl0Sl9fn5o5c2Zi1COpEJ135cqV49aB2OLjm7dPU3LWh8uUTeGHCtAAxQfWqeSJ5SnWyBTLlDyL8ZR1iubn4MGDau7cudHygqlTpxYzXpPc55w+ffp0TeriVY0iwkMK0otQ58yZM24HjC0+Jm+dH9VNhM68efO86snESp06dUodOXIkEm+TJk0ikoIEyLMgwJjsZIplSp7FeG7cuDGK1Nsfio9iXAvnLio8fMSHRDzW\/fo5UZ1l8GT0w999Bw4cUPIyzZ8\/PxjV7k8Bl4M8cSy1JTLFMiXPYjzlx7EZ\/di1a5davXo1Ix\/FsBbL7Ss8khakJm3bldqZkQ8erV7MXybPkFR7cSr5LYQ4\/5ufBiYnmWI4aivkSZ5ZBFpq2kVPlcyYMcNrjYe9KDXLjik+eLppVhPK\/p4dUTYjnxTk6UPLLS2ZunFyTUWerqTc0oXIs6XER9yWWe06WYQzPDwcTYvI7hb5mFtvtQDR6WfNmpUoYEzx8a7zTyjZ7cJPfgJ6sdTmzZs57ZIfYyMneQIgWibIFMuUPMvhGVL0uKXEB9adydbkxVm6dKl67B2\/q87\/4SPq\/B9+vaqiWQ4JkAAJkAAJjCNw3XXXqYcffjgYMhQfCa4UASL\/8EMCJEACJEACzSYg0f1QttkKS4qPZrcolk8CJEACJEACE4wAxccEczgflwRIgARIgASaTYDio9keYPkkQAIkQAIkMMEIUHxMMIfzcUmABEiABEig2QQoPprtAZZPAiRAAiRAAhOMAMXHBHM4H5cESIAESIAEmk2A4sPygD6OfWRkJPqmu7tbDQ0Nqba2tmb7qtbl24e4rVu3btxtwfYD2IfGpR38VuuHL6FyeXia1ZD8claNXE7V1dVVQg1byyRFNWUAAAjDSURBVGQenvokZL3lnn3BWZ\/n4amvxjh+\/HhkiO+73zskh2dOnz593EWoflbqk5riw\/CFfQ9M0r0w9XFfPWpiD3QuA58Ij\/Xr1zcGx6wj7+vxpNXUIg9Ps2aa5dGjRyk+lFJ5eNrtUfcFwnmi\/xgpwlPfIs733a8v0T\/UlixZQvHhh641UscNmmkX0LXGU5VbyySBFnfEva6JztPT0zPmRXIRLeU+TfOt5+Fp11rYb926VbW3t0948ZGXp3T2cpS1KTTYFyhVhOeWLVsaV2BIm5X3ffny5WP+1vw3sF41sCPxFB\/18g+sNnEdTtJACSu0xQ3pXzDLli0bM82Sp2Nh566ia7Rnz56t8vLU3BcsWKDuv\/\/+CS8+8vDkO5\/cKeXhKdakb6X48OvsdTscHR1Vn\/\/859Vdd92ldOTIz1I9U3PaxfBL3K91Tr2kN9wkwZAnihHXQdXztSmvVkV46oFBOiiZG+aaD6Xy8DQH2EcffTSKIsmHaz7y8RR2ZtscGBho\/L\/vDeXlvXn1tmzzq3dt3WpH8UHx4dZSElLl6dzjTHEO+AyVIjzNyN3jjz9O8ZGTp7kwUi+c5pqP4u0z5CmEQp2oQ2aKDwdIrZyEkQ9\/7xUZLHVpZnhxeHhYdXR0+FckkBx5edr58kSeAkE45jHy8NTiY+HChWPWJOm\/r1y5MnMnV4gsi4hjvTuGYi5fy6D4yMetZXJRfPi7Kk\/nbpZC4TGWeR6ecWsUKD7y\/1JPEhkhDgC+b3yR9tnZ2akGBwcbRVLMudMPse1x2sXwPxecur8MOmXeBWiSn8JjPO88PO3zE2yrIa2Q922heXgm5QlxAKiCZ9ICXvJ0px8iK4oPw\/\/cauv+MthTJvavmrStthQeyZzzbmW0LTLycYZIXp5x7Ze\/1PPxTPJBksjz74XCz0HxEbiP7UVl8riyZdEeWAPH4P149nyuy8Annfvu3bu5xz+Gdh6eFB\/JzTYPT1to6M5f+gIeMrZN9ff3K71+w+V955oP7251TAaKj2L8WiI3j1fP56as45bNX5JZ0wQuR7Pnq2Xr5PLhGfdULgNC69AoXtM8PO3j1Xkc+Fk\/5OFpv\/fcuuzerik+3FkxJQmQAAmQAAmQAAnEEuCaDzYMEiABEiABEiCBSglQfFSKm4WRAAmQAAmQAAlQfLANkAAJkAAJkAAJVEqA4qNS3CyMBEiABEiABEiA4oNtgARIgARIgARIoFICFB+V4mZhJEACJEACJEACFB9sAyRAAiRAAiRAApUSoPioFDcLIwESIAESIAESoPhgGyABEAE5Hff3fu\/31KJFi1RXV5eXVTkx8tFHHx1z66eXgRyJ7dN8854sKxcybtmypaWPyjdP32xvb1ebNm3y9mEOF4zJYp6o2qw6FH0G5icBVwIUH66kmI4EMgjkPdI86eKtsoHnra9dr5DEx8qVK1Vvb2\/Z6FPto\/zS1Idg4SSQQYDig02EBEAE8g4azRIfKNGAsgNyQy4zdbqxNm87yvXgzEQCTSJA8dEk8Cy29QjYl2mZoXEZgFetWtV4KPMSsriL9JYsWaIGBgaUfXmZHW5Ps5tGMM2uPd0idtIu+bLTx9VRpl3kBujPfe5zjWrpZzTraT+PbUvX+2Mf+5javHmzOn78uNIs4y56E9ujo6NjbprNYhrHLU58aFFlP1fW9FSSmLRvdk3yH8VH6\/UNrLE\/AYoPf2bMMQEJxA0c9i\/+uEEjblBLul5crmsfHBxs0JWbgLdv395Yf6AH1RkzZqSuDdFlzpw5s5FOD\/rmwOkSsYi7Sl7yrV+\/vlEvbdsUMC685EHlGXfv3t1YL6LLO3r06Jh1F3HPJHm3bt06RjglPbtZXx\/xIYJy6tSpjfq5CAiKjwnYQfCRvQlQfHgjY4aJSMBloI4THzJA2r\/M7cEpbrDSg+jChQujCIn+uAx+aWWKnaGhIdXW1qZcnikuja5vT09PVDdbjEgZ9hXgSVeC2+IsSWDZIsUsQ0Sbfqa4dLq+5rPbbTgp8hEnWuL4mvYoPiZiD8Fn9iVA8eFLjOknJAFz6iRuOkGgZIXL7WkbPZ0QN1jFDehxg7rtjLT1I7aQcBEfWQOtlB9nJytKo6MWuv46IhMnUpKeyRYV8v+zZ89WcZGhrGdNm3YZHh5WHR0dDdRJvtEJKD4mZBfBh\/YkQPHhCYzJJy6BuLUSphBJm3aRtQvykfR9fX3R+gg9zZIkPsw1JDb1JAGUNuj7ig+XiIGP+DDXe+i1HpJfeOhdJnHiI+2ZTHGkxcfBgwdjG6k5feIa+dixY8eY9SQuIpPiY+L2EXxydwIUH+6smJIExhDQv961ELDFR9Lg7TLtkvVLPckVyMgHUnwkTSMlTbvMmTOnMd3kG\/kw87o2WUY+XEkxHQlgCFB8YDjSygQkYP8it8VH0i92e1Fk2poP+9wJly2hruseXAROUhoz4iAHctmHjMWx6e\/vV\/ZOEXshbNLaEJ81H\/YaG2maWdNHVaz5iFv0G\/faZE3fTcBXjY8cIAGKjwCdykfCE4ib57cXf9oDmBYVMhjqdQPm1I25HTducLR3u7hGIsre7WI\/t8uaj7g6meto0tZ8iDfTdrvEbWs2d\/q4LNJNEh8y9WXu4nEVEHZ7iXvWpFZK8YF\/f2mxfgQoPurnE9aopgTsMyqkmuYveVNY6AFL0sj6jpGRkcZTSR45St3cYmouRrW3wyadH5KGyT7rIm69g0vkQ8pwPefDXJgZF\/WxzzuROn3+859Xd911V2ORaFLkQ+qRdM6HfGduUUaf83HzzTerL3\/5yxHuuGPPk6aUzEW18qzLli1TS5cubaxv0REZsx3I3yg+atoBsFpQAhQfUJw0RgIkUBUBe8tvkXJ91nwUKcclL8WHCyWmaXUCFB+t7kHWnwQmAIG4NR9ZW159sFB8+NBiWhIoToDiozhDWiABEiiZQNw257Sts77VibvVdufOnZXe1stbbX29xvStTIDio5W9x7qTAAmQAAmQQAsS+H\/Verq7LExlwgAAAABJRU5ErkJggg==","height":262,"width":435}}
%---
%[output:2330891c]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:55c0aace]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:6d4591c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
