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

model = 'sst_three_phase_dab_three_levels_inv';
rpi_enable = 0;
rpi_ccaller = 0;
%[text] ### Settings voltage application
application400 = 0;
application690 = 1;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 4e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*6; % PWM frequency 
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
tc = ts_dab/1000*24;

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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:1dd96712]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:174c5e41]
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
% three phase DAB
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/2) %[output:6e4cc00d]
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
iph_grid_pu_ref = 3.75;
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
danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
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
inv.Csnubber_zvs = 4.5e-9;                      % [F]
inv.Rsnubber_zvs = 5e-3;                        % [Ohm]

wolfspeed_CAB760M12HM3; % SiC Mosfet fpr 3L - NPC
inv_mosfet.Vth = Vth;                                  % [V]
inv_mosfet.Rds_on = Rds_on;                            % [Ohm]
inv_mosfet.Vdon_diode = Vdon_diode;                    % [V]
inv_mosfet.Vgamma = Vgamma;                            % [V]
inv_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
inv_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
inv_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
inv_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
inv_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
inv_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv_mosfet.Rtim = Rtim;                                % [K/W]
inv_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
inv_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
inv_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
inv_mosfet.Lstray_module = Lstray_module;              % [H]
inv_mosfet.Irr = Irr;                                  % [A]
inv_mosfet.Csnubber = Csnubber;                        % [F]
inv_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
inv_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
inv_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]
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
%   data: {"layout":"onright","rightPanelPercent":19.3}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     5.918560606060606e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.857555033442859e-05"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.345132058575099"],["81.455860923503209"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-24.674011002723397","0.996073009183013"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.388772090881737"],["67.915215284996137"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHcAAABICAYAAADMFWryAAAAAXNSR0IArs4c6QAADJNJREFUeF7tXWtoVdkVXoFhOjpjR6PRIGk0yUQ7WEbHOB3j2EoRa+dHtBVbjUJTFVE66I\/6fqAoPuOrPgafQWzBmD8WIv5QKx1xNA4BJ0KhVjs+QhqGSZNJx7bREkj5tqzTdffd53nPTc69OQcCueex99rrW6\/9XDk9PT09FF9ZyYGcGNysxFU1KgHc3bt30+nTpwnKnJOTY7V6\/PjxVFNTQ7m5ub450dXVRevXr1ff7dmzhwYMGEDHjh2jmTNnUmlpqe\/yTB\/s3buX7ty5Y9GI\/58+fUrz5s0LpXy9kIcPH9LatWupurrasQ1o+5EjR2jp0qWuvAubZtBogdvR0UGrVq2ijRs30sWLF2natGk0efJkAuNGjx4dGqPq6uro6NGjdPbs2bSA297eTosWLaIVK1aERrMOLnjS2tpqCaudBOlCZ\/cegAibZvA5p6ioqGfZsmVKusCQhoYG9X9nZyfdunWLysvL6fLlywrsgwcPKkk8efKkorOiokI1sKWlJYE4SOGCBQvo\/PnzBK1nzZ0zZ456jy88hwDhMmm4zhz81uuGJeD39u3bR2vWrKF79+6pMrldS5YsSbi3bt069RwM2LBhg\/UuymaaIOzyO75vAsJEV319vVX2yJEjlTDfvXs36d7QoUMT6gHNK1euVDy7dOmSog0W1c4KmepG+8H\/nIaGhh78g8pPnTqlwIW5OXPmjAI4Pz9fmWk8v3btGl24cEGZP6khEydO9AQuBAGNttNcMJvLR6PAXIAPMCTQTs90zZWWh8EEUGAqBA0CyuUzuFIgdZpR\/urVqy3LIwWZy2SrIWnGd7t27aIDBw4owNC2wsJCo3LI7x48eGApCisCK4cTT5TmyoDq6tWrtHz5cjpx4gTdv3+fDh06pIAFw7nBkEKWfK5El2Y7zXUDl8vZv3+\/Kpq1f8yYMQlAs9axkMCNsM81mWWTFsInQ2tZI51AknThOymc\/B1okpYIv01mWVoLk+WbMWNGQluZ9vnz5ydoL99n4dd5AiuRAK5TA02FhQ0um2YIEC4nwKTvtgN31qxZlnkD46XA2IEk39N9JMzjkydPlLnkmIFpZhMqQTZpIADdunUrbdu2TRWvuzW2gvDp8oK5lkplcg+SJwngAjz2udJsLV68WPkrNl1MEHxd2OBK6cvLy7NMspuU2oHr5C78aK5kslNAyFrJvQv0PFhA5f\/gnexByJhF11xdwPi3G08UuI2NjT0LFy6kFy9eqO4P\/uS4Bv6fPXu2kjDpLzkQgFlAAKb7Ry\/+y9QVYomE5EozF8TnSkbJIEX3ufozN5\/L7gJtl2AXFBQkgIbgk8FFvMLmHK4DZdgFpKH5XFO0LBuvdynsIla7yFNnFEspwDNFgWzmmpubk\/rWsm5ppiQzWCtgJsG8SZMmKTOIC7S0tbVZ3aR0RMscGUNwpT9GHHP8+HFlBfEOLBMuBKc6zVu2bKHt27f7jpYlT5ICKjsTkI33dZ\/lp\/8NYcKlB5ZR41NOe3t7j+zPmQhMZYQqag2W9EhLgPtO\/Un5ndcRqr5uezy23NcIpLH+BHBNYT07fRkdm+iBWbt9+7brkFwa2xIXrXHAAlf2MaUv8TKOyv6rrKwsBjdCIpYwcYCONSJLOfuD\/pTpPrcBQrFz504qKSmhpqamBHCLi4sj1NTMJuW\/hR\/QfyYupo6DP\/LckASzDC2Voy+skTz+ameOcX\/UqFFUW1ubBO6jR488E9NXLz5+\/JiKior6qnpP9TZ3PKcJOxqCg4taZN\/PLYIE+OfOnaNNmzap\/psJ3OvXr3sivi9fQt8bAxBRvirOtVDrN92pgeungbog4Fto+eHDh1UxMMux5vrhaPK7sz7+nD79otN6ENgsp0IGRmNisxyMgzC5nz3+J\/3+Tis1f\/2c8Nt0vfKPv9JXv1vuuZKEgErOMbp1ffQaYnCTeQ6QCoZ8i2obv6S6xi\/VC07gmVArzH2Npr41hI7N\/65vS+jaz9WnmjyLTRaZZdYkMPqLti6qudVCf\/77vwKB5cQ\/lI\/rw3HDaPfPkteX+XVzriNU0EhE0UEWyPklxo\/gBH0XQGGm6w9NX9Hf2rqoub2Lup4\/p7YusjWHQeuS3zFwhUNeo+K8gfTzshH0QclgVSc\/c6vHLz9dNVfOcrhVrj\/3S4xb+WBE3qBX6ebDr+mTBx0J2qNMno2vcis3yHMJCAAbm\/86VbyTRz8sHWLR4RU0r\/X75WeSz0VFQbTURKAbMQxG9dUnSoNkVOi1wam+BwC6u7upOO8NVVTpiIH0k3HDaOyI1y2N8qNdqdLj9L0bP\/VvXc1yKsTaEYMA46Pav6RStPpW1x7cKxo2gN4vGkzvF71JJXkvV4u4gZMJgxhoR+TBzf3Nn4ygAiiYt5LhA+kXZflUXvymJ2BSlhAiisENwEVd0iSwALP+1+96DiYCVO\/5kxhcz6z6\/4sMLo+L8pOmzeWRAJXpyXpw7WZ\/3GaFvAQAcggtasCC\/qwF18syG6cJe7lURV+mwprL5himGOBG7cpacHnHQRANlUOOmFnRd74B3F8d\/SP99vpThaefQe\/eFICsBReai8l4rN3FHhjeRCWZ62WBHKb\/eC8MT\/YD3M6f1ljdlihqbVab5TA2X7Np1s3y6O99n7758ctloIiMp741uDcV0nNdWau5YYALLvIarClTplgblr7zg3n07\/deTlFdqiqgkd9+xTPDe\/PFTJisBz+mT5\/ua348afgxqFlG5fpi7eG\/PEHdw8aqbk9UTXK\/NssAjHfZ69qEgOrGjRtq5T1vTML\/vI+Uo+QZbw+luqXv9KYy+qqr35pltyjaqSvE4H5c+TZVvpfvi+G9+XK\/BTeV+VwGN8rBVL8wy\/ruc6k5+o5xr1rF4Ea1f9tvhh+9AubnPYAb9WCqX2guGqnvXpMHkAQ5gwrgTi0ZTPUfvetHJnr93az3udxPraystKJdcDmVDV4A92zVOJo9fnivA+anwqwHNx2zQgD3jU+rCett4yscDvhZ5J+wzEbfXe50gk04pMalpJMDSWuo5IEjqNjrbvN0EhmXHYwDaV0gF4yk+KuwOOC6PzesiuJyep8DSftzwzihVR7REyWzLo+FsJuj1t1SKttpwobTrkdjV0+os0KoRB7ti9\/6BH7YDfZTHgJGHO\/Hp+GZBNm0oc1PHel6V44geh0xDN3nyrFoPjxL7zuniwFO5erzzXYgSgHoCzpNdfKh3HPnzlWrZeTMmxON6hyqMJbZcCWSabiHMw7lBH5fMUw3aaYJEf00n1T2SaWjnaZpVUdww1qJkQ3g6oxKZUYscuCGMbac6WZZghK1k+ICa25YY8uZHFDxsUtVVVUq\/0Iq4+qR0twwx5adTg9PR6O9lil9qlxoDxBxIY+A7Apllc+Nx5a9iklmvBePLWcGToGoDL2fG4iK+KO0cCBUcL0OAISZiEr6US9DhX6H8NLBdUlz2MOzHO8gpsh4cN2W3urgRAncdI3c8bGNgcA1RZSclAHMZA2SxweasmKx1DqtfWZw9NEjjK9yviHOG2BKISfLlhnKJkyYoA520XMtmGjmxBRIonXz5k2VWAMXklBwe2X6N7f2mEbLcPreoEGDrCxqpkNnpMVzEtLA4JoI410H0izroztyq4mepYsP4ebkFkgaZcp+xQwEmJyVC1lSOAehnu1E0sO7EHfs2KEynuFCxhVZFgRUnrnFNHP2Ek6YJQcTTJlM3Npj4iEERQo76NNzKKQEruyfSnMmp8ecJMbJ55pmZDipkxx7NvljvU45CYD0MiZw7ejUJxDkoIudcDC4TKc+6cC\/OauI3\/boimDaDgs8AoPrZ2hLTj9J4CWApiP12VwzkTJjlxQmPTAy0cZl2IFr1x4ddAkum1+Z3UsmTmQfqY9c6eDK76Wb0l0Ml6cLS1rAdToR3S5qlIQhqRTPl+oMcNJct8AiHZrLdUpwkUVL5mlgmllz7cDQwfXbHl1z7SYspOY6KaPR53rpyuhmTO70k99LcAGOnjGMJ8pN7+lJCdkk+fW50s\/zJACSMSHDqBu4kmYdXK8+l8vQ22Pnc3kS3i5vkcw3wWlUTRP3SeA67RXSl6RI3yyf8X09by3eQUq4K1euWGnjkAHTFC3b9VVN0TKCLiefafqGAyATuDKXraSZfanUSG4rgiwIrszvYJe5zMksc15gBKdy3NukrQzss2fPlJCi1yDjjsDRcjo69Zlept8MYXZdIU77nio\/jODqsyY4nWbz5s0q1XlY+eVTJTwK3+uL6LzmXjL12WG9TMk\/grbTOELFwMLMIP04J6ZAkBQngwrK6r797n8zsx19072LsQAAAABJRU5ErkJggg==","height":72,"width":119}}
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
