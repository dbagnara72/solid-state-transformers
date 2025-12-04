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
application400 = 1;
application690 = 0;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 2e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*12; % PWM frequency 
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
% three phase DAB
Ls = (Vdab1_dc_nom^2/fPWM_DAB/Pnom/3) %[output:6e4cc00d]
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
iph_grid_pu_ref = 2.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:1bc16b1f]
Iac_FS = I_phase_normalization_factor %[output:9a535e97]

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

% danfoss_SKM1400MLI12BM7; % 3L-NPC Si-IGBT
% inv.Vth = Vth;                                  % [V]
% inv.Vce_sat = Vce_sat;                          % [V]
% inv.Rce_on = Rce_on;                            % [Ohm]
% inv.Vdon_diode = Vdon_diode;                    % [V]
% inv.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv.Erec = Erec;                                % [J] @ Tj = 125°C
% inv.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv.Rtim = Rtim;                                % [K/W]
% inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
% inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
% inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
% inv.Lstray_module = Lstray_module;              % [H]
% inv.Irr = Irr;                                  % [A]
% inv.Csnubber = Csnubber;                        % [F]
% inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber_zvs = 4.5e-9;                      % [F]
% inv.Rsnubber_zvs = 5e-3;                        % [Ohm]

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

infineon_FF450R12KT4; % Si-IGBT
igbt.dab.Vth = Vth;                                  % [V]
igbt.dab.Vce_sat = Vce_sat;                          % [V]
igbt.dab.Rce_on = Rce_on;                            % [Ohm]
igbt.dab.Vdon_diode = Vdon_diode;                    % [V]
igbt.dab.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.dab.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.dab.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.dab.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.dab.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.dab.Rtim = Rtim;                                % [K/W]
igbt.dab.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.dab.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.dab.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.dab.Lstray_module = Lstray_module;              % [H]
igbt.dab.Irr = Irr;                                  % [A]
igbt.dab.Csnubber = Csnubber;                        % [F]
igbt.dab.Rsnubber = Rsnubber;                        % [Ohm]
igbt.dab.Csnubber_zvs = 4.5e-9;                      % [F]
igbt.dab.Rsnubber_zvs = 5e-3;                        % [Ohm]
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
%   data: {"layout":"onright","rightPanelPercent":36.1}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     2.840909090909091e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     3.869906319672623e-05"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.265986323710904e+02"}}
%---
%[output:9a535e97]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ldrso_pll","rows":2,"type":"double","value":[["0.006118759097557"],["1.356280750433189"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000500000000000"],["-49.348022005446794","0.992146018366026"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"2","name":"Ld_fht","rows":2,"type":"double","value":[["0.007775441817635"],["1.358304305699923"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.176291574541956"],["8.991511697631793"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATUAAAC6CAYAAADPjtq9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2QV9V1P6RJxo0Jg4tEgpsFJIsxNUG0sStCMNmxTI1CJqaF3UzKADF0gpg27Cy7xBljlK8VM36gCY3bHTOVxXaCYTe2QQejrUFqokI0sQGBdSWLBpcQSMSmiXTOg\/P37t337rv3vfvevXc5\/xlH4H8\/zv3dc37\/c+7HuSNOnDhxAvjDCDACjMAwQWAEk9owmUkeBiPACEQIMKmxIjACjMCwQoBJrYTp3LNnDyxYsACuueYaWL58ufUeDx8+DIsWLYLa2lpYs2YNVFVVWe9DbHDt2rXQ09MDnZ2dUFdXV2hfcuPHjx+H1tZWGDduXCFYFjmYHTt2QFNTU9TF4sWLjeR3ibmMCenbvHnzYO7cuUVClqltJrVMsJlVckFq2OfWrVvh+uuvNxNWo7RsYEX2JYtDxLBx40aor69PldZUtgcffBDGjx+v1XZq50IBIuO+vj7o6OiA6upqk+rgE6mh4CgPzkWWsRgNPENhJrUMoPlepWgSFQ0MsSjSC5WxNjEmUxyQ0Nra2kCXME30YLiRmumPiwlWecsGT2qo5Bs2bIhwwJBEDInI+K644opIUfGzevXqQS6zWB\/DQwrfyCCw7rFjx6JwS25fBp+Mgv6djEM2LiqHIQjKTqEIlevv7x8UosjhJX6JIRj96uPfKfxsaWmJvLNdu3ZFbUyZMmXQrykZF34nj1UMj3VwvfPOO+HWW28d0hfJEycD9Y944ocwEOdFDNNEzAkH9NAojKd\/k\/tKkiHp33fv3l0JDUku7CNJljjDk2UhfaL5ojHj3+OIM6k+LieQLo8ZM6aCt4iZjKuIG+nVJZdcEukMftDDEsc8c+bM6N+PHDlS0RdZH0WZTX8w8hKVSf2gSU12yeVfWjJMmvy472ltaPTo0RExkMHQpKESoQIMDAwoPRJV27I3I5IaGaesJGRMKPuVV145aM1MRWpIVAcOHDCSFeW5++67Kz8IOrgSbvLYZNKUZRFxQsJFcsa2aI7EceN6jeiZ0RwsXbq08sMU9z2Rs4ypiWyoBypZ5PAx7YcHiUn8IUqrL+Mm67I8RyIO4o+cqA+ky9i3LC\/+KOB6H\/0Iyvou60jZ67inBampfrWJmOLWfihUuu6664YsrqsMRDWJaaFFkqcm\/vKpQp80g0lS4qSNCVGeG264ITI28txwLCK547\/LWOuEn7LXgR4Z9SWuK8URh7gJIYY5KAsanuihiB6l7P0keRNxsuGPi+qHCTdEVCFX3HfivxGBJ62pyTjIRpz2Q4PlZW+NPMW4Hzm5P1mHH3300UGhOGFJPyhpOm9CQrbLBuupxSmsbPx33XXXoF068Xs5TCNgyW2XPRAVqaX9aumQmmoh2Dap4diIwNGYm5ubgZTVFNckT428r4svvrjiNcb9kMSRGi0niMqORIYL+DKpySES1iHSS\/LU4mRLIrUkWeRdv7gfJXFss2fPVnpqaet5aaRG5I4\/HjLOcaQm95dEajLh0FIJk5ptKo7xHtI8CvylFZU8zsBEMWWDGE6eGo6TDG7GjBmwd+\/eSuhp6gHLpCbjFucVmnhq4pykeTNkqEk\/TCrZdDw1lRq79NQmT548KOogb5uO+Njw1OSxM6kVQGrkbYihStKaGv266KypJSlCmjcmty2uQSStqakWXkV3X\/6Vp\/UOWiORw8+4EFKeAjEEk89M6XgFaWuRuCiN6znoLYubIVnW1OT1O1UIFLe2JK+TJskmE1NaaCximuZNm66pyXOomhMiNZQH138pdFSFn1nW1MSd4TR7KMjktZoNNvyk0ens0uEa0S233BJVUe1+ijuFJp4ayWK6+5m0BiTvfoqeFf4ZQzDckY3b\/aQdTcJFtWNLZeJ24nRwpZ1mua9nn302Wo\/BD+2qjRw5MiI5\/NDmAMpGc5O0+4nlSb44L1K160c\/fEiqhINKNiISXDQnQqAFdJpj1XEP1e6ljmejs\/tJmMs\/ouIuLeox\/jiTfiRtcol1ZJ3CzQQ5tBfniHc\/tfjVfiHVOpX93rjFLAiYnncyOaeWRZ7TtU7cUR\/TcNsX7IL31FRAMqn5omZvyxF3zEY8TpImMZIgbmy4uKKVJltI38tHllB2edc7zbb4RoGDGWdScwB6SpdyiCWGlzrShnz3U2d8ZZaRl0vEw+cqOfjuZ5mzxH0xAozAaY\/AsA4\/T\/vZZQAYgdMQAe9J7bzzzjsNp4WHzAiEjcAfai+HNy5eCIe\/+cnSB1Ihtbi1jjRpcOv4oYceSiuW63sktX379uVqw1XlkGVHzFh+V5pzst+Q8e\/6yauwpOtF96S2bNkyWLFihVbiP9w9WbVqVbQLlfeDC\/r4iUugGPLEhix76EbF8ue1ynz1127thbVb97sltXxDyF6bziklZQINmRj2798PEydOzA6O45osv9sJCBl\/L0jNxbUH7PPmm2+ONCcpPTOTmjvDCtmoEDWW353u3PVYH3z9B3vde2ryuRX5SpFtiDDsnDBhAvT29g7L8JONyrbGmLXH+JvhZbP07Huegyf3HnFPauKgxLt\/cvZUG4PHNbn7778fvva1r0WXnlVravjdtm3bbHRbahuYHaGmpqbUPm12xvLbRNO8rVDxb2hogN9Nb4E\/nn2+X6RGU5CUUM98it6ugafCV65cCfPnz482JYbrRgF7Cnm0JH9dxj8\/hllbqP7qj6KqTo906AhPO56333678Ws4Yvtxif3w+7jNAl5T05mZYsowKRSDq26rIePvNamp0qHoTk5aOfbU0hBy833IRoWIsfxu9ObJl47A7Huf88tTkz2ptFeU8kLHpJYXwWLqMykUg6tuq6HiT5sE73jjdXj923+jO1xr5ZQ3Cop4\/9BUcg4\/TRGzVz5UoyIEWH57uqDbUt\/hN+GiW5+Kir\/rtZ\/Baw98RbeqtXJDSE18U9FaLzkaYlLLAV7OqkwKOQHMWT00\/JHQru96MTrKgZ9R31\/k5IrjIFJzdU1KNfdMajktI0f10IxKHirLn2PyM1R9+PlD8IXOF6Ka0z80Cl5Yd617UsPMl\/TStc6Y+EK7GiU2Kh0tKq4M418ctnLL4uZAbfUZsPPGy5xdyA8i9VCoWTrYqMozqrieGP\/i8ceQc\/Nzv4ZvPLw36gwJrfvLU6P\/u4qymNQKnHc2qgLB1Wia8dcAKUcRJDQ8uoH\/J0JbP++CKPTED5NaAriugMkx15WqbFQ2UMzeBuOfHTtVTXlDgAgNQ07x48p22VMrZt6jVtmoCgRXo2nGXwMkgyJxZIbVH7zuY3DlBaOHtMSkxp6agXqVU5RJoRyck3rxBX9xE0CUVVw\/ixsDkxqTmlsLiundF6PKCgzLnxU5gJ++fBS+8YO9lTNnYkuNHx8Ly2dNjDYDVB8mNSa17BpYUE0mhYKA1Wy2bPzxTQF8WyDugwQmbgLoDME7UqNHY3t6egAfOV24cCHccccdkDdDhw4YYhlXwJjKGVe+bKW0IbPYBstvG1Gz9orEH9fH7nm8D148+PtYbwwlRSL74uXnwvWfrDUT\/FRpV7Ybu1FAhDZt2jQYP348dHV1wZo1a6C7uxu2b98e\/bmqqirTQE0ruQLGVE4mNRuI2W2jSFKwK2l8azblT1rkl3tGItv0xY\/Bh8eemXuIrmw3ltQw3RBdmRoYGKiQGmbixBekyvTWXAGTe0Z599MGhLnasEkKuQTJWDmr\/Ehg7Y\/0Qt\/A8UQvjERCErt26jkw\/7JxqWtkpsNwZbuJRzowHVB\/fz\/MmTMHtmzZEoWfS5YsiULRuKfsTAesW94VMLryqcplVUobfdtog+W3gWL2NtLwR\/I6cvyPcOP396SSF4WT+P+vXz0JLq4daZ3E5JG6sl3lOTV6vo6ELfohlrjpdwVMdlV8u2aaUtroo8g2WP4i0U1vm\/BH8joBAO1b98Mrh9\/UIjAisU9\/dAwsnlFTOIH5ZLt8+DZdtzKXYFLIDJ2ViqHhj+T18sBxuA1Dx9+8Wbl+lAYGHa343MXnwI1XnZdWvLTvXTkkTGoFTnFoRiVDwfIXoxwv45qXoddFkiCBXT5pFLTMmggjTu1QFiNl\/la9IjX5XYK44RWd4pv6dAVM\/inla1I2MMzThitSjsLFEwDtj5iFiyJx4Z+nnPNOuOXaj0b\/nHbQNQ9ORdV1ZbvKjQJ8aHju3LmVMeNjx\/jwMG4U4J\/xeMedd95ZFCZRu66AsTEoV0ZlQ3Zsg+WPR5KyUvx47xHoevqgUagotohEVXvWGTDnovdHdydl4godf1e2m3qkA9\/lpI\/4RB4e9cDjHZ2dnbZsKLYdV8DYGFToSnm6yk+k9bMDx+Cf\/utApAqUotpUL4i4rv7YGPjSjJponUzX6wodf1e2q\/TUNmzYAPT4Cu2E0tuc7Kmlq3foSjlc5SfSeqbvKHT++Fe5SQsbGF9dBf\/QUAuTxrxHm7TSNCh0\/L0jNQRcfCpPXEOz9ahx2qRy+KmDUHFlQjUqJK0\/vXUCvvnvv4CXj70jc3gormVhmPiJyWfB314ytrQ1rlDxJ430ktSKMxf9ll0Boy9hcsnQldJH+Sl8a9\/aC0++9JtcXpZMWpdNGgWfv\/QDpZFWmo75iH+azOL3rmyXj3SYzJJh2dCVsmz5KSx85MUB6N7569yEJZIWHoO46sIx8OmPnm20rmU45VaLl42\/VeEdbvIlkpp8m4AGjC9IdXR0QHV1tW0MYttzxfY2Bhe6UtqSn8iq5qwzYMX398Av+n9nlbAwNMRrPwsvP3eQl2VLfhu6kKWN0OV3ZbuJu5\/4XN7SpUvhscceg\/nz50NNTQ20trYCZu4Qj3lkmSyTOq6AMZExqWzoSpkmP5EVjv+Xr\/0een52CHpfP55rDYuwpB1CJKzzx54JKz9TB6\/+9n+NFuHT5Lcxx0W2Ebr8rmw39UjH5s2bgc6rlblBQMriChgbyhqyUiJhvfLKK3BuTQ386zOvwZN78q9fxRHWRR98Hyy6vAZGjDh5wNTkyEPaHIWMP44tdPld2a4ynxrueM6cOROamprgvvvui7J19PX1cfiZZk2nvvdRKYk08P\/f3dEPT+\/\/bSStyV1D1fDJw8I1rPrzRsEX\/vIDVolKE\/qomI\/4n07ye0VqBPz69eth1qxZgAdtkdgw7VCZCSJRDlfAmCif6\/BTDANRlodfOAT\/8fzrhZAVhoOTzzkTrptxLlS9688qa1g2PSwb2DOp2UIxezuubJd3P7PPWWpNG54CERamnnn4+UPwwxeKJas5U8bA+NFVUShoQ\/5UkAoswPIXCK5G00xqCSC5AkZjzlKLqIyKPJuXfv0GfOs\/X4E9r70RtZf1Oo4sjLjQ\/sHqMwDzal047r0VzypVeA7fdCAqtEzopOzKdlM3CpLufiYd6RBvIaiOf4jlUDOSyroCJo+2knf13cd\/CU8ffKswskKPCj0rDAfxo3unUHdsoRsVy68708WUc2W7g0hNJpq4oarW1fDBlpUrV0ZHQJAM6X5o3DocnoOjB11Uj7i4AkY1zUhax\/\/vLbjrsZeNMpGq2hQ9qz8\/973wuannwJj3vds6UZmoL5OCCVr2y4aOvyvbNfLUTKdNdQRETGOkatcVMCgTktdrR\/8Atzwc\/6irDh4iWaFntfDycTD6zHcX4lnpyGNSJnSjYvlNZtt+WVe2W+hGgcpTw4ddMAsIfSgbiAxtmcAgiT3w9EF4au8R7bUtSi0zeeyZcP0VH4R3jBhROW\/1p98ehIkTJ9rXlpJaZFIoCeiEbkLHv0zbFSGskJpOtlvV2pfYqNhWHFmJ74ri7QQMRZubm6PcbOIaHraJwOBn27ZthWnYqh8NwPdeOJbY\/riR74y+u+r8M2HOR94H9Pc0gfBJQbyJEeqH5Xc7c6Hi39DQUAFu3759pYNYqKdG5IaZcuvr6xMHJ5OcWLAotlc97ore17y\/GAtNl34g15pW6L+0LH\/p9jiow9DxL8p202alUFJTkZUoGJVrbGwcQn62gUkiMySy7i9PzUViMtihKyXLn2Y+xX4fOv62bVcXbSWp4ZpYW1tbpa20dz\/Fl90xjMSNgpaWFmhvbx8UVsrlMPzENba47B82gXnypSMw+97nKuNBIvtE3VnQfOUEq2RGHYSulCy\/rhkVUy50\/G3argnCynTeSDZENBRKYhipeqFdPhZCa2pxhLdgwYLoFXjVy1Q2gEHvbNNPX4U1P9w\/iNBse2bsqZmoXvFlQyeF0OW3YbtZtMToSEeIWTrkcLOIMDMJ+NCVkuXPYlL26oSOv1ekhtOC4WAWT83elJ5sKQ8wcYS288bLbIuY2F7oSsnyl6YqsR2Fjn8e282DvNU1tTyCJNXNA8y9T7wCN255KWq6TA+N19SK0ATzNkMnhdDlz2O75rP9do1Cdz\/zCEZ1swKDXtpFtz7ljNCw49CVkuW3ocHZ2wgd\/6y2mx2xkzWHLalVf\/VHTgmNSS2vauavHzophC4\/k1qCDmcBZvY9z1WuOd3TeAE0fvzkW41lf0JXSpa\/bI0Z3F\/o+GexXRuIKx9eqa2tLT3TrTwoU2C6fvIqLOl6MWpm+qRR0L1kqg2cMrURulKy\/Jmm3Vql0PE3tV1bwCWGn3FpiNIO39oSSmzHBBhcR8PDtZSAcf28C2D6h0YVIZZWm6ErJcuvNc2FFQodfxPbtQmi0Zoa3jDYtGmTtw+viJsDaz9bB9dNd3uZPHSlZPltmpp5W6Hj7x2pxXlqqpP\/5lOmV0MXGPFMmovjG3GjCV0pWX49HS2qVOj469qubfyUa2rYWZmvsccNThcY8V7nbddOhkWnXuu2DZhJe6ErJctvMtv2y4aOv67t2kbOKPy03blOe7rA0I6nL14aji10pWT5dTS0uDKh469ru7YRHBakJq6lud7xFCcodKVk+W2bm1l7oePPpJYw3zrA\/Mt\/H4QbHvyfqAW822n7VSUzVXy7dOhKyfJnnXk79ULHX8d27SA1uJXgPTX5GEeZF9bTJiR0pWT502a42O9Dx98rUpNzn9HU+Zh6SNwguO8LH4HPTj2nWE0zaD10pWT5DSa7gKKh4+8FqeV997OAeU1NPXTbI72w+lTyR0z66PKwrTz+0JWS5S9Co\/XbDB1\/L0iN4E7y1PSnw17JNGAwEwfdIPAp9OTdT3s6kLWl0EkhdPnTbDfrvKbVC3pNTQw9v\/KpWrjp6klp4y31+9CVkuUvVV2GdBY6\/s5Jjd4gOHToEKxbty7KfLtr164hQE+ZMqXUA7kqYMTL676FnuypuSUExt89\/s5JzT0E8RKogBEP3PoWerJRudeo0D2d0OVnUkuwARUwlAjSpwO34jBCV0qW3y0xh46\/V6RGoajP4ad4i8DH0JM9NbeEwPi7x98rUkuCA9fZZs6cOeQV9SLhSwLmgacPwtJNJ28RMKkVMwOhewosfzF6odtqEKTm0+Fb39fT2FPQVf3iyjGpFYetTstBkJpPSSJ9X09jUtNR+2LLMKkVi29a616RmmpNbePGjc7DT\/F8mq+hJ5NamsoX\/z2TWvEYq3rwitTcQjG49zhg1m7thbVb90cFfcrKIePGRuVWkxh\/t\/h7R2ryVSkMPbdv317661JxwHz22zvh8d2\/iWbs8Dc\/6XbmFL2zUbmdGsbfLf5ekdrx48ehtbUVpk2bBnPnzq0gg8TW29sLy5cvLw2tOGB8vu8pAsNGVZqaxHbE+LvF3ytS8zn1kK9ZbuPUh43KrVEx\/m7x94rUyFNrbGwctCmwY8eO6E5omY+xyMCIpLZ81kRYPmuC25nj8JPxLwiB0EnZK1LDOcJQs62tDWi3EwmtqakJbD1oLOZuU12Sl4ERNwl83vnk3c+CLN2g2dBJIXT5vSM11B05aaSt4xzoCa5cuRLmz58PdXV1EYEmbULIwNChW983CZjUDNinoKKhk0Lo8ntJagXp2pBmVTcVkkgNH1fxMTMHbxSUpTXp\/YROCqHL7xWplZ351sRTC+EmAZlb6ErJ8qcTZ5ElQsffK1LDicINgQkTJgw60mF7AsWbC0mhrQhMSJsEHH7a1hbz9kInhdDl94rUyk49RP3h+bf6+vpB2ovA4Gfbtm3wzK\/ehC9tfjX6e8\/8Ghg38p3mml5ijQMHDkBNTU2JPdrtiuW3i6dpa6Hi39DQUBnqvn37TIedu7wXbxQkHfbF0YlsH8qdTw4\/c+ullQZC93RCl98rT82KRikakdfscKOgpaUF2tvbo91Q8SMCE9LOJ4efRWtRevuhk0Lo8jsntbIfXtE9LhJHaiHsfDKppZNO0SVCJ4XQ5XdOakUrWNb2RWBC2vlkUss64\/bqhU4KocvvHan5lqUjtJ1PJjV75JS1pdBJIXT5vSI1H7N0iJsE9zReAI0fH5tV10urF7pSsvylqUpsR6Hj7xWp+Zilw\/eHi+O0MnSlZPmZ1PIg4BWp+ZilQyQ1n7PdikrApJDHJPLXZfzzY5inBa9IDQdSdJYOXbAImNCOc\/Camu4MF1eOSa04bHVa9o7UUGjdYxc6A8xaRia1UI5zMKllnXF79ZjU7GGZpSUvSS3LQGzXIWBCO87BpGZbE8zbY1Izx8xmDSa1BDQRmMd\/+gvAdwnwg7ueuPsZwoeNyu0sMf5u8WdS0yQ131N4i8Ngo3JrVIy\/W\/yZ1BSk9t1HnoXZ9z4XlQjljBqHn24NivF3jz+Tmiap+f4uAXtq7o2JJGBPze1cMKkpSG3xtx4L4kV2eQhsVG6NivF3i793pEavR8mwqF5+KgJCBObCZd+DJ\/ceiZr3+UV2JrUiNCB7m0xq2bGzUdMrUqM0RPPmzSs0nbcOcCKphXRGjdd0dGa32DJMasXim9a6d6S2bNkyWLFixZCkjWkDsf09AjNy4QOAWTqmTxoF3Uum2u6isPbYqAqDVqthxl8LpsIKeUVqOEq8JtXb2wv4boDLz4QLL4Wjf7U2EoFJrdyZYFIoF+\/htnzhFamV\/fCKSnUQmCOf6YiKhHRGjcNPt4TA+LvH3ytScw\/H2xLUXvrX8LvpLUxqDiaFPTUHoAtdho4\/k1qC\/oikFtIZNfYU3BIC4+8ef+9IjXKq9fT0wDXXXAMLFy6EO+64A26\/\/Xaorq4uDbFxV\/0jvPnh2VF\/oeRRI3BC\/6Vl+UtT89iOQsffK1IT03mPHz8eurq6YM2aNdDd3Q3bt2+P\/lxVVVXKjDOplQLzsDSq0EkhdPm9IjUxnffAwECF1PDF6FWrVpXqrb3\/774Nfzz7\/MjoQjp4y+GPOzJmT9k99iiBV6SGAq1duxb6+\/thzpw5sGXLlij8XLJkSRSKlnnMg0gttIO3TGruDSt0Tyd0+b0jNVRJ+arU6tWrS79hcPbf\/xu89Z6zgUmtfJII3ahY\/vJ1RuzRS1JzC8nJ3kPMeMvhjw+aA8Ck5nYemNQS8CdSCynjLZOaW2Ni\/P3An0kthdRCO6PGa2ruDYs9Nbdz4B2piefUCJrFixeXukkghp+hXZFiUnNrUIy\/e\/y9IjUitHHjxlVIjP4NoSrznBqFn+ypla+k7OmUj7nYY+j4e0Vq4jm1urq6Cs74DmjZ59SY1NwZVuhGxfK70x3s2StSQ4HwOAfdJKDbA3h2bcKECaUe6yBSC+2KFIc\/bg2K8XePv1ekpko9JEKFqb0feuihQejJZ9s2btwI9fX1QxCWX39PShNOpBbabQI2KvdGxZ6a2znwitSyQoFkePPNN8NNN90UXXpHgkPvrqOjY8gl+DhPMK5fJLUQD94yqWXVInv1mNTsYZmlpWFBavLAyePDa1Wyt6abWZdJLYs62anDpGAHx6ythI6\/l6SGxNPW1laZE9NrUhhitrS0QHt7+5C3DtCD27BhQ6XtpDAVSS20NN40qNCVkuXPSkd26oWOv3ekhqSDISKFjuR1ocelc6FdTF80d+7cQbMsf4f9NDc3Q2dn5xDyY1KzYyBZWgndqFj+LLNur45XpJb3SEfcOTcVVCoCRFJ7d9+P4anbPm8P7ZJawlRNNTU1JfVmvxuW3z6mJi2Gin9DQ0NlmPv27TMZspWyI06cOHEirqWsnpqKoJIkpjqNjY1D1t6Q1EK8TcAbBVb0M1cj7Knlgi93Za88NRqN6ZqaLqHJnqBqlzRkUnM1qbm18VQDLL8tJLO1w\/hnwy3RU8vSnHz2jNrATYDJkyeD+ECyWBavY8Wtp2F9JLX3PPvPUQjKH0aAEQgLAa\/Cz7CgY2kZAUaAETiJgFVPjUFlBBgBRsA1AkxqrmeA+2cEGAGrCDCpWYWTG2MEGAHXCDCpuZ4B7p8RYASsIsCkZhVObowRYARcI+AtqYlHPlykETeZGB1Z5fTo+H5qmRmEVePRkV+sr7rTa4KbrbK68ovlklJd2ZLJpB1d+cW0Xr7bBI3fRQ5GL0lNzO6Bytfa2grTpk0rNTmlrlLqyooHmfGD92B1DynrypCnnK781AfJ\/swzzySeLcwjj2ldXfnljDG6WWJM5TEtryu\/+EOCV+98tgmR0DBphWkiDFMM5fJekpqcNhwVcPv27d54NrLXIqY415VVt1zeCU6rb4o1yv3888\/Dz3\/+89jsK2n92f5eV370cp544gmtZAy2ZUzzknX0R751gx4QfnSSS5Q5HuxLvPuNfy87W7aXpCYnkFRdoyp7wuT+ssrqi1KayE8EgqEPyh+XUqrs+dCVH8n40KFDsG3bNti1axf4En7qyq\/r0ZWNf1p\/HH6eQkj2YnwmtSyy+jQeE\/lRQWfOnAmjR49OzJOXpuS2v9eVH8vdfffdlZDZlx8VXfkRNzHNflL+Qdv45m2PSe0Ugrq\/XnkBt1HfVFZV7jgb8pi2oSu\/GL75tFGgK7+8hubLD4uu\/CGFn6IOMqmdQkN3ncTUgIsobyKrL4aUZU1QzlSMbagSERSBdVybuvjrkkdZclM\/uvLLnqWPuhSHHZPaKVRCWj\/QldVXJdSVXybCpDTtZZOCrvxiuivaPRQf6y5bbupPV\/44T62\/v9\/LzTP21BK0SffsjitllI18wYIFgEomnh8Sf6X0C+2kAAAD7ElEQVTiPB1fzqolYZ30K+tT+InzoCu\/WM4X7E3kF\/Mb+rLRkWZ\/7KmlIcTfMwKMACOQgoCXRzp41hgBRoARyIoAk1pW5LgeI8AIeIkAk5qX08JCMQKMQFYEmNSyIsf1GAFGwEsEmNS8nBYWihFgBLIiwKSWFbnTtJ6YQskk\/Y0v15LEaaNUPkUeIhaPkYRytSl01WZSC2gGTTJ7mJQ1gSDrIWJfSa2rq6vwA6yqx7pNsOeyeggwqenh5EUpE6IyKWsyOPm6kW5dJrVWaGxshPr6el3IuFxGBJjUMgJXVDU5Qy6FeGLWUzoNf+DAAaCbDCiPqiy2u2jRoijtDn5UoZCYDUIsK8qQFLIlPVKNpHbs2LHov56eniH3RsW2sU9KLEh3I0eOHBnVI7nFGxo4bqzf0dEB1dXVg24YqEJLmaBlGVWn9mWSVv2IsKdWlLXEt8ukVi7eqb2J2SRkYxANBxvC7Kf06y9fXYorS9mDVdec5Ky8clYRVfgpZ5cVy37nO9+JSKmzsxPq6uqifGx0dxH7XLZsGaxYsSL6Tqw3MDAQEffSpUsrmY\/F76uqqiIc+vr6IlLDD5I3Jk9Er0glbxypYaZWkTiT7lcyqaWqsrMCTGrOoI\/vWM77JZZSeQMy+Yhl0aMTs6tim7r3OuNIDuuSVyTKpyIQExJA2Tdt2hT1gaQmX55XZazYvXs3iOtkKi8pjtREElORv8l42FMr18iY1MrFW6s38eKyGD7JpCaGYBgq4Yey0YplMeRsamoa0nfc7qVMTCakpiJdFQmQ14meHH5mzJgBR48ejSW1uPcdRJkfffRRaGtrGzLWuDz5caSGFSlFtpjZAz1I8cOkpqXKTgoxqTmBXb9TMUzr7u6uvNWA3pfowajCzzhPLUkCuR0TUsvqqSHpit6fHH7m8dRUSLOnpq+HIZVkUvNstuI8gN7e3sh7kENKXGtat25dtHaE9cQ1K9WaGq19zZs3b8gLXTbX1ESC3Lx5c4Q0eUGyJ9nc3Bytt1GuM1ojiws\/TdbUaOOCcJLDZTFUlTEUf1Bw7U5eCqAQmdb18Pu4Zw85\/CzXyJjUysU7tTd591PcgSMDHTNmTBSa4eI7LmzjZ\/369dHfaYFcLotlxN1P1cHZpN1PbCPtnJq4+4nlxUX3JFITw08Mt3HDAMeCoTR+4hJSUuiN5XFc6MXG7X5i\/aQn2pI8NSRU+XEWORQVMSIZdu7cGZGavPHBpJaq9lYLMKlZhZMbc4FA1rNzaWtqtsbCpGYLSb12mNT0cOJSHiEQd+wlS2puJjWPJtWiKExqFsHkpspBQA6Ps6bmlu9+yut+NkbDdz9toGjWxv8DZbei79\/tEiAAAAAASUVORK5CYII=","height":186,"width":309}}
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
