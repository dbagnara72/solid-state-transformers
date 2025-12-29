%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
simlength = 3.75;
% simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

model = 'sst_cllc_ttype_inv';
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
fPWM_DAB = fPWM*5; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

fPWM_LLC = 15e3;
fPWM_LLC_pu = fPWM_LLC/fPWM_DAB %[output:8141aa1d]

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

% t_misura = simlength - 0.2;
t_misura = 0.75;
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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:673f9826]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:6e722844]
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
% LLC
fres = fPWM_DAB;
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/4) %[output:3903d2b4]
Cs = 1/Ls/(2*pi*fres)^2 %[output:0f15b374]

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
ki_i_dab = 2;
kp_v_dab = 0.25;
ki_v_dab = 18;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:8af4eac5]
Iac_FS = I_phase_normalization_factor %[output:3abfa9ca]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:9834b857]

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
omega0 = 2*pi*f_grid;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:391377db]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:8f98dfc1]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:6d3af681]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:56039393]
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
%[text] ### Notch filter
a = 0.98; 

omega_notch = 2*pi*100/(ts_dab/fPWM_LLC_pu);
C = cos(omega_notch);

B = [1, -2*C, 1];
A = [1, -2*a*C, a^2];
FLTnotchd = tf(B, A, ts_dab/fPWM_LLC_pu);
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
freq = f_grid;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:63162853]

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
figure;  %[output:29d0656e]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:29d0656e]
xlabel('state of charge [p.u.]'); %[output:29d0656e]
ylabel('open circuit voltage [V]'); %[output:29d0656e]
title('open circuit voltage(state of charge)'); %[output:29d0656e]
grid on %[output:29d0656e]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:47a778fe] %[output:5cf529bd] %[output:7174987d]
%[text] #### DEVICES settings
wolfspeed_CAB450M12XM3; % SiC-Mosfet full leg
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
dab_mosfet.Cgs = Cgs;                                  % [F]
dab_mosfet.Cds = Cds;                                  % [F]
dab_mosfet.Cgd = Cgd;                                  % [F]

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

wolfspeed_CLB800M12HM3P; % common emitter half bridge
parallel_factor = 1.75;
invt_mosfet.Vth = Vth;                                           % [V]
invt_mosfet.Rds_on = Rds_on/parallel_factor;                     % [Ohm]
invt_mosfet.Vdon_diode = Vdon_diode/parallel_factor;             % [V]
invt_mosfet.Vgamma = Vgamma/parallel_factor;                     % [V]
invt_mosfet.Rdon_diode = Rdon_diode/parallel_factor;             % [Ohm]
invt_mosfet.Eon = Eon/parallel_factor;                           % [J] @ Tj = 125°C
invt_mosfet.Eoff = Eoff/parallel_factor;                         % [J] @ Tj = 125°C
invt_mosfet.Eerr = Eerr/parallel_factor;                         % [J] @ Tj = 125°C
invt_mosfet.Voff_sw_losses = Voff_sw_losses;                     % [V]
invt_mosfet.Ion_sw_losses = Ion_sw_losses;                       % [A]
invt_mosfet.JunctionTermalMass = JunctionTermalMass;             % [J/K]
invt_mosfet.Rtim = Rtim;                                         % [K/W]
invt_mosfet.Rth_mosfet_JC = Rth_mosfet_JC/parallel_factor;       % [K/W]
invt_mosfet.Rth_mosfet_CH = Rth_mosfet_CH/parallel_factor;       % [K/W]
invt_mosfet.Rth_mosfet_JH = Rth_mosfet_JH/parallel_factor;       % [K/W]
invt_mosfet.Lstray_module = Lstray_module/parallel_factor;       % [H]
invt_mosfet.Irr = Irr/parallel_factor;                           % [A]
invt_mosfet.Csnubber = Csnubber;                                 % [F]
invt_mosfet.Rsnubber = Rsnubber;                                 % [Ohm]
invt_mosfet.Csnubber_zvs = 4.5e-9;                               % [F]
invt_mosfet.Rsnubber_zvs = 5e-3;                                 % [Ohm]

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

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":47.6}
%---
%[output:8141aa1d]
%   data: {"dataType":"textualVariable","outputData":{"name":"fPWM_LLC_pu","value":"0.7500"}}
%---
%[output:673f9826]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"458.3333"}}
%---
%[output:6e722844]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"937.5000"}}
%---
%[output:3903d2b4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"1.2784e-05"}}
%---
%[output:0f15b374]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"4.9535e-06"}}
%---
%[output:8af4eac5]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"391.9184"}}
%---
%[output:3abfa9ca]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"509.1169"}}
%---
%[output:9834b857]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.3451"],["81.4559"]]}}
%---
%[output:391377db]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht","rows":2,"type":"double","value":[["0","0.0000"],["-1.4212","-0.0002"]]}}
%---
%[output:8f98dfc1]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0187"],["3.9119"]]}}
%---
%[output:6d3af681]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0003"],["-35.5306","0.9953"]]}}
%---
%[output:56039393]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.4665"],["97.7979"]]}}
%---
%[output:63162853]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.0911"],["4.7089"]]}}
%---
%[output:29d0656e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x8aWluUECMQEpJDnIAMdE9X2F5q4ZC4KRCftkdmm01qie6x6iqpe4DgPdiW5ZDQpSHYkm1OXT5alyqqvKf+SLLQoG26LE0JDVHiQw1Wk5QEQTHGElIEhgLBKCHRnue6I0ZX92Nm3nnee9+r\/z1HB6x35pm5v2femb+e+TptampqivCAAAiAAAiAAAiAQAUROA0CpoK8haqCAAiAAAiAAAgEBCBg0BBAAARAAARAAAQqjgAETMW5DBUGARAAARAAARCAgEEbAAEQAAEQAAEQqDgCEDAV5zJUGARAAARAAARAAAIGbQAEQAAEQAAEQKDiCEDAVJzLUGEQAAEQAAEQAAEIGLQBEAABEAABEACBiiMAAVNxLkOF80rgxIkT1NHREVSvt7eXqqurxao6PDxM69ato2XLltH27dupqqpKrCzdcDnf0eSFFIdPf\/rTtGbNGpMsuU7T3d1Ne\/bsCeq4fv162rJli1V9Dx48SFu3bqVt27blkge\/33e+8x3x74cVNCSuWAIQMBXrOlQ8bwTKObjHCRgeIFauXEnNzc0ieOLeUbrcuJdxGRB5AH300UetxIFLHlsHcBnXXXfddLYiCpiiCU5bHyO9XwIQMH55whoIZELg5MmT1NXVRQMDA7Rv376yCRiO\/JSj3CioajBsaWkxFiMqQmEjDlzyuDQCJWBs6hYuJ+8RGNVOjx07hiiMSyNBnhkEIGDQIHJJQA+lcwX1kLgefbjsssvo85\/\/fPAOPJDp0ykqWjA0NDTrc93GtddeS5\/85CeDNPX19dTX10eNjY2RXHShEE4fjk7w52pK6aqrrqI777yTmpqago5bH\/jT7PBUlCr38OHDQf34UVNIt956K\/3Jn\/xJIF7UE2bBv1dMdYGjBk09vRoElS19QNXf8e6776aenp7Ico8fPx7Ub3R0dLpOug91jsz8nnvuoS9+8Yuk3o\/5h1krdmpqLmqwVn7Vy1XvG34v5euFCxdOi7AwvwcffDCYklGP3j7C9tKEY7g9JtlKaodJkRqdCddZ1T0sisLfL52tsnHzzTfT17\/+deLvj\/Kd\/s78O1WG7ts0LlHtMJedECqVewIQMLl30dyqYHjQ0t9edcJRg1R44GE7LB6UeAl\/HjXAJg3+\/Flc3dRgc84558xYA6MEjF4HFgpRgkMXMWE7vgRM1F\/44cEkPLDFceXfxwmYzs5OuvHGG2exTxIM\/FlNTQ1NTEwEAi1KVHCZ+kAbrntcu1DlPvHEE5Fi5P77759ed6K3N32ADguYsC31eZyISWqznOf555+PFUp6ncLiJSwylXj40Ic+RN\/85jdndB5RIiTq+xUWIJwmqo78e1VOmm2dS96jRHOrx63st4WAqWz\/Fa72qoPW\/wJVnT+\/rB594L+yVceoDxB6Z6v\/5akPeCwSVIRA2VBlh\/\/SV5CjPtc742uuuSZWwOh\/odraSRMwHHXiJ20qJylCxFGhl19+eRYTPWrAnBYvXjzjHU2mkMLTW4q98idHW8J+57rwepCoyBCzXL16dfC+esTGZGGzyXRQOE3434qJEltc\/7SyVduLeh\/1Oxa6\/M5xU0g6R9Wewj59+OGHAyEUFVGJsxuOwqmok24jqWwVoVHtP42Lj6mywnV8eCEnAhAwTtiQSYpA3F9nagDgjnvp0qWRO3D0NEePHo38q5rrrdvgv\/rVjqG0RbhpfznGCQS9Q+fybe34EjB62SxG+NEHzLiBRR\/AP\/WpTxkLmKiIVVS5XI\/wFFlchIPT8kCs6qGzjSovPJWWJGDips7CeZKiKVHiN\/xuanoyLISUaIsTGmntU\/evbiPOr+FojmKlBEzc1KG+w05vy+p7qU\/fqX5C5xI1bSnVn8BusQlAwBTbvxX3duUWMPo25LQBwlZ4MPyobdWmdvTBOTzYsW19G7VJBIbT6Atf+d+8ZTccgQoPoLYCJswxHKUJCydfAkY19jjhxDuzogRMeCoqLQJTCQImKuKn\/Bpuf3ERGN1G3HcDAqbiuthCVRgCplDurPyXcZ1CCk91qDUFcX\/NRoX80wRM0tSPHhVgL\/BfqXECxtQOh+bD4kJNrYUFDIsEk8WR4cFdj1CEp+F4wE+bQuLoUJoACNtwnULSW3dcVCP8DQiLkXA0Qr2zHolT76PaTjhP1BRS2jdPegpJiV0VuYoTMFGRK8UoHIGJW3Qdnr5KmkKK4oIppLTWgs9NCUDAmJJCurIQkF7EmyQA0gRMUt2i1ofECZg0OxxuV+tZwtBNBAznidqFpGyFd5LoB8DZLOJVUwl6Hi73Yx\/7WBAdinqYU9T7mS7iZZtK1IWFU9wCVz2PnobL3L17N91+++2zFhxznrCA4d\/FLQhW75ommKOmV9IiYDrHuHdMEh+6YLjpppti21aSDa5D1OJe00W8Ope0CGRZOhoUUggCEDCFcGPxXsJ0G7W+BTptG3XUwmCbKSSmnDQ9kbZIVj+ZN8kOlxPecvu5z32Ojhw5Mr1oNSoCow9ucQuRddvhtTlRAkcfyPW8\/P9KwESVe9999804UZYP19PX27hso9aFiD6gRm2xj9u+Heaqr8lhm8xt586dtGnTpgCHHklTu8nitmWnnd+StI2ayzKNTMStXeEoXJQ4iIs6MaOoLexRUZw48cu\/D5\/8G7eWSNkwiRQWr0fDG0kQgICRoAqbogTSdnyIFg7jJROImqoK7zSLO4dHL9zlILuSKz9HDeiCUwm1qJ1JaXhwkF0aIXxuQ6BQAoY7ND6Dgg\/XiuoAkw42s4GGtNkSgIDJln+ppSdNoSVNfUWV63KVQKn1n6v5o6aQmEXa4Y9RorMod1fN1baQl\/cujIBJW9SnPl+xYkVwyZn6N3\/5bC9My4vz5mo9IGAq3\/PhPyb4jWzFC+fB3TrlbQvhqV0b8cI1heAsr7+KXlphBAzP8\/KXg5+4CEzYmfwXxeDgYFlv8y16g8L7gQAIgAAIgEA5CBRCwPBfc7fddhu1tbUFIsangOEj5\/kHDwiAAAiAAAhUKgG+noN\/ivQUQsBwJIUfPgkyaQ2M7jgVwm5tbQ2mlKIeFi6bN2+mQ4cOFcnneBcQAAEQAIE5RmD58uW0Y8eOQomYihcwPAfe399Pt9xySxApMREwav0Lt1\/99uJwe1bbD9npDQ0Nc6y5y74ui0I+gwNs\/XMGW\/9MlUWwBVs5AnKWVbtNuzFdrgYylitewPCUEZ8xwaeGpu1CYoSm4oXTKgFTNKfLNCU7q2Brx8smNdja0LJLC7Z2vGxSg60NLbu0RWVb0QImaieDcmuU6LDdeVRUp9s1fZnUYCvDFcJbjivYgq0sATnrRe1vK1rAhN2dFoHhaA2fPpk0baTbLKrT5b4m5pb5tmie+mtvb6dFixaZZ0TKVAJgm4rIOQHYOqNLzQi2qYicExR1LCu0gFERF96dtHjx4uBmYHUcuGoJSUeuF9Xpzt8CjxnfeustevHFF+n888+n+fPne7QMU2Ar1wbAFmzlCMhZLupYVigB49v9RXW6b04u9jAQuFAzywO2ZpxcUoGtCzWzPGBrxsklVVHHMgiYhNZQVKe7fAF850Fn5ZvoO\/bAFmzlCMhZRruVY1vUsQwCBgJG7luTYBmdlRx2sAVbOQJyltFu5dhCwMixza3lojo9D8DRWcl5AWzBVo6AnGW0Wzm2RR3LEIFBBEbuW4MIDNhmQkCuUAyyYCtHQM4yBIwc29xaLqrT8wAcA4GcF8AWbOUIyFlGu5VjW9SxDBEYRGDkvjWIwIBtJgTkCsUgC7ZyBOQsQ8DIsc2t5aI6PQ\/AMRDIeQFswVaOgJxltFsZtsdOvEXdDz1Hf3\/Lb1PRrsVBBAYRGJlvTYpVdFZy2MEWbOUIyFlGu5VhywLmstu\/TQv+tgMCRgZxPq0iAiPnF3RWYCtHQM4y2i3YyhGQsfzYM6\/S6nufhICRwZtfqxAwcr7BQAC2cgTkLKPdgq0cARnLEDAyXHNvFQJGzkUYCMBWjoCcZbRbsJUjIGMZAkaGa+6tQsDIuQgDAdjKEZCzjHYLtnIEZCxDwMhwzb1VCBg5F2EgAFs5AnKW0W7BVo6AjOX9j4\/RDfufwhoYGbz5tQoBI+cbDARgK0dAzjLaLdjKEZCxrATMWf9vCx34qz+j5uZmmYIysIpt1AnQIWDkWiQGArCVIyBnGe0WbOUIyFhGBEaGa+6tQsDIuQgDAdjKEZCzjHYLtnIEZCzzIXbdDx3FFJIM3vxahYCR8w0GArCVIyBnGe0WbOUIyFiGgJHhmnurEDByLsJAALZyBOQso92CrRwBGcsQMDJcc28VAkbORRgIwFaOgJxltFuwlSMgYxkCRoZrplaHh4eps7OTenp6qLGxMbIuEDByLsJAALZyBOQso92CrRwBGcst9zxJ33r2VayBkcFbfqsnT56krq4uOnz4MPX19UHAlN8FhIFADjrYgq0cATnLaLcybCFgZLhmZpUjK93d3UH5iMBk4wZ0VnLcwRZs5QjIWUa7lWHLN1HzjdS4jVqGb1mtnjhxgm677TZqa2sLRIyJgNmwYQMtX758up51dXXEP3jcCXBnNTY2RrW1tVRVVeVuCDlnEQBbuUYBtmArR8CfZe5b+Yef3\/rSyeC\/EDD++GZm6eDBg0HZS5cuNV4DE65se3s7rV27NrN3KELBk5OTNDExQTU1NTRv3rwivFJu3gFs5VwBtmArR8Cf5b1791J\/fz\/9\/Ixz6bWPnpptOOv\/bqQDX7wXJ\/H6w1xeS7xwl516yy230PHjx40FzI4dO6ihoQERGI\/u4nVI4+PjQSRr\/vz5Hi3DFNjKtQGwBVs5Av4sqwgML979wuFTfyAiAuOPbyaWeMpo5cqVgQLFLqRMXDBdKOa75fiDLdjKEZCzjHbrn626iZotn\/lYD33pz\/4YERj\/mOUt8tqXjo4OGhoamlXYvn37Ip2KbdRyfkFnBbZyBOQso92CrRwB\/5bVGTAQMP7ZZmoREZhM8WMbtSB+DLJycMEWbOUI+LestlD\/wpsvEd9GHffHuv+Sy2Nxzt5GDQFTngYWVwoGAjn+YAu2cgTkLKPd+mXLW6d5CzU\/p7\/0w2AKCQLGL+NcW8MUkpx70FmBrRwBOctot2ArR8CvZX39y8r3jNNQ\/2cgYPwizrc1CBg5\/2AgAFs5AnKW0W7BVo6AX8tq+oit3rJsku753PUQMH4R59saBIycfzAQgK0cATnLaLdgK0fAn2V9+uiC6vn0F1efRtdddx0EjD\/E+bcEASPnIwwEYCtHQM4y2i3YyhHwZ1kXMA9ev4ROf+kHEDD+8FaGJQgYOT9hIABbOQJyltFuwVaOgB\/LLF5W3\/tkcP8RR1+OfPYKKupYNmd3IZk0laI63eTdpdNgIJAjDLZgK0dAzjLarR+2+x8foxv2PxUY2\/Xxi2ndinoIGD9oK8sKBIycv9BZga0cATnLaLdgK0egdMvhtS8cfeGnqGMZIjAJbaaoTi\/9a1K6BQwEpTOMswC2YCtHQM4y2m1pbFm8cOSF7z\/i5562S6nt8joImNKwVm5uCBg536GzAls5AnKW0W7BVo5AaZbveuQY\/fHAs4GRD164gAZuWDJtsKhjGSIwiMCU9q1xzI2BwBGcQTawNYDkmARsHcEZZANbA0gxSb72vZfov3\/xu8GnauGunhQCxp1txeYsqtPz4BB0VnJeAFuwlSMgZxnt1p4tTxt9\/Qcv08avPD0tXnjbNIsYCBh7noXKAQEj5050VmArR0DOMtot2MoRsLPM4mX\/4y9S90NHE8ULf1jUsQxTSAltpqhOt\/uayKTGQCDDla2CLdjKEZCzjHZrx5YvamQRww9HXO5uvZSuvGhBpJGijmUQMBAwdt8aT6nRWXkCGWEGbMFWjoCcZbTbdLYsWHiXkTrnRYmXqGkj3RoETDrbwqUoqtPz4Ch0VnJeAFuwlSMgZxntNpkt3y5944GnpqMunLrt8vPpnrZLUp1S1LEMERhEYFIbv0QCdFYSVE\/ZBFuwlSMgZxntNp4tR1z4hF318JTRllXvnz7nJc0rEDBphAr4eVGdngdXobOS8wLYgq0cATnLaLcz2fJ0UfdDz80QLpziul87nzo\/umjWTqMkzxR1LEMEBhEYuR4pwTI6KznsYAu2cgTkLKPdUjA9pISLOlFXj7qkrXWJ8w4EjFy7za3lojo9D8DRWcl5AWzBVo6AnOW53m71W6R1yjxd5CpclJ2ijmWIwCACI9cjIQIDtpkQkCt0rg+ycmTn5totXpjL00ThaAtz5usA+D6j8KF0Lj6AgHGhVoY8J0+epK6uLhoYGAhKW79+PW3ZsiW25IMHD9LWrVuDz1taWmj79u1UVVUVmb6oTi+DW1KLwECQisg5Adg6o0vNCLapiJwTzBW2LFoG\/uVHdN9jI7NYsVj5+NJaWttc70W4IALj3BzLk7G7uzsoiEXLiRMnqKOjg1pbW2nNmjWzKsCChNP39vYGooWFT319fazggYCR8+Fc6azkCMZbBls56mALti4EWLQ8cGSc+gZHI7OnHUTnUqaep6hjWeGmkHRBE3Y6R18GBwenoy7hf4fTF9XppX4ZfOTHQOCDYrQNsAVbOQJylovWbpOmh5iiEi38Xx\/TREmeKepYVigBoyIwHI1pbm42isCsWLEiMlrDmYvqdLkuyNxy0Tor8zeXTwm2cozBFmzjCPAi3L\/+9ijd\/+T4jMPm9PTlFC2IwMi1Ve+WOfKyZ8+e1HUtw8PDtG7dOhodHaV9+\/ZFCh1VOSVgNmzYQMuXL5+uc11dHfEPHncCPBCMjY1RbW1t7Bokd+tzOyfYyvkfbMGWCYy+9nYgUqamfk53\/uNI5CJcTld\/1ul0\/rtPpy2r+NyWKvFIi\/IO9638o56RkRHavHlz6pgn510Zy4WKwDAiFjIsTqIW5\/KU0YEDB4I1MNXV1UFafuIW\/SoBE0bf3t5Oa9eulfHIHLE6OTlJExMTVFNTQ\/PmzZsjb12e1wRbOc5gO3fZsmj5yvdep++NTdLhkVOXKEY9LFpWX3omLW2YT8sa5ssBS7C8d+9e6u\/vn5Ui7Y\/2TCpbQqGFEzAcYens7KSenh5qbGycRqN2K+lTRnFpVSYlYHbs2EENDQ3TthCBKaHF\/UdW9sf4+HgQyZo\/P5sveelvkU8LYCvnF7CdO2wfe+YVuv\/JH9E\/PftaEHGJe3haiLc8f2JJTRBhYQGT9ROOwBw6dIh2796NCEzWjkkrX99pxFEW9ZQiYIqmWtMYluNzrCWQowy2YCtHQM5ylu2WF9xOvv1z+tOvPx87HaTeXAkWvkixHAtwfRAv6nrOTCIwarHt0NCQlW+amprogQcemJFHnwZSIiVua3TUFFLcdBMXUlSnW0EXSpxlZyX0SrkxC7ZyrgDbymfLYuW1t96mP3\/0hVSxwm\/LImXl4mraePX7gpeX3jEkQbioY1mmAiZut1CUA1VkJSxgwgfZ6YfTqc\/a2tqmF+uqxb5cBg6yk\/iqmNnEQGDGySUV2LpQM8sDtmacXFJJsGWx8vbPf067Hk6PrChx8t6z59Pmjy6iReeUb9GtCy+bPBAwNrRS0qZtd7YRMB6rNctUUZ0uyczUtkRnZVp20dOBrZyHwTZ\/bHk3EEdF+L88BTT8ozeNIyssVn5v+fm0cMGps1gqMbpi4pGijmWZRGA4MsI\/+hoVEyeUO01RnV5ujlHlYSCQ8wLYgq0cATnLpu2Woyon3vwp3ffN40ZCRY+sXHnR2cGC2ysvWiD3Ijm0XNSxLBMBo6+BSbu7KMu2UFSnZ8lUlW3aWeWhrpVWB7CV8xjYloftj96kIKJy5Pjr9ND3X6IXXnkr9nC4cI04isKRFT57hei0QkdWTL1R1LEsEwGjoOvrUXjhbV9f34ytz6bOkUpXVKdL8bKxi4HAhpZdWrC142WTGmxtaKWnZZHCP8+\/fJIO\/PMYPTfxRuKWZd2imu75xLJaWtlYDaGSgLuoY1mmAkbxDu9KyktUpqhOT+9W5FNgIJBjDLZgK0fAzTJP+7Dg2PPN4\/Qvx183nvpR0z\/8X576UVuX9d+71Whu5SrqWJYLAaM3Jf2of\/59lmewFNXpefjqYpCV8wLYgq0cgXjLKpryi+86jW7\/2r8FCb\/17KvGVVEHwF31KzW08epFQWSmyAtrjcF4SFjUsSx3Akb3VdyhdB78aWSiqE43ennhRBhk5QCDLdhKEWBRwc8zP3qT\/vd\/XFpoI1JU5ITXqPzGxdV0+aL3TO\/8Oe8MohdffJHOP\/98nM7t2YFFHctyJ2AQgfHccnNqDoOsnGPAFmxLIaAiKU+P\/5gOH3stiIS4iBSuA0\/7fHxpLf3iu34hNZqCdluK15LzQsDIsQ22VHd1ddHAwEBQStoBc4JVmWG6qE4vF7+kctBZyXkBbMHWhACvS\/nxT35Gdz9yLEhuK1L0aMoffLCBas78peloist5Kmi3Jl5zS1PUsSzTCIy+C4ndkuV6l6hmUVSnu30F\/OZCZ+WXp24NbMFWTfXwfx\/615foyAuvW21F1gmqbcm\/c9l5tLj2l4OPJNamoN3KtduijmWZCBh911Feoi0QMHJfnijL6KzkeIPt3GDLERR+fjY1RV\/65zGnqR4lRvi\/vC7lY0vOo8bzfrmkSIorfbRbV3Lp+SBg0hkZp3jmmWfopptuoltvvXX6jqK0zHF3IaXlK+Xzojq9FCa+8qKz8kVyth2wLQ5bFinvnv8uuu+xkeCsFJdpHl2k8MLZX204k6665JxpSC7TPRKE0W4lqJ6yWdSxLNMIjI\/LHOVcXlynSzIztY3OypSUfTqwtWdmmsMnWzXN89xLJ+n+J8fp31466TzNo0QKR1E+3Hg2XfGBU0flS0z1mLKyTeeTrW3ZRU8PAePRw+GD60xNNzU1Ufg2atO8LumK6nQXFr7zoLPyTfQde2CbLVt1foma4hk6\/jr964tvOE\/xqLdRa1EurDkj2NlTaQIlzStot2mE3D8v6liWSQTG3Q3lzVlUp5eXYnRp6KzkvAC28mx\/Ou\/s6bNK\/u67E\/T333+JpqbcdvKEBQoLlU8sq6MPnFs1\/SJ5meaRI0uEditHt6hjGQRMQpspqtPlvibmltFZmbOyTQm2tsSi03MEpeHsecE246fH3yxpekdFS3iKZ9E5VcFiWXU2ivrMT60r1wrarZzvijqWQcBAwMh9axIso7OSww62yWzV2hNOxdGTr33vpSCD6wJZXYCwQHn\/uVW06Zp3jsKHQDFr62i3ZpxcUkHAuFCr8DxFdXoe3ILOSs4Lc5mtfv7JS2\/8hB5+6uVg7ckLr5y69dj1UVM4y993Jl30nim69r9cQD96k+jKi04tlsVTOoG53G5Lp5dsoahjGSIwiMBIf3ci7aOzksNeRLbhhbG8KPa7I2\/Q0ZdL27kTjp6wUOEbj\/Xf6+tPishWriXaWQZbO142qSFgbGgVJG1RnZ4H96CzkvNCpbJVu3bOe\/cv0V2PHPMiTpQQ4akdFiJXX3pOcOQ9Py7Rk0plK9fa\/FkGW38sw5aKOpblJgJz8OBB2rp1a8CdrxR4\/vnnaXBwkLZv305VVe+sxg87JnyP0vr164nPl4l7lCP5c96W3dvbS9XV1ZHJi+p0ua+JuWV0VuasbFPmja26HJAFxL+MvE7\/+IMTNPyj0hfF6lESvjRwyQVn0SW175wiq39uyzAufd7Y+nqvPNgBWzkvFHUsy4WA4TuRRkdHqbOzk2688cZAgLC44Ase6+vrEwUJ5+WH86jzZVpbW2nNmjWzWoO66Xrnzp3BCcAsmpJEUlGdLvc1MbeMzsqclW3KcrLV15y8+NokfeOHJ7ysOdHFBwufpoXvpk9duTCwraZ0sthaXE62tn6v9PRgK+fBoo5lmQsYJTpYgCxevJg6OjoCMcICQ10fkBQlCbtcFzThz1iwHD16NFEQ6XmK6nS5r4m5ZXRW5qxsU\/pgq685YaHQNzhC\/\/z8a0FVStmto95FiQ+OnKxsPJvqF5ya4pGImtjyS0rvg63P+hTJFtjKebOoY1mhBIwuhlgA6Y+aalqxYkVkdCaq6RTV6XJfE3PL6KzMWdmmTGIbFiZPHHvN204dXZyoNScfuuhsWnHhgunISRZRE1t+EDA+iZnbQp9gzso2ZVHHsswFDDtCTeXoU0gqGhM3HRQVedmzZw\/F3W6tBMyqVavovvvuo6GhIeM1MBs2bKDly5dPF1lXV0f8g8edAHdWY2NjVFtbm7jGyb2EuZVz9LW3qf6s04n\/O\/ziq\/SLk6\/So6On0xMjJ71FTdg+P+e\/+3RaXHsG\/cr5Z9LFtWf8R9Tk1Dq1Shcoaa0G7TaNkPvnYOvOLpyT+1b+Uc\/IyAht3rw5WF8a\/uPeX6nlt5QLAcOvrS+uVRi2bdtmHC1RedR6mvDiXyVgjh07Nr1wNy6tshVVJ\/6svb2d1q5dW35vFajEyclJmpiYoJqaGpo3b16B3szvq+jC5MXX3w7Eww8mJumfnjsZiBX+Hf+31EcXJxfX\/BL9+gdOCRMujx\/1eanlVHp+tFs5D4KtP7Z79+6l\/v7+WQYhYPwxFrHEC3U5ktPT00ONjY3TZURNIcWlDQuYHTt2UENDw7QtRGBKdx37Y3x8PIhkzZ9\/au3DXHrUVM6pHTon6YLqKnrlzZ\/SXz02Emwf9iVMVERETelcvug9tOjsU9uI+TMIE7tWN9fbrR0tu9Rga8crKXU4AnPo0CHavXs3IjD+EMtYSlr4yxGXRYsWTUd1WMDccccdtGvXrsit1EWdN5Qhb2e1qPPdM4XJqR0zP\/3ZFN39jWP0DB\/d6mkRrBIg\/F9eCHvp+WfS6v9cE6w1Oe8MCqJbyy5+75wUh3Yt0S51UdutHQWZ1GArw5WtFnUsy3wKSS285TUpSU\/c+S76riMVZYnbeh0WN0k7lorsdLmvibnlSu2s9DNNfj41Rfd84wX6wdiPRYQJR01+teFM+qMPv3f6GHwWRGnrTCqVrXmZaGlUAAAgAElEQVTryS4l2MqxB1s5thAwcmyDRbwHDhyYcaicfqbL6tWrY8+ECR9kpy\/iVZ+1tbVNL1zS17XELfhVr1pUpwu60th03jorJUz4BRaePY\/+\/NEX6F9flBMmH6g5g2789ffS+Gs\/CZiZCBNTuHlja1rvSkgHtnJeAls5tkUdy3ITgVFnv+gu1CMmTz\/9NHHE5IEHHpDzcshyUZ1eNoAJBZWrswoLk3sffYGeEhQmi86pok9e2UCvv\/Uz78LE1G\/lYmtanyKlA1s5b4KtHNuijmUQMAltpqhOl\/uamFv20Vmpu3M4YiItTN53ThW1N59Pk29PZSZMTOn6YGta1lxLB7ZyHgdbObZFHcsyFzDssrQpJL4WQJ0Vwyupy\/UU1enl4pdUTtpha2ox7F9\/e5QeP\/rvgakXXuEdO2+VXH19Zw5HTNaGhAkXkLbOpORKCBrAQCAHF2zBVo6AnOWijmW5EDDstqgzV9SedRYvd911F\/X19c3YGi3n7lOWi+p0aW5x9vUFsIefe5n+z5EXafSNqWDLsG9hwgLk96+or4iIiW9\/YJD1TfQde2ALtnIE5CwXdSzLjYCRc5275aI63Z1Iek6e1mlYMI\/u\/sYL9PT4j71ETcJnmbT92vlEU+9ESSo5WpJO1D4FBll7ZqY5wNaUlH06sLVnZpqjqGMZBExCCyiq000bfTidfvPwY8+8ElzqV+q0ji5Orm2qoUvqzqyIS\/1cGZYjHwYCOcpgC7ZyBOQsF3Usy4WA4QPl1q1bR6Ojo7M82NTUNGN7tZyLZ1suqtNNGHIkZYqm6MDjY8H0jssNxPqNw8suOIuuvvScwNaVFy0gDAQmXnBLA7Zu3Exyga0JJbc0YOvGzSRXUceyzAWMfsS\/Ou+Fz21RlzlGba82cZiPNEV1us5GRVX4GPvPfvUZK6GiBMpHLq6mVb9yTnC5n3rSpnXQWfloodE2wBZs5QjIWUa7lWNb1LEscwGjDqxTQkU\/7p+h79+\/n8IXM8q5eablIjpdLaRVU0AmkRUWI3xc\/bL3nUWLz\/tlL4euobOSa8VgC7ZyBOQso93KsS3iWMa0cidgeMfR0aNHiQVN0r1Gcq5+x3JRnK5ES\/dDzyVGWFio8PH1HFH5+NLaAERaJMXVD+isXMml5wPbdEauKcDWlVx6PrBNZ+SaoihjWfj9MxcwXCH9TiJdtDz88MM0ODiICIxjq+V1LEmihcXJVZdU0+9cVuslqmJTTXRWNrTs0oKtHS+b1GBrQ8suLdja8bJJDQFjQ8syrb4Ohg+tY0GzZ88e4ksZy332i171SnQ6R1tYtOx\/fGyWF9RU0JZV7xeNrpi4H52VCSW3NGDrxs0kF9iaUHJLA7Zu3ExyVeJYZvJeuYjAmFQ0izSV5HQWLjfsf2rWFJEuWqSmg1x8g87KhZpZHrA14+SSCmxdqJnlAVszTi6pKmkss3m\/zAVMeBFvOALC0Zje3l6qrq62eS8vaSvB6UnC5cHrl4itYSkVMDqrUgnG5wdbsJUjIGcZ7VaObSWMZS5vDwGTQC3PTmfh8o8\/OEE3f+WHM96Aoyx5Fi6qsuisXL6uZnnA1oyTSyqwdaFmlgdszTi5pMrzWObyPipPZgKGdxtt3bo1te7r168PdiRl8eTV6SxeVt\/75Iz7gypFuEDAyLdkDARyjMEWbOUIyFnO61hW6htnJmBUxZOmkEp9uVLz583pajs0ixf1sHC5u\/XS4HTbSnowEMh5C2zBVo6AnGW0Wzm2eRvLfL1p5gLG14tI2MmT01m87H\/8Rep+6Oj0q97Vegn9Hl9sWIEPOis5p4Et2MoRkLOMdivHNk9jmc+3hIBJoJkXp4enjDjqwluh2y6v89kWymoLnZUcbrAFWzkCcpbRbuXY5mUs8\/2GmQgYNW00NDSU+j5z\/TLHKPFSCYt00xyLziqNkPvnYOvOLi0n2KYRcv8cbN3ZpeWEgEkjVMDPs3Z6lHg58tkrCkEanZWcG8EWbOUIyFlGu5Vjm\/VYJvVmmURgfL6MOsV3YGAgMGu6a2l4eJg6Ozupp6eHGhsbI6uUpdOLLF4YNjorn9+CmbbAFmzlCMhZRruVY5vlWCb3Vjm4zFG9XNS26m3bthFfLZD06Pcoqamp1tbWxHxK9Bw+fDjxqoKsnB4+nI7XvBQl8qJ8ic5K7msNtmArR0DOMtqtHNusxjK5NzplORcRGBYvBw4cmHHirqkYCQPSBU0cPHVhJH+exwjMnf\/wPN3+tX8Lql9E8YIIjOzXGgOBHF+wBVs5AnKWIWCE2Pq8SsDkTBlOc9ttt1FbW1twaaSJgNmwYQMtX758mkBdXR3xj8Tz5ScnaMOXnwlM1591Ot3\/h7+a2+sASnl\/HgjGxsaotraWqqqqSjGFvCECYCvXJMAWbOUI+LPMfSv\/qGdkZIQ2b95M+\/bto+bmZn8FZWwp8wiMLwGjbrBuaWmh7du3xw6KHO3hZ+nSpcZrYMI+am9vp7Vr13p33ehrb1NL\/\/Fp8fI\/rz6XljXM915OHgxOTk7SxMQE1dTU0Lx58\/JQpcLUAWzlXAm2YCtHwJ\/lvXv3Un9\/\/yyDEDD+GE9b8j2FNDo6GilieOEuO\/WWW26h48ePGwuYHTt2UENDw3R9JSIwvO5lw5eH6dDzbwTl7P7ERfSJJTUCtPNhktchjY+PB5Gs+fOLKdKyIg22cuTBFmzlCPizHI7AHDp0iHbv3o0IjD\/EMy25LuIN1ydpdxFHaVauXBmE0PK2C6n7oeemT9n94IULaOCGJVKoc2EXawnk3AC2YCtHQM4y2q0cW6yBkWPr1bJaoNvb20vV1dXTtpMOz4sLq5XT6dU3PxLUtdIuZXR1HjorV3Lp+cA2nZFrCrB1JZeeD2zTGbmmKOdY5lpHl3y5WQOTtvU57uX0XUdqe3R9fX3qDdZ5isC03PMkfevZV4NX5O3SLGKK\/qCzkvMw2IKtHAE5y2i3cmwhYOTYUnj6yGahUfggO30Rr\/qMdxyFV17nRcDsf3yMbtj\/VEB3LkwdqWaEzkruCwW2YCtHQM4y2q0cWwgYObYzLKvdRPxLjqT09fXFnpQrXaVyOF1FX+bK1BEEjHSrxSnHkoQxyMrRBVs5tuUYy+RqH2858ymkpJdmMcPgw+tZygVK2um88+iy278dvM6nP3IB3dZyYbleLfNy0FnJuQBswVaOgJxltFs5ttJjmVzNky3nTsDoEZgsb6JmbJJO1+86mmvRF2aLzkruKw+2YCtHQM4y2q0cW8mxTK7W6ZZzIWDyNG2kI5N0+mPPvEqr730yKO7OT1xMv39Ffbq3CpQCnZWcM8EWbOUIyFlGu5VjKzmWydU63XLmAsbk+P\/015BJIeV0\/bLGuRh9QQRGpr0qqxgI5PiCLdjKEZCzLDWWydXYzHLmAsasmtmkknK6vvOo+2ON9KkrF2bzghmWioFADj7Ygq0cATnLaLdybKXGMrkam1mGgEngJOX0ubrzSEeNzsrsC+qSCmxdqJnlAVszTi6pwNaFmlkeqbHMrHS5VBAwZRYw+tqXKy9cQA8W\/MqAOLzorOS+1GALtnIE5Cyj3cqxhYCRY5tbyxJOn4un7kY5GJ2VXLMHW7CVIyBnGe1Wjq3EWCZXW3PLmUdgkhbxxt1rZP56paX07XT93BdevMvXBszVB52VnOfBFmzlCMhZRruVY+t7LJOrqZ1lCJgyTiHpAmau3HmEKSS7L6SP1BgIfFCMtgG2YCtHQM4yBIxntuH7j+LMr1+\/PvViRs9Vmzbn2+n64t25HH1hwBgIpFot2MqRBVuwlSQgZ9v3WCZXUzvLuY7A2L2K\/9Q+na5HX+bSpY2IwPhvl2kWIQ7TCLl\/Drbu7NJygm0aIffPfY5l7rXwnzNzAeP\/lfxZ9On07oeeo+6HjgaVe\/D6JXTlRQv8VbQCLaGzknMa2IKtHAE5y2i3cmx9jmVytbS3DAGTwMyn0zF9NBM0Oiv7L6tpDrA1JWWfDmztmZnmAFtTUvbpfI5l9qXL5chEwOg7jxYvXkwdHR00NDQU+ZZZXujoy+n69FHb5XV0T9ulch6tEMvorOQcBbZgK0dAzjLarRxbX2OZXA3dLGciYNyqWv5cvpyuTx\/N9d1HyovorOTaM9iCrRwBOctot3JsfY1lcjV0swwBU4YpJEwfzYaMzsrtC2uSC2xNKLmlAVs3bia5wNaEklsaCBg3bqm51HRSUaeQMH0U3QTQWaV+NZwTgK0zutSMYJuKyDkB2DqjS80IAZOKyG8CFjYbN26kz3zmM9TY2OjXuKE1H07X7z7C7qN3wKOzMmyEDsnA1gGaYRawNQTlkAxsHaAZZvExlhkWVdZkuZ5CYuj79++n7du3U1VVVSSYkydPUldXFw0MDASfJx18F472tLS0JNr24XRMHyECU9ZvNA4JFMWNQVYOL9jKsfUxlsnVzt1y7gVMd3c39fb2UnV1deRb8uf8bNmyhZRAaW1tpTVr1sxIr4TOihUrgs\/Uv+vr62NP+i3V6Ti8Lr5horNy\/9Km5QTbNELun4OtO7u0nGCbRsj981LHMveSZXPmWsCwOBkdHU2MkoTx6IImDR1fZzA4OBhrv1Sn6wKGt07zFmo8pwigs5JrCWALtnIE5Cyj3cqxLXUsk6tZaZYzFzBJi3g5OtLX12e8BibpZusoTKYCZsOGDbR8+fJpE3V1dcQ\/ac8nep+ibz37apDsO5uWEN9AjecdATM2Nka1tbWx04Ng5UaABwKwdWOXlgts0wi5fw627uzCOfn7zz\/qGRkZoc2bN9O+ffuoubnZX0EZW8pcwPD7x03vqOkeE0YcedmzZw+lrWtRtpKmm1QapVrD5be3t9PatWtTq7XsrlNXB9SfdToNtC9MTT+XEkxOTtLExATV1NTQvHnz5tKri78r2MohBluwlSPgz\/LevXupv79\/lkEIGH+Mpy1FTRWZCIyoqphMOynBxPmTFggrAbNjxw5qaGiYLs4kAsPTR807nwzybLxqIW286r0C5CrXJPtgfHw8iGTNn4\/IlE9Pgq1PmjNtgS3YyhHwZzkcgTl06BDt3r0bERh\/iE9ZSpr2MdmFFK7P8PAwdXZ2Uk9PT+TUk6l4YbulzBvi8sbkloL5bt\/fpHfsgS3YyhGQs4x2K8e2lLFMrlalW858CilNwKTtQgojYEfF5THZeaTbK8Xp2D4NAVP619PNAgYCN24mucDWhJJbGrB142aSq5SxzMR+VmkyFzDh9S86iLRFtpxW33WUJlBMppd8CBhsn05vzuis0hm5pgBbV3Lp+cA2nZFrCrB1JZeeDwImnZFzCoa7adOmGTuOeCpo3bp1tHPnzsRV0+GD7PRFvOqztrY2irv1Oum2a1en66fvYvt0dLNAZ+X8dUnNCLapiJwTgK0zutSMYJuKyDmB61jmXGCZMmYegVHvGbXjJ+sV065Ox\/qX9NaLziqdkWsKsHUll54PbNMZuaYAW1dy6flcx7J0y9mmyI2AyRZDdOmuTsf6l3RvorNKZ+SaAmxdyaXnA9t0Rq4pwNaVXHo+17Es3XK2KTIXMPo0T94O2HF1evXNjwRe\/eCFC2jghiXZejinpaOzknMM2IKtHAE5y2i3cmxdxzK5GvmxnLmAsT09189rm1lxcbq+gLev\/T\/RtU3nmRU2x1Khs5JzONiCrRwBOctot3JsXcYyudr4s5y5gOFXMdlt5O+VzS25OF1fwPvg9UvoyosWmBc4h1Kis5JzNtiCrRwBOctot3JsXcYyudr4s5y5gEm6C4lfM2mXkD8M0ZZcnK7Wv7DFI5+9AvcfxTgJnZVc6wVbsJUjIGcZ7VaOrctYJlcbf5YzFzD+XsW\/JRenYwGvmR\/QWZlxckkFti7UzPKArRknl1Rg60LNLI\/LWGZmOdtUEDAJ\/G2djgPszBszOitzVrYpwdaWmHl6sDVnZZsSbG2Jmae3HcvMLWebMhMBoy\/cjTtgTmGppCkkXcBsWbWItqx6f7bezXHp6KzknAO2YCtHQM4y2q0cWwgYOba5tWzrdP0AO6x\/SXYrOiu5Zg+2YCtHQM4y2q0cW9uxTK4mfi1nEoEJv0L4PqSk+5H8vn6yNVun6wt4T9z5kXJWteLKQmcl5zKwBVs5AnKW0W7l2NqOZXI18Ws5FwIm6pJFNc3U2tpKa9as8fvWhtZsnX7Z7d8mnka6oHp+sAMJTzwBdFZyrQNswVaOgJxltFs5trZjmVxN\/FrOXMAkHWTH0Pfv30\/bt2+nqqoqv29uYM3G6VjAawBUS4LOyo6XTWqwtaFllxZs7XjZpAZbG1p2aW3GMjvL2abOvYDh6Exvby9VV1eXnZSN03EDtZ170FnZ8bJJDbY2tOzSgq0dL5vUYGtDyy6tzVhmZznb1JkLmKT1Llmf0GvjdNxAbdeQ0VnZ8bJJDbY2tOzSgq0dL5vUYGtDyy6tzVhmZznb1JkLGH59hrtp0ybq6+ujxsbGgMjw8DCtW7eOdu7cSVld8mjjdCzgtWvI6KzseNmkBlsbWnZpwdaOl01qsLWhZZfWZiyzs5xt6lwIGCVirrvuuhk09u3bl5l40etkUg+cwGvXkNFZ2fGySQ22NrTs0oKtHS+b1GBrQ8suLQSMHa9CpLZxevXNjwTv\/MELF9DADUsK8f6SL4HOSo4u2IKtHAE5y2i3cmxtxjK5Wvi3nJsIjP9XK92iqdOxgNeeNTore2amOcDWlJR9OrC1Z2aaA2xNSdmnMx3L7C1nmwMCJoG\/qdN1AfPg9UvoyosWZOvVCigdnZWck8AWbOUIyFlGu5VjazqWydVAxvKcEjBqx9PAwEBAc\/369bRly5ZYsqZO\/+tvj9LNX\/5hYAcCxqyhorMy4+SSCmxdqJnlAVszTi6pwNaFmlke07HMzFp+Us0pAcNnyvDDosXkpF9Tp2MBr32DRmdlz8w0B9iakrJPB7b2zExzgK0pKft0pmOZveVsc8wpARNGrQuaKDeYOl1dIYAFvOaNGZ2VOSvblGBrS8w8Pdias7JNCba2xMzTm45l5hbzkTIXAkad+TI6OjqLSlNTk8hJvElXGKhKmDhdv0LgNy6upq+sb8qHZ3NeC3RWcg4CW7CVIyBnGe1Wjq3JWCZXupzlzAWMWpdSX1+fuB7FJwKOvOzZs4daWloS71lSTt+wYQMtX758ugp1dXXEP\/x85+gb9LG\/\/G7w\/xuvWkgbr3qvz6oW1hZ3VmNjY1RbW5vJPVeFBUtEYCvnXbAFWzkC\/ixz38o\/6hkZGaHNmzeTyZlm\/mohbylzAWMSCZHCEHULtl6WEjDh8tvb22nt2rXBrw+PvEV\/eP+phvKXH6ujZQ3zpapbKLuTk5M0MTFBNTU1NG\/evEK9W9YvA7ZyHgBbsJUj4M\/y3r17qb+\/f5ZBCBh\/jANLKgLT1tZW9lN3eeqqs7OTenp6pq8wiBIwO3bsoIaGhumP9AjMrq+\/QLu+fjz47DubltAF1RAwJk2E\/T4+Ph5EsubPBzMTZqZpwNaUlH06sLVnZpoDbE1JpacLR2AOHTpEu3fvRgQmHZ19Co50ZHHrdFq5JvOG2IFk72\/OgfluN24mucDWhJJbGrB142aSC2xNKLmlMRnL3Cxnmys3U0hDQ0ORJHwu4tV3HZmsvTFxutqBxJGXI5+9IltvVlDp6KzknAW2YCtHQM4y2q0cW5OxTK50OcuZCxi5V5ttOXyQneki3rh5Q30HErZQ23kSnZUdL5vUYGtDyy4t2NrxskkNtja07NJCwNjxKkTqNKfrVwhsWbWItqx6fyHeuxwvgc5KjjLYgq0cATnLaLdybNPGMrmSZS3nJgJz8OBB2rp1a\/C2HPF4\/vnnaXBwMHGbsywaojSn6wKGp4+wgNfcI+iszFnZpgRbW2Lm6cHWnJVtSrC1JWaePm0sM7eUr5S5EDBqOzPvCLrxxhuD82B47UtXVxeV83yYsGvSnN790HPU\/dDRIBvuQLJr2Ois7HjZpAZbG1p2acHWjpdNarC1oWWXNm0ss7OWn9SZCxj9HJjFixdTR0dHIGCam5uDCEgWu5OUe9Kc\/t\/2DNEjPzwRJD9x50fy49UKqAk6KzkngS3YyhGQs4x2K8c2bSyTK1nWMgRMAt80p2MLtXvjRGflzi4tJ9imEXL\/HGzd2aXlBNs0Qu6fp41l7pazzZm5gOHX5\/UvvN5Fn0JS0ZjW1lZas2ZNJpTSnF598yNBvbADyd496KzsmZnmAFtTUvbpwNaemWkOsDUlZZ8ubSyzt5iPHLkQMIwi6tj+bdu2ZSZe9DrFbaNWAqbt8jq6p+3SfHi0QmqBzkrOUWALtnIE5Cyj3cqxhYCRY5tby0lOxxbq0tyGzqo0fkm5wRZs5QjIWUa7lWMLASPHNreWTQUMdiDZuxCdlT0z0xxga0rKPh3Y2jMzzQG2pqTs00HA2DOzyqGfA6MyZn1zZpLTsYXayr2zEqOzKo0fIjBy\/MAWbLMhIFcqBIwc22AR74EDB6i3t5eqq6uDktT26rwu4r1h\/1O0\/\/GxU3XFFmrr1gEBY43MOAPYGqOyTgi21siMM4CtMSrrhBAw1sjMMujnwPDZL\/qT53NgsIXazL9xqdBZlcYPUQI5fmALttkQkCsVAkaIbaUKGHULNbZQuzUMCBg3bia5wNaEklsasHXjZpILbE0ouaWBgHHjZpSL4W7atIn6+vqosbExyJPnKSTcQm3k1sRE6KxKZ4jolhxDsAXb8hOQKxECRoitEipDQ0OpJfD9SA888EBqOl8J4pyuCxjcQu1GGwLGjZtJLrA1oeSWBmzduJnkAlsTSm5pIGDcuFV0rjin62fA8AF2fJAdHjsC6KzseNmkBlsbWnZpwdaOl01qsLWhZZcWAsaOVyFSmwgYnAHj5mp0Vm7cTHKBrQkltzRg68bNJBfYmlBySwMB48bNOFfUOTB5vUpAPwPmyGevoAuq5xu\/JxKeIoDOSq4lgC3YyhGQs4x2K8cWAkaObcWdA6O2UDMSnAHj1jDQWblxM8kFtiaU3NKArRs3k1xga0LJLQ0EjBu31FyVuI0aZ8CkujU1ATqrVETOCcDWGV1qRrBNReScAGyd0aVmhIBJReSWoFQBE97F1NLSQtu3b6eqqqrICulTVWlp45yubqHGGTBuPudc6Kzc2aXlBNs0Qu6fg607u7ScYJtGyP1zCBh3dqk5Xa8SOHnyJHV1ddGKFStozZo1pP5dX19PW7ZsmVWufrIvCxzOG5eWM0c5Xd9CzbuPeBcSHnsC6KzsmZnmAFtTUvbpwNaemWkOsDUlZZ8OAsaemVUOX4t42c7g4GBkFCb8WVJaEwGDM2CsXDwjMTord3ZpOcE2jZD752Drzi4tJ9imEXL\/HALGnV1ZcyaJkqgIjIreRFUyLQKDM2DcXYvOyp1dWk6wTSPk\/jnYurNLywm2aYTcP4eAcWdXtpwm1w8MDw\/TunXraHR0lPbt20fhCyT1yiqnb9iwgZYvXx589Dfff5v+5vs\/Df7\/S39wCV150dlle78iFcSd1djYGNXW1sauVyrS+5bzXcBWjjbYgq0cAX+WuW\/lH\/WMjIzQ5s2bU8c8fzUoj6XTpqampspTlGwpav0LlxK3iDe81qa7uzuoVNR6Gf69EjB6zd+6ZDW9dcm1wa8G2hdS\/Vmny75YQa1PTk7SxMQE1dTU0Lx58wr6ltm8FtjKcQdbsJUj4M\/y3r17qb+\/f5bBtD\/a\/dWgPJYKIWBMxEt4wS\/j5WhMZ2cn9fT0TF8iqWNXAmbHjh3U0NAQfHT9gxN09K0zg\/8fveOK8nipgKWwP8bHx6muro7mz8dBgD5dDLY+ac60BbZgK0fAn+VwBObQoUO0e\/duRGD8IfZjKW3nkSqlFAGjq1acAePHb5jv9sMxygrYgq0cATnLaLdybLEGRo5tSZZ5GojXsySd\/aIKiJpCSsob5fTLbv828VZqnAFTkttwDkxp+BJzYyCQgwu2YCtHQM4yBIwcW2fL4UPslKGmpibq7e0NFofyWS9tbW3Ti3VZ8OzZsydI6nKQHQ6xc3bXjIwYCPxwRARGjiPYgm15CciVBgEjxza3lsNO1w+xwxkwpbkNAqY0fkm5wRZs5QjIWUa7lWMLASPHNreWw05\/7JlXafW9Twb1ffD6JXTlRQtyW\/e8VwydlZyHwBZs5QjIWUa7lWMLASPHNreWIWDkXIPOCmzlCMhZRrsFWzkCcpYhYOTY5tZy2OndDz1H3Q8dRQTGg8cwEHiAGGMCbMFWjoCcZbRbObYQMHJsc2s5ScAc+ewVdEE1zi9xdR46K1dy6fnANp2RawqwdSWXng9s0xm5poCAcSVXwfnCTldnwPArnbjzIxX8ZtlXHZ2VnA\/AFmzlCMhZRruVYwsBI8c2t5bjBAxHXjgCg8edADord3ZpOcE2jZD752Drzi4tJ9imEXL\/HALGnV3F5gw7HYfY+XMlOit\/LMOWwBZs5QjIWUa7lWMLASPHNreWw07HIXb+XIXOyh9LCBg5lmALtuUjIFcSBIwc29xa1p1ev\/gy4ggMP\/\/jqvfRrb\/9gdzWuxIqBgEj5yWwBVs5AnKW0W7l2ELAyLHNrWXd6W+fe8n0IXY4hbd0l6GzKp1hnAWwBVs5AnKW0W7l2ELAyLHNreU4AXNP26XUdnldbutdCRVDZyXnJbAFWzkCcpbRbuXYQsDIsc2tZd3pz71rEd2w\/6mgrrhGoHSXobMqnSEiMHIMwRZsy09ArkQIGDm2ubUMASPnGggYsJUjIGcZ7RZs5QjIWYaAkWObW8u60\/\/Xc++h\/Y+PBXXFKbyluwwDQekMESWQYwi2YFt+AnIlQsDIsc2tZd3pXzg8j7717KvB9QE4xK50l0HAlM4Qg6wcQ7AF2\/ITkCsRAkaObW4tQ8DIuQYCBmzlCMhZRrsFWzkCcpYhYOTY5tay7vQ\/+ocpOnbiLfrghQto4IYlua1zpVQMA4Gcp8AWbOUIyFlGu5VjCwEjxza3lnWn\/9aXTgb1hIDx42oqtNUAABBdSURBVC50Vn44RlkBW7CVIyBnGe1Wji0EjBzb3FpWTu++54u0\/uGpoJ58\/gufA4OnNALorErjl5QbbMFWjoCcZbRbObYQMHJsc2s5SsDgFF4\/7kJn5YcjIjByHMEWbMtLQK40CBg5tiVZPnHiBHV0dNDQ0FBgp6WlhbZv305VVVWRdpUj+cOmpibq7e2l6urqxLQ3fP5e4l1I\/OAU3pLcNZ0ZAsYPRwyychzBFmzLS0CuNAgYObbOlk+ePEldXV20YsUKWrNmDal\/19fX05YtW2bZHR4epnXr1tHOnTupubmZDh48SIODg7GCRzldFzA4A8bZXTMyQsD44YhBVo4j2IJteQnIlQYBI8fWq+UkUcKfHT16NFLcRFVCOb2p\/Q569N9rgyS4RsCPuyBg\/HDEICvHEWzBtrwE5EqDgJFj69VynIAJR2tMClVOP\/fq6+mZM5cFWfZccxpdfukiqqvDZY4mDOPSsIAZGxuj2tra2Om+UuzP5bxgK+d9sAVbOQL+LHPfyj\/qGRkZoc2bN9O+ffuC2YeiPKdNTU2d2l5TgEeth2ltbQ2mlPRHCZhVq1bRfffdF6yZMV0D8+bSP6CfXPDBwNyCv+2g9vZ2Wrt2bQGIZfcKk5OTNDExQTU1NTRv3qn1RXj8EABbPxyjrIAt2MoR8Gd579691N\/fP8sgBIw\/xl4tKYHCRqMW8arPjx07Nr1wt7u7m0ZHR1PXwLxxZSe9fe7FwTUCf3H1aUH0BRGY0tzH\/hgfHw84zp8\/vzRjyD1LrIOtTKNAu5XhylbB1h\/bcATm0KFDtHv3bkRg\/CH2ZylNvKgvh77gl3\/Hi3o7Ozupp6eHGhsbZ1VITSHpAgb3IPnxG9bA+OEYZQVswVaOgJxltFs5tlgDI8e2JMtpO4904xxxWbRo0fT0EguYO+64g3bt2hW5lVo5\/bWPdtPPzzgXp\/CW5KmZmdFZeYQZMgW2YCtHQM4y2q0cWwgYObYlWU6bBtKNsxM5vTr7hf+fn6gt1\/x75fRX\/2tvkA7XCJTkqhmZ0Vn5Yxm2BLZgK0dAzjLarRxbCBg5ts6Ww4fYKUNqcS4fZsfTRm1tbdMrr\/WD7EwOvWv95E3EEZhA6KxaRFtWvd+5vsj4DgF0VnKtAWzBVo6AnGW0Wzm2EDBybHNrmZ0OASPjHnRWMlzZKtiCrRwBOctot3JsIWDk2ObWMjv9d2+6jXgRLz+4RsCfq9BZ+WMZtgS2YCtHQM4y2q0cWwgYOba5tcxO\/3jXPcTnwPCDU3j9uQqdlT+WEDByLMEWbMtHQK4kCBg5trm1HF7ECwHjz1UQMP5YYpCVYwm2YFs+AnIlQcDIsc2t5bCAwUWO\/lwFAeOPJQZZOZZgC7blIyBXEgSMHNvcWmant7V30L\/\/9t1BHU\/c+ZHc1rXSKgYBI+cxsAVbOQJyltFu5dhCwMixza1lfQ0MXyOAU3j9uQqdlT+WiBLIsQRbsC0fAbmSIGDk2ObWsj6FBAHj100QMH556tbAFmzlCMhZRruVYwsBI8c2t5Z1AYNTeP26CZ2VX54QMHI8wRZsy0NArhQIGDm2ubUMASPnGggYsJUjIGcZ7RZs5QjIWYaAkWObW8u6gGm7vC44yA6PHwIYCPxwjLICtmArR0DOMtqtHFsIGDm2ubWsC5g\/\/d2LaW1zfW7rWmkVQ2cl5zGwBVs5AnKW0W7l2ELAyLHNrWVdwOAiR79uQmfll6duDWzBVo6AnGW0Wzm2EDBybHNrWRcwuAfJr5vQWfnlCQEjxxNswbY8BORKgYCRY5tby7qAwTUCft0EAeOXJwZZOZ5gC7blISBXCgSMHNvcWoaAkXMNBAzYyhGQs4x2C7ZyBOQsQ8DIsc2tZeX03\/zC39GWVe8nPswOjx8CGAj8cIyyArZgK0dAzjLarRxbCBg5trm1XFSn5wE4Ois5L4At2MoRkLOMdivHtqhj2WlTU1NTctgq23JRnZ4Hr6CzkvMC2IKtHAE5y2i3cmyLOpZBwCS0maI6Xe5rYm4ZnZU5K9uUYGtLzDw92Jqzsk0JtrbEzNMXdSyreAFz4sQJ6ujooKGhocCbLS0ttH37dqqqqkr07vDwMHV2dlJPTw81NjZGpi2q082bvVzKo0ePUn9\/P7W3t9OiRYvkCpqDlsFWzulgC7ZyBOQsF3Usq2gBc\/LkSerq6qIVK1bQmjVrSP27vr6etmzZEtsaVLrDhw9TX18fBIzc9ybWclG\/UBmgnFUk2Mp5AWzBVo6AnOWittuKFjBR7j548CANDg4mRmHYmd3d3UF2RGDkvjRJlov6hcqG5sxSwVbOC2ALtnIE5CwXtd3OOQHDU0633XYbtbW1BSLGRMBs2LCBli9fLte65qDlkZER2rx5M4Gtf+eDrX+myiLYgq0cATnLqt3u27ePmpub5Qoqs+VCCRi1Hqa1tTWYUoqL0PDvly5dmroG5vjx48Ege+jQoTK7BcWBAAiAAAiAgD8C\/Ef4jh07aOHChf6MZmypMAJGrWthnnGLeHnhLi8cveWWW4jFSdoiXrbF6fgHDwiAAAiAAAhUKgEWLkUSL+yHQggYE\/HCL8tTRitXrgxCaCa7kCq1oaLeIAACIAACIFB0AhUvYEx3HoW3W+uOLdq8YNEbLd4PBEAABEAABCpewHBUZXR01OjsF93diMCg8YMACIAACIBA5RKoaAETF1Vpamqi3t7e4DA7PieGdxyFV15DwFRuo0XNQQAEQAAEQKCiBQzcBwIgAAIgAAIgMDcJQMDMTb\/jrUEABEAABECgoglAwFS0+1B5EAABEAABEJibBCBgEvzOC4T37NkTpMBOJbcvCK81WrduXbDQOu2iTZ0332eVdE+VW22KlcuGrXrz8P1hxSLi521suIbX4aGfSPaBDVs9LfqD0tq2+t5HrQctzXK2uSFgYvir+5J4MfDTTz8dnCHD\/19dXZ2txyqodH2wXL169YyLN8OvEb7Div994MABMI\/xtw1b3QRz3bp1K23bti32tOoKamLeq2rDNXyEAzYGJLvDhq0ShnwpL2\/AQH\/g3tQV94GBgcL9IQ4BE9Mu1GWP\/AUqqnp1\/0qY5Qx36CwK9+\/fb7TlHYNB+l+y+knSJmx5UNi4cSO9+uqrlHTdhpl3i5nKps1y2jvuuIN27dqFP2wMmoMtW719oz8wAByRREWxli1bRseOHSMlCN2s5S8XBEyET8JhdoTd3RquHsXiyFX430lW0WElM3dhy6L88ssvp69+9au0YsUKRGAiENtwDUcN3b4lcyeXDduoCMzg4KDRHz9zh2j6m\/IljvzwkSIdHR0QMOnIKj9FVMSFO\/9Fixah07dwbzgqYPMXq+sBhRbVq+iktmzVPWA333xzcBs7BEy0+224soA5evRoYAhr5dK\/TjZs2Zo+9bF+\/fpg8MXjRiAsCN2s5C8XIjAJERh9wRMEjH3jte2wVAk8MNx1111YxJuA3IYtDwRf+MIXqL29PbjMjQ93hIDxI2B4PZFauMs+2bRpE9ptTLu1abNq6mPnzp3BGhib6K19T1X8HBAwxffx9BtiCsmPs21CxhAvdsxt2HLaRx99NPgLFtOhyZxtuIankMAWbO2+xeVLDQFTPta5KEmPuGARr5tLwlNGaQtNsdPAnLMNW317ul4CwvKzedtwDbdn9BPJ7deGLcSheV9gkhICxoRSgdJgG3XpzrTZNonwux1vG7a6ZUQJkjnbcA0PCpjm8Mc2agoJ03N2fYSeGgLGnV3F5sRBdqW7LungKrUIkqc24qIEOBgs3gembCFg7NqxDVf9IDsctpbO2YYtC8LrrrsuMAq26WyTUkDAlMYPuUEABEAABEAABEDAGwHsQvKGEoZAAARAAARAAATKRQACplykUQ4IgAAIgAAIgIA3AhAw3lDCEAiAAAiAAAiAQLkIQMCUizTKAQEQAAEQAAEQ8EYAAsYbShgCARAAARAAARAoFwEImHKRRjkg4EDg2WefpbPPPtv4tmPeLvnKK6\/QhRde6FCaWRa15b2pqYl6e3uN61YpF3Sq9yvX1t1yl2fmZaQCgfwTgIDJv49QwzlKwPZgtHKc9VCKCCklbzmbAAsKfsp5eWClsCmnH1AWCKQRgIBJI4TPQSAjAnkUMLZ10tFVyiANAZNRg0exIGBJAALGEhiSg4BPAvpJrmxXTcs8\/fTT06eQ8u\/VicThE4vVNMc555xDHR0dNDQ0FFRP3XOkjsYfGBgIfm8y7aOXoU+j8MnJfPuyerZt20Zr1qyZhSMunRIwq1evps9\/\/vNBvvA0jX76argcZrVx40b68Ic\/HORX7\/Lyyy\/TunXraHR0NMjyuc99jh588EHq6emhxsbG4Hdx7xTly7CA4X+\/\/vrrwY\/imHSPVPgeHy4j6neVKO58tn3YAoFSCUDAlEoQ+UHAkUDUvUT64BmOdsRdcMfFb9++PbhpmkUMT300NzeTEketra3TQiPpwkxVH2WvqqoqGHjvuusu6uvrC8RAWgQmnF6\/04ZFFguNZcuWBfVV9g8cOBCspWEh0tnZOUN46PaUSLvgggum84ffUf17YmIiqPPChQupq6srEEpqSijt3q0oAbNnzx5Sgi2Kq94EIGAcvxDIBgKWBCBgLIEhOQj4IpA2EKaJhfBf9mEBE3X7d9JljlFTPOH0SXVKuygyfEEf1z9tWkn\/XAmYsCAbHBycFjRsUxco\/O877riDdu3aNWOxcdI0UZSA4eiOEl2qDE4XtYgZAsbXNwR2QCCZAAQMWggIZEhAn24JT+\/EiYXwNEtLS0tkBCY8laO\/ZtT0T1x5+qWbSQImbRFxlFiJ+l14Wi08TaYiTPw+UUJEt8lRHXUhYNjNcdNAUQKG8+qLepOEFwRMhl8oFD2nCEDAzCl342XzSkAftPV1MPpf+UqQhNelqAhEOALDecORg6T3jxMnSdNaur1SBQzbUmtZlMCKisDYCJgnnniC1BRVdXW1kfshYIwwIREIZE4AAiZzF6ACIPAOAV0EqAgDT1PwehFey7FixYoZC2d1kRIWMEnrXaKYl2MKKbzGRS+TxUbSdJCaQtIFTFS0Q59C4gjMpk2bptfwmLQ1iSmkNDGZNpVmUm+kAYG5RgACZq55HO+bGwJRa2D0KIi+qDVqMaqKyKgpJH4xXeQo+7ygV01\/RK1DUUB8LeLVIx76upilS5fOWqQbFjB6XlVXrh8vyI0SMKaLeNmGWsOStvao1EW8aopP7RxT76EvXg43QgiY3HwtUZEKIgABU0HOQlWLR0ANbmoLsD49pG+B5imVa665ZsZWaRYu1157Ld16663TEYawqFFRGbW9mgmqgTWOZtKWY9OFxVHbrU3WwITL3rlzZ7DOhRfuqvfXIzD8DmGG4W3U4a3knCduC7iKevF\/leiL2kat5496L339EfvpsssuoyNHjgQiKiw01TuEo1PFa+14IxDwSwACxi9PWAMBEMiYAAuKqJ1HptUyWQNjass0HSIwpqSQDgTeIQABg9YAAiBQsQTC63xUtEU\/98X25SBgbIkhPQhkQwACJhvuKBUEQMATgfDpxEmn5JoUGb5c8f777w+ySd2NhMscTbyCNCAwmwAEDFoFCIAACIAACIBAxRH4\/wvcf6AEUL8VAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:47a778fe]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"13.2000"}}
%---
%[output:5cf529bd]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"0.0075"}}
%---
%[output:7174987d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"0.0075"}}
%---
