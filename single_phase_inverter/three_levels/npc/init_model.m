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

model = 'three_levels_single_phase_inverter';
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
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:60f0763a]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:22461dfc]
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
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:2cb67184]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:6e693ac7]
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
figure; bode(flt_dq_d, options); grid on %[output:68bfffaa]
[num, den]=tfdata(flt_dq_d,'v') %[output:926cdc68] %[output:72ee0024]

%[text] #### Resonant PI
kp_rpi = 2;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:92c8c0dd]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:8de2fd29]
a11d = 1 %[output:908585c9]
a12d = ts_inv %[output:23a9305b]
a21d = -omega_set^2*ts_inv %[output:6ea3ed09]
a22d = 1 -2*delta*omega_set*ts_inv %[output:2c12ffd6]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:80a7f48a]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:7ddfb290]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:424972f3]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:780af92e]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:108fa3e8]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:63a32b6a]

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
figure;  %[output:064f1b7e]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:064f1b7e]
xlabel('state of charge [p.u.]'); %[output:064f1b7e]
ylabel('open circuit voltage [V]'); %[output:064f1b7e]
title('open circuit voltage(state of charge)'); %[output:064f1b7e]
grid on %[output:064f1b7e]

%[text] ## System Identification
%[text] ### Normalization
Ibez = I1rms_load_trafo*sqrt(2);
Ubez = V1rms_load_trafo*sqrt(2);
Xbez = Ubez/Ibez;
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### Diode rectifier
Vf_diode_rectifier = 0.7;
Rdon_diode_rectifier = 1.85e-3;
%[text] ### HeatSink settings
heatsink_liquid_2kW; %[output:2a3c823f] %[output:31cd044d] %[output:0ead980c]
%[text] ### DEVICES settings
% danfoss_SKM1700MB20R4S2I4;

infineon_FF1800R12IE5;
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
inv_t.Csnubber = Irr^2*Lstray_module/Vdc_nom^2 %[output:09269494]
inv_t.Rsnubber = 1/(Csnubber*fPWM_INV)/5 %[output:6bec205b]

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
inv.Csnubber = Irr^2*Lstray_module/Vdc_nom^2 %[output:845c2e86]
inv.Rsnubber = 1/(Csnubber*fPWM_INV)/5 %[output:28f81047]
%[text] ### Load
uload = 3;
rload = uload/I2rms_load_trafo;
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
%   data: {"layout":"onright","rightPanelPercent":39.2}
%---
%[output:60f0763a]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"     6.600000000000000e+00"}}
%---
%[output:22461dfc]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:2cb67184]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:6e693ac7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:68bfffaa]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAm0AAAF2CAYAAAA88q97AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9sVteZ5t+Z9UzTyu6STGfqGFMl4HbKIHCjNtBJssxGSCRtplkrmRLwzhoJrAlx3aShpukiNpkighRMrKK6iLQGBVbB4G6y1mSTTRDsdLxJW5N2WLIr0U5Noa1rYGgTprZKmGamq\/fQ8+V+19+f99577r3n3PNcCWF\/33v+3N\/zXs7Dufec+zu\/+c1vfkM4QAAEQAAEQAAEQAAErCbwOzBtVuuDzoEACIAACIAACICAIgDThkQAARAAARAAARAAAQcIwLQ5IBK6CAIgAAIgAAIgAAIwbcgBEAABEAABEAABEHCAAEybAyKhiyAAAiAAAiAAAiAA04YcAAEQAAEQAAEQAAEHCMC0OSASuggCIAACIAACIAACMG3IARAAARAAARAAARBwgABMmwMioYsgAAIgAAIgAAIgANOGHAABEAABEAABEAABBwjAtDkgEroIAiAAAiAAAiAAAjBtyAEQAAEQAAEQAAEQcIAATJsDIqGLIAACIAACIAACIADThhwAARAAARAAARAAAQcIwLQ5IBK6CAKuEti9ezft3LmzrPtNTU104MABam9vj3xa58+fp1WrVtGWLVto5cqVkcpv3LiRRkdHa\/blyJEjtG3bNhoZGaHm5uZI9SMYBEAABNImANOWNmHUDwIeE2DTNjY2RkNDQ9TY2KhI8GeHDh2KZYySmjZuf2BgoKQI9+Wpp54qmUiYNo+TFacOAg4QgGlzQCR0EQRcJVDJtJ08eZJ6e3tpcHAw8mybadPGXHkGLmzmXOWNfoMACBSbAExbsfXF2YFArgSqzbSFZ9+Cty47OjrKZsN49mvDhg3qPJYuXUpTU1Ol26PaxE1OTqrv9+zZU\/W2aTVzFuzjt771rbLbo8G2uf6+vj7q6elRbc3MzFB3dzcdP36c+JYv9216elrNKnI9u3btUnHcN74dzEdXV5eK4SN4ntw3\/pz\/cH2tra30la98hT772c+q8vw7btnmmspoHASsIADTZoUM6AQIFJNApWfawgYkaJrYCPEza6tXr1bmKDyzputjc3bLLbco07R8+XIVW28Gr5ppC94Sff3110umjRVZv349bd++Xc0Ihm+dVup3S0tLybSx0dQmUhu8devWKVMZ7iv37dixY8rcLViwQJ0Xm1M2anwEmRQzU3BWIAACEgIwbRJKiAEBEIhFoNJMG5ufTZs2lRkUbby4kVozX0ET9\/73v582b95Me\/fuLS0aYPPT1tZWmg0LdjqqaQsvRAg+i8fP5wUNY71+h+EFGbAhDPct+Ls2fEFGscRAIRAAAecJwLQ5LyFOAATsJVDJtAVNyD333DNrNWiwDM88BW+lBmes+Kz1bdMggfDtVf1d1Nuj2pjx7Uo+PvzhD6tbosHZr+Aq1lpmk8sHbwHPnTtX1amf64NpszeH0TMQsIkATJtNaqAvIFAwAvVMGz\/jFWXGqt5MWy18lUybNoF8W5NXlVa7VcqzbsHvos60hW+HVro9yn3XK1sx01awCwGnAwKGCMC0GQKJakAABGYTqHd7lG8NSp5p08+46YUBlZ5p04ZOx4Z7U8m01dryg59vC9\/G1c+ZsYmr90xbcL83NmlsUPv7+9UzbcFn2HB7FFcOCICAlABMm5QU4kAABCITqLQQgSsJr\/KstXpUGx5eWblw4ULVh4ceekiZn\/Dq0Wq3RrlMpc11w4siKs2m6dWhn\/\/85+nrX\/966ZZmcPUo13P77bfTD37wg9JChPAmvcH2\/\/qv\/1rF6duruD0aObVQAAS8JADT5qXsOGkQAAHTBNigTkxMlG1XYroN1AcCIOA3AZg2v\/XH2YMACMQkEFypWu\/WbMwmUAwEQAAEygjAtCEhQAAEQCAGgeBtWy5e69ZsjOpRBARAAARmEYBpQ1KAAAiAAAiAAAiAgAMEYNocEAldBAEQAAEQAAEQAAGYNuQACIAACIAACIAACDhAAKbNAZHQRRAAARAAARAAARCAaUMOgAAIgAAIgAAIgIADBGDaHBAJXQQBEAABEAABEAABmDbkAAiAAAiAAAiAAAg4QACmzQGR0EUQAAEQAAEQAAEQgGlDDoAACIAACIAACICAAwRg2hwQCV0EARAAARAAARAAAZg25AAIgAAIgAAIgAAIOEAAps0BkdBFEAABEAABEAABEIBpQw6AAAiAAAiAAAiAgAMEYNocEAldBAEQAAEQAAEQAAGYNuQACIAACIAACIAACDhAAKbNAZHQRRAAARAAARAAARCAaUMOgAAIgAAIgAAIgIADBGDaHBAJXQQBEAABEAABEAABmDbkAAiAAAiAAAiAAAg4QACmzQGR0EUQAAEQAAEQAAEQgGlDDoAACIAACIAACICAAwRg2hwQCV0EARAAARAAARAAAZg25AAIgAAIgAAIgAAIOEAAps0BkdBFEAABEAABEAABEIBpQw6AAAiAAAiAAAiAgAMEYNoMiDQ5OWmgFlQBAiAAAiAAAiAQlUBra2vUIs7Gw7QllI4N26ZNm2h8fDxhTSgOAiAAAiBQZAK33367Or0zZ87Q2bNni3yqmZ7bsmXLqL+\/n3wwbzBtCVPrO9\/5DnV2dqqEmTt3bsLaohdns7hr1y4j7cepS1pGElcrptp3UT6X9CG6AvISJtuPU5e0jCQOWqWru0QD3YN6sdDKHq2OHj2qDNuKFSto\/vz5szoWRyuupFI56WdyOskj6+VqlBZ0Xffeey89++yzNDY2BtMWBaCvsdq05ZUwPNPHCfvQQw8lliBOXdIykrhaMdW+i\/I5tJLlCrQikjCQXnBx6opSpl4sriupUvF0r8c\/2PrQ0BCdP3+e7r77blqyZMmsjsXRiiupVK7SZ0X8N5Bn2XjiJK8xWJ5dZiIx05aQI18Ef\/HFryrT5MPUbFxc8669Jm5RY+V+9rOrt7J5VvRPF7dFrvcD1+V\/DpE77WiBvAcXR7Hl0m1oJcf+zDPPqODFixdXNG3ymuJFFlGrIp5TLXVh2uLlfqkUJ8wnRy4nrAXFi0KglrELG9dwbKWyH7ju3QpNsKyOK7KJ5Od9nn76adqyZQs1NDQUJT0KeR7QSi5r3qatiFrBtMnzD5FE5FvCxBX9J2+8FbeosXJvvfUWXbx4ka6\/\/nplBKL26adv1j+Hn7xR3cCH26vUfrCNKP0LGjg2eGFjx+ZPGz\/+znbDx1qdO3eO5s2bB9Nm7ApIpyJoJeeat2krola+jcGYaatzve3evZt27typovr6+qinp6eshG8JI\/\/nyb5Il\/\/BChq44M\/a5AXNIn+vY\/j7WuZPmzc2dLe1zVGiaYOXp7lzWSv7Mj\/dHkErOV+YNjkraaRvYzBMW43MOHnyJPX29tLg4KCK0j+3t7eXSvmWMNILycY43wcXbd7KDd3VmcFXJi6pv189ffXv4KGN3a0L5pRm6G5dcG3J5KWhte9apcE0rTqhlZwsTJuclTTStzEYpq1GZvAsG69I4RU\/jY2NtHHjRmprayubbfMtYaQXko1xGFzkqmhjxyZOz+KxsQvP3IUNHZs5PvSsnbzF8khoFZdc9uWglZw5TJuclTTStzEYpq1GZrBJ42NgYED9Hf6dP9MJc\/DgwdLq0ebmZmm+IS5DAhhczMHWt2BfPf2mqpQN3fiPZ8oaYEO35uar1wKbuY\/f0CjuALQSo8o9EFrJJRgeHlbBixYtymX1aFG04m1T9MFbm2DLD3kOFjoyPLPGM28TExMlExc0bUEQXV1dtHbt2kKzcfHkrly5ohYisKnGisR0FJz65duq4udPXTVw3\/vZW+qPPlre20AfnXsNXd\/UQB9rvUb9XOmAVunok0at0EpOlTfX5YM31q20ua68pniRRdFq\/\/79dODAgTII2KctXk4UqlQU0xZ8IwKbAsy22ZcKly9fpgsXLqgZUZi2bPVhM8czcXyrlWflgs\/OsZH79E3vU6tbO5derzoGrbLVJ0lr0EpOb2RkRAXzTBv\/yfooilY806Zn2\/SbEWDass4mC9uLcnvUl4SxUCZxl4pya0B8wpYH8i3W4dfOqV4+8fI772Hk26ps4qanZ2jbXyyGwbZcR1xXcoHwTJuclTQSz7RJSXkQF74dioUIbouOwcVu\/YLPyX3z+z8vPSOnn43jrUj0M3J2n4lfvcN1Jdcbpk3OShoJ0yYl5UEctvwolsgYXNzRk7X63g9+Sr9+17U0\/uPpsluq2sSlve2IO7Ty7SmuKzl\/mDY5K2kkTJuUlCdx2Fy3OEJjcHFHy0paVbqdygZObwz8yB03unOCBeopriu5mDBtclbSSJg2KakM4mZmZqi7u5uOHz8+q7WlS5eW9k\/LoCtVm\/AtYfJknbRtDC5JCWZXXqKVNnHBhQ16Fm7Nzddb\/6qu7Gim25JEq3R74E7tMG3mtfJtDLZynza+LcnbZvDBy3qDbyDQkh85coQ2bNhATU1NVWPMp8fsGn1LmCyYptUGBpe0yJqvN6pWbOD0RsB6UQNuo5rXpVKNUbXKpld2tgLTZl4X38Zg60wbL+PdsWMHbd26Vb2FoN7Bs3GPPvpo2d5p9cqY\/N63hDHJLuu6MLhkTTx+e0m10rNww6+dV+9exWKG+FrUK5lUq3r1F+l7mDbzavo2Bltn2sxLmm6NviVMujTTrR2DS7p8TdZuUisYOJPKzK7LpFbp9jT\/2mHazGvg2xhspWkLP8u2Z88epTTfDuXDlufZuC++JYz5Sy67GjG4ZMc6aUtpaQUDl1QZmLYkBGHaktCrXNa3MdhK0xbc1FY\/u9bR0aFugWpD19LSktst0WDq+JYw5i+57GpMywhkdwb+tJSFVtUMHLYSiZZnWWgVrUf2RsO0mdfGtzHYOtPGz7StX7+etm\/frhYgaJO2fPly6unpUYrzQoXNmzfT3r17c39dlG8JY\/6Sy65GDC7ZsU7aUtZawcDFVyxrreL3NP+SMG3mNfBtDHbGtK1bt45WrlwJ02Y+572pEYOLO1LnqVU1A4dtRCrnT55auZPRV3sK02ZeMZg280wj1Vhtpg2mLRJGBFcggMHFnbSwRStt4MLbiGAj33dyyRatXMhumDbzKsG0mWcaqUaYtki4EByBAAaXCLByDrVRKxg4zLQlvSxg2pISnF0eps0800g1smlbtWoVTU5O1izX2tpKIyMjeKYtEl2\/g200An4rUv3sbdYq\/CYG3zfxtVkr264vmDbzisC0mWda6Bp9SxiXxcTg4o56rmiF59+IXNHKhuyHaTOvgm9jsHULEcxLmm6NviVMujTTrR2DS7p8Tdbuola+3j51USuTuRqlLpi2KLRksb6NwdaZNtwelSUqoqITwOASnVleJVzX6omXz6jXZ\/FrtIr+EnvXtcoyx2HazNOGaTPPNFGNvNFuW1tbaY82rmz37t00NjZGQ0NDoveTJupAncK+JUyaLNOuG4NL2oTN1V8UrardPi3S6tOiaGUue6vXBNNmnrJvY7B1M21BScMrSfV32FzXfOL7UCMGF3dULqJWlW6fsnlbc3OzO8JU6GkRtUpLEJg282Rh2swzTVQjz7SNjo4Sv3+UN9cNv9YqUeUGCvuWMAaQ5VYFBpfc0EduuMhaFW3xQpG1ipy4dQrAtJkm6t\/7v62eaQvOrHV1ddH09DQ1NTXRgQMH1CuubDhg2mxQQdYHDC4yTjZE+aJVERYv+KKViesCps0ExfI6fBuDnTBt5mU2V6NvCWOOXPY1YXDJnnncFn3UihctvDLxpnOLF3zUKm5ew7TFJVe9nG9jMExbwhzyLWES4sq1OAaXXPFHatxnrVybffNZq0hJjXePRsUlivdtDLbOtPHigx07dtDWrVtFK0NnZmbo0UcfpYGBAZHApoN8SxjT\/LKsD4NLlrSTtQWtrvJjA8fbh9i8dQi0kuc6ZtrkrKSRvo3B1pk2FopXh\/IzbHxUe35NL0jI+xk33xJGeiHZGIfBxUZVKvcJWpVzsXnrEGglv65g2uSspJG+jcFWmjYtFs+idXd30\/Hjx2fpt3TpUuzTJs1qxCkCGFzcSQRoVV2r8O3TR+64gdbcfL3axDePA1rJqcO0yVlJI2HapKQQpwj4ljAuy47BxR31oFV9rWyZfYNW9bXSETBtclbSSN\/GYKtn2qSi5RnnW8LkyTpp2xhckhLMrjy0isY6z9k3aCXXCqZNzkoa6dsYDNMmzYwqcb4lTEJcuRbH4JIr\/kiNQ6tIuErBecy+QSu5VjBtclbSSN\/GYJg2aWbAtCUklX9xDC75ayDtAbSSkqoel9XsG7SSawXTJmcljYRpk5JCnCLgW8K4LDsGF3fUg1bmtEp79g1aybWCaZOzkkb6NgZjpk2aGZhpS0gq\/+IYXPLXQNoDaCUlFS0ujX3foJVcA5g2OStpJEyblFRGccFtP3ibjwcffJAef\/xx2rt3LzU3N2fUi+rN+JYwuQNP0AEMLgngZVwUWqUL3ORbF6CVXCuYNjkraaRvY7DVM23asC1fvpza2tpo3759am823nB3bGwM+7RJsxpxigAGF3cSAVplp1XS2TdoJdcKpk3OShoJ0yYllUEcv9Jq\/fr1tH37drpw4ULJtJ0+fZo2b95sxWybbwmTgeypNYHBJTW0xiuGVsaR1q0w7uwbtKqLthQA0yZnJY30bQy2eqaNRdu4cSNNTU3RfffdR4cPH1a3Rx944AFasWJFbu8bDSaTbwkjvZBsjMPgYqMqlfsErfLVit91+srEm6J3nkIruVYwbXJW0kjfxmDrTRsLp98zqkXs6+ujnp4eqaapxvmWMKnCTLlyDC4pAzZYPbQyCDNBVZVm3x6540Zac\/M7zxNDKzlgmDY5K2mkb2OwE6ZNKl4ecb4lTB6MTbWJwcUUyfTrgVbpM47awhMvn1Ezb2zk+D2nbNzYwEErOUmYNjkraaRvYzBMmzQzqsT5ljAJceVaHINLrvgjNQ6tIuHKNDg8+\/b5Fa30x\/\/2ber4+AepoaEh07641hhMm3nFfBuDrTNtvPhg1apVNDk5WVPd1tZWGhkZyX3bD98Sxvwll12NMALZsU7aErRKSjD98mlv2pv+GWTfAkybeea+jcHWmbawpLwQgbf7CD7Dtnv3bmz5YT73C18jjIA7EkMrt7T63g9+St+cJHry2GTp1umam69XP+N4hwBMm\/lsgGkzzzR2jcEtP9rb20v1nDx5Elt+xKbqb0EYAXe0h1ZuajX1y7dp+LVz9MTLZ9UJBJ99c+eM0uspTJt5tjBt5pkmqpFn2kZHR2nPnj20cuXK0krSjo4ObPmRiKx\/hWEE3NEcWrmv1SsTl5SB48ULMG9X9YRpM5\/XMG3mmSaukWfWurq6aHp6mpqamtQbEYIzb4kbSFCBbwmTAFXuRWEEcpdA3AFoJUaVe2A9rcILF3jV6W1t15ZtG5L7SWTUAZg286B9G4Otf6bNvMRma\/QtYczSy7a2eoNLtr1Ba7UIQCt38iOKVtW2DXHnbJP1FKYtGb9KpX0bg2HaEuaQbwmTEFeuxaMMLrl2FI1j7y+HciDOdRWcfdO3Tn1YuADTZj6xfRuDYdoS5pBvCZMQV67F4wwuuXbY48ahlTviJ9FK8sYFd0jU7ylMW31GUSN8G4OtNm219mwztU9bpTaCr8ni7UV27typ8qjS67N8S5ioF5RN8UkGF5vOw4e+QCt3VDalFS9Y4Nun4TcuuEOifk9h2uozihrh2xhstWmrJh6vKL3zzjvVatKkR63tQ\/i73t5eGhwcVM3on4OLIHxLmKS88yxvanDJ8xx8aRtauaO0aa3Cs2+P3HEDFeXWKUyb+bz2bQx20rSZ3KeNX0a\/b98+GhoaosbGxrKMCm\/iW2mjX98Sxvwll12NpgeX7HruX0vQyh3N09KqiLdOYdrM57VvY7CTpo3N1KFDh4y8xqrW2xXYpPExMDCg\/g7\/zp\/5ljDmL7nsakxrcMnuDPxpCVq5o3UWWhXl1ilMm\/m89m0Mttq01XqmTW+2mzQF9Oa9up6lS5eWZt3CM2ts8CYmJso29dUJc\/DgQeLn7Phobm5O2i2UT4FAFoNLCt32skpo5Y7sWWqlZ9+Cr8v69E1\/6MzrsoaHh5WwixYtoiVLlmQucpZapXly7A30we8p7+zsVK+21GNwmm3nXbfVpi1tODMzM9Td3U0tLS3KiIV\/j2Lagn3ljYDXrl2bdvdRf0QCV65coYsXLypT3dDQELE0wrMkAK2ypJ2srTy04tdlPX9qhr52\/JLq\/KcWNtJfLZ1DLe+1+7o+evSo6u\/8+fPVn6yPPLRK4xz379+vNtkPHjBtaZCOWKfpd48GZ9WCM2rBbvEzbtu2bVO3Xnfs2KG+ktwe7e\/vp7lz56p4NgWYbYsodgbhly9fpgsXLqj\/jcG0ZQA8QRPQKgG8jIvmrdWuv7v6qiyehWPT9uW\/aFNvXLDx4HGFD55p4z9ZH3lrZep82Rvo2bbx8XHatWsXZtpMwY1TT\/C1VdXKVzNdcdoLmza9MIGdfPB2KBYiJKWbb\/mi3BrIl2I2rUOrbDibaMUWrdi06Tcu2PquUzzTZiLjyuvAM23mmcausdpMW+wKQwX1M3NbtmxR24fo31evXk09PT2ELT9MkbajHlsGFzto2N0LaGW3PsHe2aZVpbctPHLHjVYAhWkzLwNMm3mmVtcYXuzQ0dFRttAAm+taLV+kztk2uETqvGfB0ModwW3VqtJ+b3mbN5g283kN02aeaaQatYn6p3\/6J+LnxPj5Ml4dEj5MvREhUucqBPuWMEl55Vne1sElTya2tg2tbFVmdr9s18qmzXph2szntW9jsNerR02kj28JY4JZXnXYPrjkxcXGdqGVjapU7pNLWvEzb0+8fFadSB5vWoBpM5\/Xvo3BMG0Jc8i3hEmIK9fiLg0uuYKyoHFoZYEIwi64qFVe5g2mTZhUEcJ8G4Nh2iIkR6VQ3xImIa5ci7s4uOQKLMfGoVWO8CM27bJWWZs3mLaIySUI920Mtt608b5pGzZsmCUdnmkTZDNCygi4PLj4JiW0ckfxImgVNm9pLViAaTOf1zBt5pnGrjG8BUfsilIs6FvCpIgy9aqLMLikDsmSBqCVJUIIulEkrbR5S2ufN5g2QUJFDPFtDLZ6pi3tfdoi5kbFcN8SxgSzvOoo0uCSF8Os2oVWWZFO3k7RtArv8za4eiHd1jYnOSgigmkzgrGsEt\/GYKtNGytT6SXt5mWPX6NvCROfVP4liza45E80vR5Aq\/TYmq65qFqxefvM8Cl69fQlunXBHHr+MzclRgfTlhjhrAp8G4OtNm3hjW+DauGZNvPJX\/Qaizq4FFE3aOWOqkXX6pWJS3T37hNKEN4mJMnzbjBt5vMaps0800LX6FvCuCxm0QcXl7UJ9x1auaOmL1rp592SzLrBtJnPa9\/GYKtn2szLa75G3xLGPMHsavRlcMmOaHotQav02Jqu2SeteNat99AphTDOs24wbaazj8i3Mdhq01br9qiWvqmpiQ4cOEDt7e3ms0FQo28JI0BibYhPg4u1Igg7Bq2EoCwI800rftaNZ92GXztPa25upq+uWShWAaZNjEoc6NsYbLVpY9U2btxIbW1t1NPTUxIxuDiBfx4bG6NDhw6JRTYZ6FvCmGSXdV2+DS5Z8zXZHrQySTPdunzVik0bL1SIcrsUps18Lvo2Bltt2qpt+XHy5EnavHkz7d27ly5cuKB+fuGFF8xng6BG3xJGgMTaEF8HF2sFqdExaOWOaj5rpRcp8L5u\/2fLn9YVDaatLqLIAb6NwVabNj3TNjo6Snv27KGVK1eSfkNCR0cHDQwMqC1BMNMWOc+9LODz4OKa4NDKHcV814pvl35k27dJYtxg2sznNUybeaaJa+SZta6uLpqenqbgM2zBGbfm5ubE7cSpwLeEicPIljK+Dy626CDpB7SSULIjBloRSY0bTJv5nPVtDLZ+ps28xGZr9C1hzNLLtjYMLtnyTtIatEpCL9uy0Ooqb23caj3jBtNmPjd9G4Nh2hLmkG8JkxBXrsUxuOSKP1Lj0CoSrlyDodU7+PUzbtU24YVpM5+qvo3BMG0Jc8i3hEmIK9fiGFxyxR+pcWgVCVeuwdCqHL9eVfo3PTfNemcpTJv5VPVtDLbetOmFB2Gp8Ror88lf9BoxuLijMLSCVu4QmN3TT331BP30zbdmrSiFaTOvKkybeaaxa9Sb6z788MP04osvUm9vLy1YsIC6u7tp+fLlZXu3xW4kYUHfEiYhrlyLwwjkij9S49AqEq5cg6HVbPz6+bbw5rswbeZT1bcx2OqZtuA+bfv37y9tsmvDqlGder4ljPlLLrsaMbhkxzppS9AqKcHsykOryqz1bVLev423A+EDps18Xvo2Bltt2mZmZtSsWktLC9155520YcMGevLJJ+nw4cM0NTVFIyMjlNdWHzBt5i++tGvE4JI2YXP1QytzLNOuCVpVJ3zdxr8te9UVTJv5bIRpM880cY2PPfYY3XPPPertB2zcli5dSkNDQ9TY2Ji47qQV+JYwSXnlWR6DS570o7UNraLxyjMaWlWnz+8ofeLls\/TGwO2YaUspSX0bg62eaUtJY6PV+pYwRuFlXBkGl4yBJ2gOWiWAl3FRaFUbOM+28Uvl+fk2zLSZT07fxmCYtoQ55FvCJMSVa3EMLrnij9Q4tIqEK9dgaFUbP68k5Wfa2LjBtJlPVd\/GYOtMm14xOjk5WVNdbPlhPvmLXiMGF3cUhlbQyh0CtXsaXJDwv\/\/nsyp48eLFtGTJksxPsYjXFUxb5mlU3mDQtNlizGoh8S1hck6PRM0X8R+sREAsLgytLBYn1DVoVV8rfYv0X\/\/hGExbfVyRInwbg62baQuqFdxY11YD51vCRLqaLAvG4GKZIDW6A62glTsE6vf0I9u+TfxO0lv+9e9h2urjihTh2xhstWkLKrdx40YaHR1VH2H1aKScRvBvCcAIuJMK0ApauUOgfk8\/M3yKXj19iTbd+COYtvq4IkXAtEXClX2wvn3KLWMsXWfGAAAgAElEQVSftuz5u9wijIA76kEraOUOgfo91c+1fWnxBbru9\/8Fz7TVRyaOgGkTo8ouUG+ye\/z4cdVoR0cHDQwMZNeBGi35ljBWQI\/ZCRiBmOByKAatcoAes0loJQPHz7X95Q2XaNkf\/AqmTYZMFOXbGGz17dHgLVGbjFowk3xLGNFVZGkQBhdLhanQLWgFrdwhIOspTJuMU9Qo38Zg60xbcPWorUYNpi3qZWVHPIyAHTpIegGtJJTsiIFWMh14McL1v\/OGmm3Dlh8yZpIomDYJpRRjsE9binA9rxqDizsJAK2glTsEZD3lxQjf\/f6P6cEP\/QKmTYZMFAXTJsKEIE3At4RxWXkYAXfUg1bQyh0Csp7q95B+5aNTMG0yZKIo38Zg626PilSyKMi3hLEIfeSuwAhERpZbAWiVG\/rIDUMrGTK9ghSmTcZLGuXbGAzTJs2MKnG+JUxCXLkWx+CSK\/5IjUOrSLhyDYZWMvyvTFyiu3efIN72488++id4jZUMW90o38ZgmLa6KVE7wLeESYgr1+IYXHLFH6lxaBUJV67B0EqG\/ydvvEW8GOHBD\/2c7rnlj2HaZNjqRvk2BsO01U0JmLaEiKwpjsHFGinqdgRa1UVkTQC0kkuht\/3o\/vcLYNrk2GpGwrQZAulLNb4ljMu6YnBxRz1oBa3cISDvadsXj9IHm67QE5+aB9MmxwbTFiCAmbaEiQPTlhBghsVhBDKEnbApaJUQYIbFoZUc9icef0EFw7TJmdWL9G0MhmmrlxF1vvctYRLiyrU4Bpdc8UdqHFpFwpVrMLSS42fT9sPpd9FznX+EmTY5Nsy0YabNULYQEUybOZZp14TBJW3C5uqHVuZYpl0TtJITfuaZZ1Qw3oggZ1Yv0rcxGDNt9TICM20JCdlTHIOLPVrU6wm0qkfInu+hlVwLmDY5K2kkTJuUlINxR44coX379tHQ0BA1NjaWziD4Yvo9e\/bQypUrS9\/t3r2bdu7cqX7v6+ujnp6esjP3LWEclL3U5bNnz9LTTz9N3d3d1Nra6vKpFL7v0ModiaGVXKu8TVsRtfJtDPZmpo0N24YNG2jp0qVlpo0\/37ZtG42MjNDrr79e+rm5uZlOnjxJvb29NDg4qK5K\/XN7e3vpKvUtYeT\/PNkXCa3s06Raj6AVtHKHgLyneZu2Il5XRTynWhnlhWnjmbRjx44pwzY9PV1m2vg7PgYGBmhmZkbNwqxbt07NtvEs29jYWCmeY9va2spm2\/JOmMnJSXr22Wfp3nvvTTx7FKcuaRlJXK2Yat9F+RxayXIFWhFJGEiH6jh1RSlTLxbXlVSpeLrX4x9sne\/ynD9\/Xo0vPB6FjzhacR2VylX6rIj\/BvJdk02bNqmx2oc7KF6YNp5Nq2TCtElbvny5MmLh34OGji+M8O\/8mb4IDh48mEvC8IXZ2dlJJtqPU5e0jCSuVky176J8LumD\/J\/36JEm249Tl7SMJA5ayfWX8Kw0eEuv63r1Qyt7tBodHaUTJ07QXXfdVdW0VdM9qo6V4uvlipxUvEiT7eu6+vv7YdriyWF\/qfDMWXhmTRszPZsWnlnj8hMTE2pWTh+cOOzyx8fH7QeAHoIACIAACORG4Pbbb1dtnzlzhvj5MhxmCCxbtoyGh4fNVGZ5LV7MtGkN0jBtemqazRsOEAABEAABEACBbAnwbVEfbo0y1UKZNl440NXVpZ5b46PSStDgM2ombo9mm5poDQRAAARAAARAwFcChTJt9UQMz7RxfPAWaKWFCMHboZUWItRrE9+DAAiAAAiAAAiAgAkC3pu2pFt+mBABdYAACIAACIAACIBAPQLemzY928areviIurluPcD4HgRAAARAAARAAARMEPDKtJkAhjpAAARAAARAAARAIA8CMG15UEebIAACIAACIAACIBCRAExbRGAIBwEQAAEQAAEQAIE8CMC05UEdbYIACIAACIAACIBARAIwbRGBIRwEQAAEQAAEQAAE8iAA05YHdbQJAiAAAiAAAiAAAhEJwLRFBIZwEAABEAABEAABEMiDAExbHtTRJgiAAAiAAAiAAAhEJADTFhEYwkEABEAABEAABEAgDwIwbXlQR5sgAAIgAAIgAAIgEJEATFtEYAgHARAAARAAARAAgTwIwLTlQR1tggAIgAAIgAAIgEBEAjBtEYEhHARAAARAAARAAATyIADTlgd1tAkCIAACIAACIAACEQnAtEUEhnAQAAEQAAEQAAEQyIMATFse1NEmCIAACIAACIAACEQkANNWB9ju3btp586dKqqvr496enoiIkY4CIAACIAACIAACCQnANNWg+HJkyept7eXBgcHVZT+ub29PTl51AACIAACIAACIAACEQjAtNWAxbNsY2NjNDQ0RI2NjbRx40Zqa2vDbFuEBEMoCIAACIAACICAGQIwbTU4sknjY2BgQP0d\/t2MBKgFBEAABEAABEAABOoTgGmrY9qCM2s88zYxMVEycbro5ORkfdKIAAEQAAEQAAEQME6gtbXVeJ22VgjTltC0sWH7yLZvl9Xyu7\/6RSy9f\/dXP69arlqd4TKzfy\/vS602YnUahUAABEAABEQEbr\/9dhV35swZOnv2rKgMguoTWLZsGfX395MP5g2mrY5p469r3R79zne+Q\/dsPUz33nsvtbbOVbX95I236mdZhYh65X76Znm99eLrdaKWwfvAddeo4vOuvfq3PvTn+m82k7t27aInv\/QIzZ179fwrHePj4yqOL6xqcbViqn0X5XNJH+oxS\/K9yfbj1CUtI4mDVvJMkPAM1xalTL1YaGWPVkePHlWGbcWKFTR\/\/vxZHYujFVdSqZz0Mzmd5JH1cjVKC7ouHnufffZZ9fw5TFsUggWMDd8OrbQQgU1bZ2dnbgnDM32csA899FBFwxg2dkHj95M3LpepFowN\/qzLSEyiNnNBw8efsbm72s8H6QPXvbtkBvk7XSZ8LsHOVfsuyue2aZXkkqnFqlq90jKSOGglV0\/CM1xblDL1YqGVPVrxorbz58\/T3XffTUuWLJnVsThacSWVylX6rIj\/BvIsW55jsDy7zERipq0GR8mWH3lfBGbSIFotlczdO8buHSPIcWHzV8v4BWf3gmYuaPJua5sTrbOBaB+1ig0r54LQKmcBIjQPreSwnnnmGRW8ePHiiqZNXlO8yCJqVcRzqqUuTFud3K+3ua5vCRPvn4rZpbR547\/Dhi9o9l49falik9rg3bpgTmmmTpu7asaOnyF5+umnacuWLdTQ0GDqVFBPCgSgVQpQU6oSWsnB5m3aiqiVb2MwTJv8eqsY6VvCJMQVu3jY5Olbu69MXDV1lcwdGzt+Jk\/P2rW8t4He\/S8z1PHxD8K0xVYim4JvvfUWnTt3jubNmwetskEeuxVoJUeXt2krola+jcEwbfLrDaYtIau0i1cydnrWLmzqwjN19Wbp0u476i8nUMTBpagaQyu5sjBtclbSSJg2KSnEKQK+JYzLsv\/D1CW6ePEi\/fpd19L4j6fVqfBMXSVDxzN0fJsVZi4fxWEE8uEep1VoJacG0yZnJY30bQzGTJs0M6rE+ZYwCXHlWrzW4KJn6djA8a3XamZOP0N364JrlanDkQ4BGIF0uKZRK7SSU4Vpk7OSRvo2BsO0STMDpi0hqfyLxxlctJkbfu1cxZk5vs265uZm9R2MnDmN42hlrnXUFIUAtJLTgmmTs5JGwrRJSSFOEfAtYVyW3eTgwmaukpHTix94Fg4mLn62mNQqfi9QUkIAWkkoXY2BaZOzkkb6NgZjpk2aGZhpS0gq\/+JpDy7ayIVvrerZOJg4eQ6krZW8J4isRwBa1SP0zvcwbXJW0kiYNikpxGGmzbEcyHpwCc7GPfHyO+8ZhImrnzhZa1W\/R4ioRgBayXMDpk3OShoJ0yYlhTiYNsdyIO\/BRZu4q3+fV\/SCz8Q9cseNjhFNr7t5a5XemRWvZmgl1xSmTc5KGgnTJiWFOJg2x3LAtsEleDuV3wrBv2sTt+bm60tvenAMs5Hu2qaVkZMqaCXQSi4sTJuclTQSpk1KCnEwbY7lgM2DS61ZOB8NnM1aOZb2qXcXWskRw7TJWUkjYdqkpBAH0+ZYDrg0uFxdzPAm6WfhfJuBc0krxy4D492FVnKkMG1yVtJImDYpqYLEnT9\/nlatWkWTk5OlM+rr66Oenh71O14YXxChicjVwUXPwoUNXJGfgXNVq+JcLfIzgVZyVjBtclbSSJg2KamCxJ08eZI2b95Me\/fupebmq5uk6oO\/6+3tpcHBQfWR\/rm9vb0U41vCuCx7EQaXSjNwbN70Br8u6xPsexG0KooW9c4DWtUj9M73MG1yVtJI38Zg7\/dpO3LkCO3bt4+GhoaosbGxLE94lm1sbKz03caNG6mtra00C8fBviWM9EKyMa5Ig0t4T7ii3T4tklY2Xgsm+wSt5DRh2uSspJG+jcHem7awMQsmCps0PgYGBtTf4d9h2qSXlR1xRR1cinj7tKha2XElmO0FtJLzhGmTs5JGwrRJSRUkjo3Y6Oho6WyWLl1adWaNDd7ExETJxAVN28GDB6m1tVXVE77NWhBUzp+GD4MLG7gnXj5D3zjx89L2IZ++6Q+d2z7EB62cv6B+ewLQSq7k8PCwCl60aBEtWbJEXtBQZFG04mfR9cHPo3d2dqq7YnoMNoTLymq8nmmbmZmh7u5uamlpUUYs\/Hv4dmgt0xZUt6uri9auXWul4D536sqVK3Tx4kVlqhsaGgqNYuqXb9Pzp2boa8cvqfNseW8D\/dXSOfSpheWPANgKwSetbNVA2i9oJSVFdPToURU8f\/589Sfroyha7d+\/nw4cOFCGD6Yt62zKoL3grFpwRi3YND\/jtm3bNhoZGaEdO3aoryS3R\/v7+2nu3Lkqnk0BZtsyEDRiE5cvX6YLFy6o\/40V3bQF0ez6u3PqDQw8C8fm7dM3vY9sX3nqq1YRU9qKcGgll4HHFT54po3\/ZH0URSueadOzbePj47Rr1y7MtGWdTLa0F1yYwE4+eDsUCxFsUSleP4pyayDe2ZMybXzrlA2cXrhgq3nzXau4GudRDlrJqeOZNjkraSSeaZOSyiBO3648fvz4rNaqzZRF6Zbeo23Lli20cuVK5dx5z7bVq1erFaLY8iMKTftjMbhc1Si8cOGRO26wbuYNWtl\/PekeQiu5VjBtclbSSJg2KakU49gs8XNhfPBsV3BfNN0sz4ht2LCBmpqaqsZIuhjeXLejo6NsoQE215VQdCMGg0u5TpXMmy2vzIJWblxT3EtoJdcKpk3OShoJ0yYllVIcmyh+lmzr1q2z9k2r1CTPxj366KNlRiulrlWs1reEyZKt6bYwuFQnyrdN9RsXeOYtb\/MGrUxnf3r1QSs5W5g2OStppG9jsNerR6VJUSvOt4QxwSyvOjC41Cevn3njWbhbF8yhr65ZmMt2IdCqvla2REAruRIwbXJW0kjfxmCYNmlmVInzLWES4sq1OAYXOf6gectj5g1aybXKOxJayRWAaZOzkkb6NgZbbdoqvcw9LGTSZ9qkiVEtzreEScorz\/IYXKLTD982zWq1KbSKrlVeJaCVnDxMm5yVNNK3Mdhq08aiVdpmI7jJrX4N1aFDh6QaG43zLWGMwsu4Mgwu8YFnbd6gVXytsi4JreTEYdrkrKSRvo3BVps2nmlbv349bd++vWwFKa8u3bx5M+3du1dtlso\/v\/DCC1KNjcb5ljBG4WVcGQaX5MCzMm\/QKrlWWdUAreSkYdrkrKSRvo3BVps2PdPG7wbds2eP2ktNb\/Wht+bATJs0tRGHwcVMDgS3Cklrk15oZUarLGqBVnLKMG1yVtJImDYpqQzj9L5t09PTZfuyBWfc8nptlG8Jk6HsxpvC4GIWadi88fNua25uNtIItDKCMZNKoJUcM0ybnJU00rcx2PqZNqlwecX5ljB5cTbRLgYXExRn1xF+Pdbg6oV0W9ucRI1Bq0T4Mi0MreS4YdrkrKSRvo3BMG3SzKgS51vCJMSVa3EMLuniZ\/P2meFT9OrpS4n3eINW6WplsnZoJacJ0yZnJY30bQy23rQF3z\/K7xt98MEH6fHHH1eLEPK6JRpMJt8SRnoh2RiHwSUbVV6ZuES9h06pd5zGfa8ptMpGKxOtQCs5RZg2OStppG9jsNWmTRu25cuXU1tbG+3bt4+GhobUu0bHxsbUz42NjVJtU4nzLWFSgZhRpRhcMgL922b0StM4ixWgVbZaJWkNWsnpwbTJWUkjfRuDrTZtwS0\/eGsPbdpOnz5d2vIjymwbrzzVdQTNHu8FxytU+dCrVHXC4IXx0kvH\/jgMLtlrFHexArTKXqu4LUIrOTmYNjkraSRMm5RURnFsqKampui+++6jw4cPq9ujDzzwAK1YsSLSS+L1ViF8izU4Q8efb9u2jUZGRuj1118v\/cxmkFen9vb20uDgoDpb\/XN7e3vp7H1LmIxkT6UZDC6pYBVVGnWxArQSYbUiCFrJZYBpk7OSRvo2Bls906ZF04ZL\/97X10c9PT1STdVbFY4dO0Zs2HjbkKBp4+\/4GBgYIH07dt26dWpPOL0HnI6v9HYG3xJGDN3CQAwu+YsiXawArfLXStoDaCUlRQTTJmcljfRtDHbCtEnFqxbHpq+SCQs+M8cmMPx70NBx3eHf+TPfEiapFnmWx+CSJ\/3ytodfO0\/8zFu1xQrQyh6t6vUEWtUj9M73MG1yVtJI38ZgL0ybFj88cxaeWdPGjBc9sIkLz6wF33mq69QJc\/DgQWptbVUfR3nOTpqYiEtOAINLcoama2Dj9uSxSdKLFT6\/Yp5qAlqZJp1efdBKznZ4eFgFL1q0iJYsWSIvaCiyKFrx8+76mJycpM7OTrU4UY\/BhnBZWY11po3FWLVqFbEQtQ4Wh59Di2KQ0jRtwb52dXXR2rVrrRTc505duXKFLl68qHKmoaHBZxRWnfvUL9+m50\/N0NeOX6KW9zbQXy2dQyvn\/x60skql6p3BdSUX6ujRoyp4\/vz56k\/WR1G02r9\/v9pFInjAtGWdTVXaq\/QcWdh86aLB113xZ5VWgga3CjF5e7S\/v5\/mzp2rusKmIIqZtAR14btx+fJl4lXIbPhh2uyTm83bQ9\/4odqcl83bf\/53TfQflrVBK\/ukKusRriu5QDzRwAfPtPGfrI+iaMWTO3q2bXx8nHbt2oWZtqyTqVJ7wS0\/gis2475ztJLZC5rCSgsRJiYmSqtUsRDBhqyI34ei3BqIT8CNkvyc2927T6jn3W5dMIe+umahun2Kw04CuK7kuuCZNjkraSSeaZOSyihO76GmZ830StKOjo5IW35wdyuZNmz5kZGQFjSDwcUCEYRdYK2+9r9+SPv+fibRmxWEzSEsAQFcV3J4MG1yVtJImDYpqQzjgrc9m5qa1L3s4MybtCvVbqtic10pQbfjMLi4o19QqyeP\/ZSeePlsabHCI3fc6M6JeNBTXFdykWHa5KykkTBtUlKIUwR8SxiXZcfg4o56Ya3Cb1ZYc3MzwbzZoSeuK7kOMG1yVtJI38Zg61aPSoWyJc63hLGFe5x+YHCJQy2fMtW0gnnLR49areK6kmsC0yZnJY30bQy2zrTx4oMdO3bQ1q1bRS+D58UDjz76aOTn26QJUS\/Ot4Spx8Pm7zG42KxOed\/qaRV+LRbPuvHsG47sCdTTKvse2dsiTJt5bXwbg60zbSypfoaNf672\/JpekJDkGTcT6eNbwphgllcdGFzyIh+9XalWwddi8QpTmLforJOWkGqVtJ0ilIdpM6+ib2OwlaZNy6q34Dh+\/PgspcMvfjefCrIafUsYGRU7ozC42KlLpV5F1QrmLT9to2qVX0\/zbxmmzbwGvo3BVps28\/Kar9G3hDFPMLsaMbhkxzppS3G1gnlLSj56+bhaRW\/J\/RIwbeY19G0MhmlLmEO+JUxCXLkWx+CSK\/5IjSfVCuYtEu5EwUm1StS4Y4Vh2swL5tsYDNOWMId8S5iEuHItjsElV\/yRGjelVXjBArYKiSSDKNiUVqLGHA+CaTMvoG9jMExbwhzyLWES4sq1OAaXXPFHaty0VtgqJBL+SMGmtYrUuGPBMG3mBfNtDIZpS5hDviVMQly5Fsfgkiv+SI2npRXMWyQZRMFpaSVq3LEgmDbzgvk2BsO0Jcwh3xImIa5ci2NwyRV\/pMbT1ipo3rhjj9xxA625+Xq8mD6SSleD09YqRpesLQLTZl4a38Zg601bcNsP3ubjwQcfpMcff5z27t1Lzc35b6bpW8KYv+SyqxGDS3ask7aUpVZPvHyGhl87r15Mz3u94bm3aOplqVW0ntkXDdNmXhPfxmCrTZs2bMuXL6e2tjbat28fDQ0NqQ13x8bG1M+NjY3msyBCjb4lTAQ01oVicLFOkqodykMrLFqIlx95aBWvp\/mXgmkzr4FvY7DVpo1fabV+\/Xravn07XbhwoWTaTp8+TZs3b44828ZvUdDGT5s9bmPVqlU0OTlZyqa+vj7q6elRv+\/evZt27typfg5+roN9Sxjzl1x2NWJwyY510pby1Cp86xRvWqitZp5aJc2zrMvDtJkn7tsYbLVpY3k3btxIU1NTdN9999Hhw4fV7dEHHniAVqxYEel9o\/q1V+E3KfArs6oZQP6ut7eXBgcHVabpn9vb20uZ51vCmL\/ksqsRg0t2rJO2ZItWlW6d4tm3cnVt0SppzmVRHqbNPGXfxmDrTRtLrA2XlrvSjFetVGDjd+zYMWLDNj09XXZbtdLsm66LZ9mCt2G5Hr5Nq2fhOM63hDF\/yWVXIwaX7Fgnbck2rSrNvuHZt6sq26ZV0txLszxMm3m6vo3BTpi2pDKzMVu5cqW61Rl+Fq7SZ7o9Nml8DAwMqL\/DvwdN28GDB6m1tVXF2bBAIimzIpbH4OKOqjZrNfzaOXpl4hJ948TPSwsXPnDdu+nTN73PHcAGe2qzVgZP00hVw8PDqp5FixbRkiVLjNQZpZKiaMWPNemDH23q7OxUY7seg6MwcS3WC9OmRalk0NiIjY6OlnQL3j4Nz6xx+YmJibLbstrlB4Xv6uqitWvXupYLhe\/vlStX6OLFi8pUNzQ0FP58XT5BF7Sa+uXb9PypGfof358h\/rnlvQ305x9upI+1XkMfnXuNy\/gj9d0FrSKdUIrBR48eVbXPnz9f\/cn6KIpW+\/fvVwsSgwdMW9bZVKU9niXbtm0bjYyM0HPPPacWBTQ1NSnBgs+WSbobNm16dWpLS4syYuHfo5i2\/v5+mjt3ruoGmwLMtkkUyTbm8uXLakEL\/28Mpi1b9lFbc00rNm3fOHGxtHUIGzieebt1wRy6re3aqKfvVLxrWuUJl8cxPnimjf9kfRRFK55p07Nt4+PjtGvXLsy0ZZ1MldrTJmrdunVqKplXeW7ZskWFhleB8me8cIBnufi5NT727Nmjbovqo9atUB0TNIk7duxQH0tuj\/ri8m3Ii7h9KMqtgbjn71I5l7Wq9vzbrQuupdva5rgkg6ivLmslOkGDQXimzSDM31aFZ9rMM41dY3jLDz3jxrMlcbb8kJq24H5wwduhWIgQW0orCmJwsUIGUSeKopUPBq4oWokSM2EQTFtCgBWKw7SZZxq7xuBMG5snvYgg7ua6YdOm92jj2TuekdO\/r169Wq0QxZYfsaWzsiAGFytlqdipImqlDVzw7Qvzrr1Gzb49cseN7ogT6mkRtUpLDJg282Rh2swzTVSj3u5DP8fGlVXaL03SSKWZtvDmuh0dHWULDbC5roSsGzEYXNzQiXtZdK20geNVqK+evqSE0a\/Qcu02atG1MnnVwLSZpHm1Lpg280wLXaNvCeOymBhc3FHPJ620gWN1nnj5bJmB419sn4XzSaukVxBMW1KCs8v7NgZ7teWH+XTxz+WnwTCrOjG4ZEU6eTs+a+XaLJzPWkXNdJi2qMTqx8O01WeUaUT4bQi6cd62gZdP5721hm8Jk6n4hhvD4GIYaIrVQaurcNnA8e3Tn7xxuTQLx5\/bdCsVWskvBJg2OStppG9jsNUzbfp5s4cffphefPFF9SzbggULqLu7m5YvX172OimpwKbjfEsY0\/yyrA+DS5a0k7UFrSrzqzYLpxc05PE8HLSS5zpMm5yVNNK3Mdh607Z+\/Xravn078Q7I+r2ftV7yLhXaVJxvCWOKWx71YHDJg3q8NqGVjJsNJg5aybTiKJg2OStppG9jsNWmLfiGgjvvvJM2bNhATz75JB0+fJimpqZwe1Sa1YhTBDC4uJMI0CqeVnmYOGgl1wqmTc5KGgnTJiWVYdxjjz1G99xzj3oFERu34PtBM+xGxaZ8S5i8eSdpH4NLEnrZloVWZnhnYeKglVwrmDY5K2mkb2Ow1TNtUtHyjPMtYfJknbRtDC5JCWZXHlqlwzoNEwet5FrBtMlZSSN9G4Nh2qSZUSXOt4RJiCvX4hhccsUfqXFoFQlX7OBKJo4ri7I6FVrJ8f\/H\/r+h8V+8h57r\/CP1Pu2sjyJq5dsYDNOW8KrxLWES4sq1eBH\/wcoVaIqNQ6sU4daoOo6Jg1ZyrWDa5Kykkb6NwdabNuzTJk1dxNUjgMGlHiF7vodWdmhR6W0N4Zm4j7VeQ+fOnaN58+ZRQ0ODHR23tBcwbeaFgWkzzzR2jeEXuMeuKMWCviVMiihTrxpGIHXExhqAVsZQGq2omolreW8D\/ac\/baU89okzeoIpV\/aJx1+gN\/7539DX723B7VFDrH0bg62eaWPTpvdpa29vjy3xxo0baXR0VJXXL54P1hf8fs+ePbRy5cpSW3hhfGzs1hWEEbBOkqodglZuaMUm7r9++6f0ze\/\/nL73s7dUp\/l5OL3Z75qbr1e\/47hKAKbNfCbAtJlnmqhGvj360ksv0cDAQKx62HSNjY3R0NAQNTY2Ev9+6NCh0h5vXP+2bdvU76+\/\/nrpZ349Fm\/iy29hGBwcVG3rn4OGz7eEiSWCJYVgBCwRQtANaCWAZElIUKupX75Nw6+do1cmLsg5aRQAABPeSURBVKnXb2kTt+bmZszCwbSlkrG+jcHWzbTpW6KTk5M1BY777tGgEWPzxbNsfLAp1Jv5rlu3Ts22hQ0fx+q3MujO+ZYwqVx1GVUKI5ARaAPNQCsDEDOqopZWV83bmzT82nn1HtWgifNxFg4zbeaT0rcx2DrTZl7S8hqDpi38HlNt2vR7TYOGjmsJ\/86f6YQ5ePAgsZHkI++X2KfN0NX6YQTcUQ5aFU+r4MrU8R\/PzJqF+\/gNje6cdMye3rL1qCq5pwNbfsREqIrx5I4+eIKns7NT3VHTY3CSum0va61pq\/WcWRKoXC+\/Aotvl\/LBL5\/XM2vamOnZtPDMGs+8TUxMlN2q1aYt2Keuri5au3Ztkm6ibAoErly5QhcvXlSmGqvcUgBssEpoZRBmylXF1YpvpT5\/aoa+dvzqbVRezPDnH75q3O5fNiflXudT\/af3nqJrf+\/X9MXbGmn+\/PmZdyKuVpl3tE6D\/C7yAwcOlEXBtOWoEpul7373u+o5M351FZug+++\/n3p6ehL1ik3XU089pcTmW6Ph26FJTFt\/fz\/NnTtX9Y9NAWbbEkmVSuHLly+rfOL\/jcG0pYLYWKXQyhjK1CsyoRUbuG+cuFj2LNxH515Dt7XNoUfuuDH1c8iqgeXbv0nX\/f7b9PgnrqdFixZl1WypHRNaZd7pCg3yTJuebRsfH6ddu3Zhpi0vYSqtGOXFAvv27SstJqjWN771yQZvenpahQRXgoYNG38fvh2a5PaoLy4\/r7ww0S5uuZmgmE0d0CobziZaMa1VpQ1+b10wpxAGbtljL9F1v\/8v9MSn5mHLDxPJF3hEyZcx2Lrbo0lMW7UcCK8YDcYFb4FWWogQvB2KhQiGrrKcqjE9uOR0Gl40C63ckTlNrcIGTr9e6wPXvZt4RaprB0ybecWwEME800g1mjZtPEu3adOm0i3RcGew5UckeZwOTnNwcRqMhZ2HVhaKUqVLWWmlDZxeiaoNnEurUNm0tTX+Mz1yxw2YaTOU4jBthkDGrca0aQsuaAj2KXjrFJvrxlXLrXJZDS5uUbGzt9DKTl0q9SoPrbSBe+Lls6pL2sDZ\/vxb2xeP0rI\/+BVMm8H0hmkzCDNOVWnv0xanT7XK+JYwpvllWV8eg0uW51ektqCVO2rmrRUbuCdePqP2guODZ7FsnX2DaTOf176NwdY902Ze0nRr9C1h0qWZbu15Dy7pnl2xaodW7uhpi1bVbp\/aNPsG02Y+r30bg2HaEuaQbwmTEFeuxW0ZXHKF4Ejj0MoRoYjIRq2Cs2823TqFaTOf176NwTBtCXPIt4RJiCvX4jYOLrkCsbhxaGWxOKGu2axV+Nm3vG+dXrfxb+kT10\/jmTaD6e3bGAzTljB5fEuYhLhyLW7z4JIrGAsbh1YWilKlSy5oVenW6eDqhWrvtywPNm1\/ecMl6v73C7B61BB438ZgmLaEieNbwiTElWtxFwaXXAFZ1Di0skiMOl1xTSt+iX3voVPqBfZ86zRL8wbTZj6vfRuDYdoS5pBvCZMQV67FXRtccoWVc+PQKmcBIjTvqlZh88YLFtLcsJdN4ke2fZse\/NDP6Z5b\/hgzbRFyrFaob2MwTFvCxPEtYRLiyrW4q4NLrtByahxa5QQ+RrOua8Vm6jPDp+jV05dSnXmDaYuRXIIivo3BMG2CpIDLTwjJkuKuDy6WYMykG9AqE8xGGimKVkHzxu86\/eqahcrEmTpg2kyRLK8Hpi0droWt1beEcVnIogwuLmsg7Tu0kpLKP65oWgVvm\/JqU1P7vHG9d+8+QV9afIH+7KN\/gtujhlLXtzEYM20JE8e3hEmIK9fiRRtccoWZcuPQKmXABqsvqlb8lgX9miwT5k2btq98dIoWL14M02YoB30bg2HaEiaObwmTEFeuxYs6uOQKNaXGoVVKYFOotuhaafOWdJNefs3Wf3n2\/6mZNpg2c4no2xjshWkLvhC+qamJDhw4QO3t7SprKr3rtK+vj3p6etT3u3fvpp07d6qfg5\/rlPMtYcxdatnXVPTBJXui6bUIrdJja7pmH7QKv2Hhb3puivy8G5u\/r\/\/taZg2wwno2xhceNPGpmtsbIyGhoaosbFRmbBDhw7RyMgINTc308mTJ2nz5s20d+9e9Xvw4O96e3tpcHBQfax\/1oaPP\/MtYQxfb5lW58PgkinQFBuDVinCNVy1T1oled4Nps1w4v22Ot\/G4MKbtnCaBI0Ym68jR47Qvn37SqYuGB82fDxj19bWVpqFg2lL5yJMq1afBpe0GGZVL7TKinTydnzUKnjLVLq\/26e+eoL+8R8v0IMf+gVujyZPu1INMG0GYdpYVdi0hY1ZsM9s0vgYGBhQf4d\/h2mzUeHqffJxcHFLoXd6C63cUc5XrYK3TCVbhPDGuovedZE+2TIN02YwvWHaDMK0sSo2XlNTU6WZteDzbtzfpUuXln0XnFljgzcxMVEycUHTdvDgQWptbVWnHL7NaiMHH\/vk6+DiotbQyh3VfNeKb5l+7r\/9UL0Wi1eZfn7FvFniTf3ybfU2hM9+8CK1NV6hRYsWYfVoghTnZ9H1MTk5SZ2dneoxKD0GJ6ja+qJe3R5l0\/XUU0+VFiLMzMxQd3c3tbS0KCMW\/j18O7SWaQsq3dXVRWvXrrVefN86eOXKFbp48aIy1Q0NDb6dvlPnC63ckQtaXdXqqfFL9LXjl6jlvQ305x9upPuXvfMy+udPzajvHpr7fRU7f\/589Sfroyha7d+\/X43jwQOmLetsMtAe3\/pkwzQ9Pa1q27NnD61cuVL9HDZs1ZrjZ9y2bdumFirs2LFDhUluj\/b399PcuXNVPJsCzLYZENRwFZcvX6YLFy6o\/43BtBmGa7g6aGUYaIrVQat34PKM2jdOXFT7u7F5+\/RN71MzcP\/9\/15S7zVd9uvXVDDPtPGfrI+iaMUzbXq2bXx8nHbt2oWZtqyTKc32witGa7UVXJjATj54OxQLEdJUKf26fb+Nkz5hcy1AK3Ms064JWs0mzEZt+LVzxLdO+bitbY56s8Izzzyjfsc+beayEs+0mWNpRU1swjZt2lS2N5vumN6jbcuWLWpGTv++evVqtUIUW35YIaGxTmBwMYYy9YqgVeqIjTUAreQoYdrkrKSRMG1SUo7EhRca6G7rW6fhzXU7OjrKFhpgc11HhBZ08+zZs\/T000+r5xh9eGBVgMTaEGhlrTSzOgat5FrlbdqKqBVMmzz\/EInNdZ3KAd8ubqfECXUWWrmjHrSSa5W3aSuiVkU8p1oZ5dXqUfmlJY\/MO2F4ufOzzz5L9957b+LZozh1SctI4mrFVPsuyufQSpYr0IpIwkD6r0ScuqKUqReL60qqVDzd6\/EPts5v5uG7O\/w4Dm8vFT7iaMV1VCpX6bMi\/hvId034ESisHpXnudeR+iII7tOWJRC9R42J9uPUJS0jiasVU+27KJ9L+pCmdibbj1OXtIwkDlrJM0XCs9LgzXtPSa7revVDK3u0Gh0dpRMnTtBdd91V1bRV0z2qjpXi6+WKnFS8SJPt67p45waYtnh6eFmKE4cThpcd4wABEAABEACBagRuv\/129dWZM2eIny\/DYYbAsmXLaHh42ExllteC26MGBGLjxn9wgAAIgAAIgAAIZEuAb5H6srgMpi3b3EJrIAACIAACIAACIBCLAExbLGwoBAIgAAIgAAIgAALZEoBpy5Y3WgMBEAABEAABEACBWARg2mJhq18ouGlveMPe+qURkQeBxx57jO655x5qb2\/Po3m0KSAQfL8wrisBsBxDgv8G9vX1qbfM4LCbAL9B6KWXXirbYN7uHvvXO5i2lDTnNynwwS+wf\/jhh6m3txdmICXWSaudmZlRb0k4depUxdedJa0f5c0RCBprmGxzXNOoif8NbGtro1tuuYUeffRR+sIXvkDNzc1pNIU6DRDQJvtjH\/sYTJsBnmlVAdMmJKsH9nXr1qmNEfVR7TVXwQFF\/+MVLCdsFmExCETV6kc\/+pFqZf\/+\/Zhpi8E7SZGoWgXbgmlLQj562bhasRnYsWMHbd26lRobG6M3jBKRCcTRil\/5+P73v59+9atf0Ze+9KXIbaJANgRg2gSc9QVw\/Phx0u8s5WK1XigP0yYAm0JIHK10N2ACUhCkRpVJtMJtHDe00rez77\/\/ftwezUiyONeVvp7Wrl1Lzz33HExbRlrFaQamrQ41\/Y\/OwoULaWpqirZs2VKaaeMZNH51Br+ahP8Hyf9T4dsB\/OwGTFucdExWJq5WMG3JuMcpnUQrvu4mJiZwCycO+Bhlkmilm8PdhhjgYxSJqxWPXfy2Bn0EJydidANFUiQA01YHrr519p73vIdWrVpVZto40fkYGBhQfwd\/xzNtKWZtlarjagXT5o5WPCPAhg0PtWenWdzriv89vPPOO9V\/coM\/Z9dz\/1qKq5UmxaYPM2125w1Mm1Af\/ZBmcKYtOLPG1QRnALBySgg2hbCoWsG0pSCCsMooWvEzUbxghB9TwIyAELDBsCha8X9ksYLeIPyIVUXVCqYtIuAcw2HahPDjXgTC6hFmkAC0Mggz5aqgVcqADVYPrQzCTLkqaJUy4Byrh2kTwq92EXDxSrdHhdUiLAUC0CoFqClVCa1SAptCtdAqBagpVQmtUgJrQbUwbUIRKl0E4Qeiw7dLhVUjzDABaGUYaIrVQasU4RquGloZBppiddAqRbg5Vw3TJhSg0kVQa8sPYbUIS4EAtEoBakpVQquUwKZQLbRKAWpKVUKrlMBaUC1Mm1CEShcBF622ua6wWoSlQABapQA1pSqhVUpgU6gWWqUANaUqoVVKYC2oFqbNAhHQBRAAARAAARAAARCoRwCmrR4hfA8CIAACIAACIAACFhCAabNABHQBBEAABEAABEAABOoRgGmrRwjfgwAIgAAIgAAIgIAFBGDaLBABXQABEAABEAABEACBegRg2uoRwvcgAAIgAAIgAAIgYAEBmDYLREAXQAAEQAAEQAAEQKAeAZi2eoTwPQiAAAiAAAiAAAhYQACmzQIR0AUQAAEQAAEQAAEQqEcApq0eIXwPAiAAAiAAAiAAAhYQgGmzQAR0AQRAAARAAARAAATqEYBpq0cI34MACIAACIAACICABQRg2iwQAV0AARAAARAAARAAgXoEYNrqEcL3IAACYgLnz5+nVatW0eTk5Kwye\/bsoZUrV4rrKkrgkSNH6KWXXqKBgQHauHGjOi3+OXhU+zzMgOPuvPNOLzkWJR9wHiCQhABMWxJ6KAsCIFBGQJu2LVu2wFgQEfP43Oc+R1\/+8pepubk5sWkL14f0AwEQ8IsATJtfeuNsQSBVAjBt5Xh3796tPujp6VF\/J51p4zrCdaYqKCoHARCwigBMm1VyoDMg4DaBeqZtZmaGuru7qaWlhUZHR6mjo0PdKjx58iR1dXXR9PQ0NTU10YEDB6i9vV3BCH63YsUKFbN8+XJlhMImiA3N2NgYDQ0NUWNjo5rpCt6u1bdodT+4rePHj6s6W1tbaWRkRM2IhdvVfeLPN2\/eTHv37lVxup5169bNmlnk7x5++GHq7e0tnYvEtHEMswkemhN\/xrdb9+3bVzpHtzMGvQcBEIhCAKYtCi3EggAI1CRQ7Zm2vr4+ZbK0yeFKwsZK31JlU7Jt2zZloPhg07V69WpVnr\/bsGED6fpqmTYuywZRGzw2f2ygBgcHacGCBeq7qakp1Q4bPG0m2USGzafu09NPP61MmzZpXGfQxAXhVPqukiHTZYLGTH8W7LM2sdy39evX0\/bt20tmEGkJAiDgBwGYNj90xlmCQCYEpDNt2khVmjkKzl7x99rABWe2JDNtp0+fnmWo2DS1tbWpWb2goeN2ggYwaBz1zJsGyLN5ExMTaoYw+HMYcKUZMclMm66nGst6jDMRGo2AAAjkQgCmLRfsaBQEikmgnqHQhixo2tj47Ny5cxYQnk1jgxXXtH3rW99Ss3Lhg2e0tm7dWtO0hW+zVptB27FjR9XVnElMWyVOug+1bskWM6twViAAApoATBtyAQRAwBiBOKat1jNa4RmvsJmpdXu00kxb2PgEzaN0pk334ZOf\/CS9+OKLpZWhJmfaam0BUo+xMTFREQiAgHUEYNqskwQdAgF3CdQzFJVmkMJl9MKD\/v5+uuWWW8pmxMLPtPGM2KFDh8qeS2N6\/LwcH8FboLodfj6u3u3RSn3Sz8Pxs2V6dnDp0qVVFwRUe6aN+1Vrn7Zas3xcFs+0uXt9oOcgkJQATFtSgigPAiBQIhDHtHHh4ApR\/l0vNNAmRa8AXbhwoWrrrrvuUgsTggsfeIVnZ2cnnThxourqUf2wfyXzGJ7dCvcpuDmw\/u7+++8vbecRToM4q0e\/8IUvVNycOGgOsXoUFxwI+EsAps1f7XHmIOAcgVrPemV5MtLZrjT2VNOLKfTeb1meN9oCARDIlwBMW7780ToIgEAEAraYNulsl+k3GJiuLwJ6hIIACFhAAKbNAhHQBRAAARkBG0wbz3QdO3asbAPgWr0PvntUdpbVo\/Du0aQEUR4E3CYA0+a2fug9CIAACIAACICAJwRg2jwRGqcJAiAAAiAAAiDgNgGYNrf1Q+9BAARAAARAAAQ8IQDT5onQOE0QAAEQAAEQAAG3CcC0ua0feg8CIAACIAACIOAJAZg2T4TGaYIACIAACIAACLhNAKbNbf3QexAAARAAARAAAU8IwLR5IjROEwRAAARAAARAwG0CMG1u64fegwAIgAAIgAAIeEIAps0ToXGaIAACIAACIAACbhOAaXNbP\/QeBEAABEAABEDAEwIwbZ4IjdMEARAAARAAARBwm8D\/B0\/00yJqsoTXAAAAAElFTkSuQmCC","height":299,"width":497}}
%---
%[output:926cdc68]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","2.443525700815186e-03","2.362998398567786e-03"]]}}
%---
%[output:72ee0024]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000e+00","-1.901953846588630e+00","9.043571086383212e-01"]]}}
%---
%[output:92c8c0dd]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ares_nom","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-2.526618726678876e+05","-2.513274122871834e+01"]]}}
%---
%[output:8de2fd29]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-2.526618726678876e+01","9.974867258771282e-01"]]}}
%---
%[output:908585c9]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:23a9305b]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:6ea3ed09]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -2.526618726678876e+01"}}
%---
%[output:2c12ffd6]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"     9.974867258771282e-01"}}
%---
%[output:80a7f48a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["1.490161953970131e-01"],["3.652194550246457e+01"]]}}
%---
%[output:7ddfb290]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:424972f3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:780af92e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-9.869604401089360e+00","9.984292036732051e-01"]]}}
%---
%[output:108fa3e8]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["1.555088363526948e-01"],["2.716608611399846e+01"]]}}
%---
%[output:63a32b6a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["3.719106107041118e-02"],["1.937144673860303e+00"]]}}
%---
%[output:064f1b7e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAm0AAAF2CAYAAAA88q97AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQm4FsWV9w+KIMouGGQJENyuG2Jco6JGBeMWdBSRIBqCsmkWZJOQuAR1ECSuLMIgIsoSVNSIAXV0UFwQA+gYUEGIQQRB9oCAyvedYvraNP2+XdVd1aff2\/9+Hh6UW3VO9e9UV\/1vrZV27dq1i\/CAAAiAAAiAAAiAAAhkmkAliLZMxweFAwEQAAEQAAEQAAFFAKINFQEEQAAEQAAEQAAESoAARFsJBAlFBAEQAAEQAAEQAAGINtQBEAABEAABEAABECgBAhBtJRAkFBEEQAAEQAAEQAAEINpQB0AABEAABEAABECgBAhAtJVAkFBEEAABEAABEAABEIBoQx0AAUECs2bNou7du1ONGjVowoQJ1LJly9RLs2XLFuratSvNnTuX2rVrR8OHDy8vA5dv8ODBNHXqVGrQoEGqZVu4cCF17tyZNm\/eTKNGjaI2bdoo\/155W7duTT179ky1TFHOevfuTdOnT9+jvFF5vJ\/beK8RI0bQ7NmzaezYsVS9enVd19bS+WPGRhs3bpyo7hSrm9YKnYIhj0tZWZlYbFJ4TbhIgQBEWwqQ4QIEChHIsmhjATBs2LDEHW\/c6IeJtlWrVlH79u1pxYoV1KdPn0yJNq+8tWrVMhYqNt7LE4wnn3yyiDDwCywv5knLUlFEm\/89\/L+AxP02kC+\/BCDa8ht7vDkIFCUA0WZWQTxewdFKHSsVTbTFYRDGqaKINn63JPVDpw4hTT4IQLTlI84l+ZbeKJRX+LApRP\/owlVXXUU333xz+bsWGonx8ngJg+mCHcUFF1ygpjALpS8E198RF8obNtLmf6dWrVrR6NGjVXZ\/Ob0OwLMbnIYqJLj8TL3f+IPve8cdd5RPl\/rfrdCoSaERHv\/7B0cXdGIbHGnjsvjj4JXNbzsYW04TNrIRnMbjNEuWLAkdWdSZ8jN5Vy6TX9QEWZi+V1g9C\/rQeYdijURUvIL2db+VsHxhU+He1L3Otxj8NoLfDv+\/zjfmr0tc9++66y667rrrQkd5o9oU9um9K\/+31FKIkuwIUOg9CEC0oUJkkkBY5xvWERRLF+y4wqZvPJv+TrRYuiSdUZivYqLNHxi\/YC30zv40aYq2QlO83r8HBaVubE1FWzG7fiFQSCSFCeBCaYO\/QEQxCPvIvDoXJdqi3uu4444rnzL2+4myr7uOUidecURbsTh4v6DofIv+2IYJNt12w+PRokWL0F9a\/Gx1yhccbbQxmprJxhqFSpUARFuquOFMh4C\/MfePLnmdRyEB428kw9J6Dbo\/f1hnG+wovE7B3zEVW6vjz+8XLGFlihJtwVHAMDb+cnkMkog2byOC7vRoWGdUaFrLJLYma9rCRjH85fK4FIqNv1xezLiueuvnwvL761shVmGjkGH1sFCHrvtewdEjbyNCFIOoaUyTeJlMZfrL5X1L\/A7ehhgvBryZwvs3\/rn3LYa9V9hop79M\/m\/WL0R1vrFgm+Dl0W1TuOwmfHTaSaTJJwGItnzGPdNvrTPd5jWaXtrgaE6wE+RdiGE7JP0Nadhvz0FxprPYu9Cux7CdmMVEW9jOuzD\/YbsO0xRtYYJh6dKloTs\/TWJrItqCFTo44uKJE7\/NYGcdrEvvv\/9+6M7esBHEQu\/lFwfFBJLuKEyh9yok2qJGAKN2d5rEy0SUFCpXcPdrIdFV6H399SA4khcm2op9Y8GfBeuOSZvilUun\/ch044zCiROAaBMPAQrgJ1Cs4Q\/7WaFGMJi2X79+oVNIYdNJxcqg0+gWEm1hkY5a0xY8ukHHP\/tJW7QFR4TmzJmz1\/ow09iairZiU2Nhoi241i3I7Omnn1bvUOgJm04LCrOwacOwacliok3nvQqJmGJ5OU+xKVLTeNkQbUHWpt9isSnXMNEWNmKuK1Qvv\/xy7TbFey\/d0Wv0CiBQiABEG+pGpgiYdhQQbeFncaUt2vxx69atG82fP3+vc99MY2si2vydtSdEfvCDH+w1vVlMULsQbd7HFSYm\/CM5hUSb7ntBtI0l\/+gu82BR\/pOf\/KR8hB2iLVNNPQoTkwBEW0xwyOaOgO5vunywatLp0bC3MP3tPmij0BSc9+9Dhw4tPyg27khb2OL+lStXlp\/PlbZo84\/uNWrUiDZt2qSwBHfJmcTWRLR57+vvmMPWPdmYHjUZDQqrX2FlKCTadN+rkGgrNA2p+\/WaxCvOSJsnrryDk7m8ffv2La83Jt\/im2++qaaz\/d9G1Jq2YiNtcadHi7ENi6duLJAOBJgARBvqQeYImCx+LrRmSHcjQpgwMOkowk6dL7TY3T9V5U3NmYq2MDZhi7q9DoyD663dCh4NUejID9ONCF4FCk4FhnWIJrGNI9rCdtBy+WxtRCgkjoqtNeTjKqLEZJRoi3qvQuUKE66F0oY1BCbxMhFtYXWWv6Xgd+vfyRmceg4y99f54PfF76Y70hb2ziYbEYqN5uoub8hco4wCZYYARFtmQoGC+AlEHXPg\/WZeLJ2\/s+b\/LnSeVbBBTyra2F6hIxCCvkxFm7\/DDasxYTtdC9WsKNFWbCF3mM1CHVswrW5sowS1Z5ffg6dCvSuvwsqmcy7akUceSYsXL95jpKbYmrCwoyaCozPF1lj5hViQHb+D6XsV2qSg+w6F6oluvExEG\/sqxiZqt26Y8GT\/3m7fsHfRFW1hsWB73ggyX6tW6Bchv9\/gLy2mfNAjgEAYAYg21IvMEgg26lGH6\/7617+mHj16qLsq+dE9XDf4G7wN0VZIJIbd7Rm8e1Tnt\/FgRxzGJmzky38AcZRoC3ZeUTsN\/R1x1BlgOrEttgs37LDjoE3dA3O9soZtnggT4MVYc\/rglHDYLwthLIPl9+qv7nsF\/fhFQ7AuRMUn2CjoxCuOKAn75cb\/3Zp+i0F7bOvQQw\/daxewzjcWHMX3b2YqtPPY4xa2U7jYAcyZbYRRsMwRgGjLXEhQIBMCOo2viT2kzS+BJDv70CFX\/HqjeyxLIRJhZ+5VfGp4Q9sEKrxo44Z48uTJkRc4F5rOSnrhse2Awd6eBCDaUCNMCPg73kIL1qMOnC3kzxN9cfObvAfSuiPgH8X2j6gl3dSB+uEuZnmyXKFFmyfEatWqFSnagruW8lQJSvldIdpKOXoyZY9aBxl2V6lOSU3aGx17SCNDoNh6VC5RnF\/k\/dO8ceuXDA14zRqBCivaCu3gK\/Zb8uzZs8uPTMhaoFCecAIQbagZcQiELc43XecV5terj+iY40QlO3kKbVqKO4rqCcGysjL0MdkJc0mWpMKKNu86lFatWtELL7wQOdLGjS0\/3nEHJRlNFBoEQAAEQAAEQKDCEqiQos0\/1ck7wqLWtHm\/VfE5Qbzl33vi\/lZVYWsLXgwEQAAEQAAEQECMQIUTbZ4A69ChA\/GJ+TobEbyh63PPPbd8pM2z07Bhw8jh7BUrVogFEI5BAARAAARAIM8EeFNRXp4KJ9p4mjN4nU\/USFuhYHu7hYqtT2HBxteuvPPOO3mpM3hPEAABEAABEMgMgVNOOYX4esA8iLcKJdrCdoDqjLQVqnneCBxfgM2jdmHP22+\/TR07dlQVhk\/MxpMuARbL999\/P\/ini73cG\/gLgfe5RQxkYwD+2eDPGwkh2mRjYew9ait\/oRPybYi2vFQY46A4zuCJZvB3DLqAefCX4e73ihjIxgD8wT9NAhVqpC0MnM5IW6FpUJ2z2\/DBplld9\/YF\/uAvS0DeO74B2RiAP\/inSQCi7f\/fUemd6cZr4aZOnUoNGjQov\/Dbvzmh2PQoRnrSrLbf+1q+fDmNHz+eBg0aRJUrV5YpRI69gr988BED2RiAvyz\/vInmXIo2Hn0bPXr0Xhc7B6dXdaZT81ZhZD\/Pvb1\/\/fXX9MUXX1CTJk0g2gSCA\/4C0AMuEQPZGIC\/LP+89cEVXrS5rk55qzCueZraR4NpSsxuevC3yzOONcQgDjV7ecDfHss4lvLWB0O0xaklvjx5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCTMWx8M0ZawGuWtwiTEZT07GkzrSI0Mgr8RLieJEQMnWLWNgr82KicJ89YHQ7QlrEZ5qzAJcVnPjgbTOlIjg+BvhMtJYsTACVZto+CvjcpJwrz1wRBtCatR3ipMQlzWs6PBtI7UyCD4G+FykhgxcIJV2yj4a6NykjBvfTBEW8JqlLcKkxCX9exoMK0jNTII\/ka4nCRGDJxg1TYK\/tqonCSc9O4q+t2QsTTvvuuocePGTnxkyShEW8JoQLQlBJgwOxrMhAATZgf\/hAAtZEcMLEBMYAL8E8CzkPWSh+fTnKUb6P3eh0G0WeBZ4U1AtMmGGA0m+MsSkPeOb0A2BuAvyx+iTZZ\/yXmHaJMNGRpM8JclIO8d34BsDMBflj9Emyz\/kvMO0SYbMjSY4C9LQN47vgHZGIC\/LH+INln+Jecdok02ZGgwwV+WgLx3fAOyMQB\/Wf4QbbL8S847RJtsyNBggr8sAXnv+AZkYwD+svyPH\/wWfbbua2xEkA1D6XiHaJONFRpM8JclIO8d34BsDMBflj9Em0P+GzdupJEjRxL\/HfepVasWDRgwIG526\/kg2qwjNTKIBtMIl\/XE4G8dqbFBxMAYmdUM4G8Vp7ExiDZjZPoZVq1aRe3bt6cVK1boZwqk5MPzZs+eHTu\/7YwQbbaJmtlDg2nGy3Zq8LdN1NweYmDOzGYO8LdJ09wWRJs5M+0cnmgbNGgQtWnTRjufl3DWrFk0ePBgiDZjchU3AxpM2diCvyx\/9o4YyMYA\/GX51+39qioADtd1EIc1a9bQjTfeqP6ceeaZxh5ef\/11euihh2jKlCnGeV1lwEibK7J6dtFg6nFylQr8XZHVt4sY6LNykRL8XVDVtwnRps\/KOOW3335L\/KdKlSrGebOaAaJNNjJoMMFfloC8d3wDsjEAf1n+EG0O+fP0aMeOHalJkybUs2dPOvHEE2nfffd16NG9aYg294yLeUCDCf6yBOS94xuQjQH4y\/KHaHPIn3eN\/ulPf6K\/\/vWvtGPHDqpfvz5dd9111KFDB6pTp45Dz+5MQ7S5Y6tjGQ2mDiV3acDfHVtdy4iBLik36cDfDVcdq3w+G29E4Adr2nSIxUyzZcsWevXVV2ncuHH0\/vvvKyvHHXcc9ejRg84+++ySmj6FaItZCSxlQ4NpCWRMM+AfE5zFbIiBRZgxTIF\/DGiWskC0WQJpYoY3J0ybNo3Gjx9P\/N\/Vq1enK6+8krp06UKNGjUyMSWSFqJNBHu5UzSY4C9LQN47vgHZGIC\/HH+INjn2tGvXLvr4449pzJgx9Le\/\/Y22bt1KzZs3V6Nvl156aWZH3yDaBCsNjjuQhQ\/+4vy5ABANsmEAf1n+WNMmy1955\/Vub731Fv3hD39Q\/z916lRq0KBBqiUbMWIETZ48OdI3RFuqYdnLGRpM8JclIO8d34BsDMBfjv8bSzbQpSPmqwJgTZtAHDyxxlOlLNr4eBA+hPeuu+4ivr4qrWfhwoXUuXNn5TNKMEK0pRWVcD9oMMFfloC8d3wDsjEAfzn+EG0C7FmYLViwgCZNmlQ+LSq5q5Q3SXTt2pXmzp1LfGUWRJtApTBwiQbTAJaDpODvAKqhScTAEJjl5OBvGaiBuSEzl9GQmcsx0mbALFZSXr\/G94\/yiNpzzz1HX331lVqzduGFF1K3bt3o8MMPp0qVKsWynTQTT4vy3aatWrWiF154AaItKVDH+dFgOgYcYR78Zfmzd8RANgbgL8cfos0x+23bttHjjz9OEyZMoJUrVyphVlZWps5qu+CCC9TOUcmH7zbt27evKt+cOXOM1rQ9+eSTamSOn7TX30kyk\/aNBlM2AuAvyx+iDfzlCaRfAj6on5\/eM76ilxevw0ibqxB4F8Zv2LAhc8d6eGXjg375tgbTjQh+Zrwe7tprr3WFEXZ9BLZv366OimGhXLlyZbBJmQD4pww8xB1iIBsD8E+f\/2OPPaYGV7ac0Y++qXcERJurEPBI2\/Lly6lFixaZO8Kjd+\/eavRv7NixasTPVLQNHTq0\/Fw5FhAYbXNVi\/a0y3Vq9erVapQToi0d5n4v4J8+86BHxEA2BuCfPn8eZOE\/F07dVu4cu0cdxMEbzRo0aJDaFWr68PTl4MGD1Zozm49\/WrRly5bKtKlo4zJ506M2ywZbxQlgek62hoC\/LH\/2jhjIxgD8Zfj7D9blEkC0OYhDVkUbj7JNnz694Bv36dNHTZmGPTjyw0FFMTCJBtMAloOk4O8AqqFJxMAQmOXk4G8ZqKa5Se+uol6TFqnU+2xdSwsGnZaLgZNKu3grZ0qPJ9p452jch0ezbI+0hZUFI21xI5RuPjSY6fIOegN\/Wf4YaQN\/eQIyJbj6v96nmR9+pZxXXvsR\/f2uiyHabIdi48aNNHLkSOK\/4z584O2AAQPiZtfOB9GmjUo0IUSDKH5MzcniV97xDcgGAfzT5x+cGt1\/8XM095GbIdrSD0V2PEK0ZScWxUqCBlM2TuAvyx+iDfzlCaRfAv9NCOy9+hv30JtPj4FoSz8UpecRa9pkYwbRAP6yBOS94xuQjQH4p8\/\/kofn05ylG5RjXs9Wc1Z\/tWwqD5sBU13Tln5o3XuEaHPPGCNtsozBP7v8MdImHxuItnRjEJwaPavWalr42ECItnTDULreINpkY4cGE\/xlCch7xzcgGwPwT5d\/u5ELaPYn65XTH9bdn0adV4k6duwI0ZZuGErXG0SbbOzQYIK\/LAF57\/gGZGMA\/unxD65l+915Ten8ul9CtKUXgtL3BNEmG0M0mOAvS0DeO74B2RiAfzr8eVr00hHzif\/2Rtn4bLa89cGZWNPGR8WtX7+evv32W6pbty7ts88+tHPnzsxddRVWNfNWYdL5PPW9oMHUZ+UiJfi7oGpmEzEw42U7NfjbJrq3PRZqfJCut\/mAUzx8dRldfVIDiDb3+L\/3wGLtpZdeoj\/84Q\/q0m\/e+TF16lSqWrUq\/epXv6ITTzyR+DaCKlWqpFksI18QbUa4rCdGg2kdqZFB8DfC5SQxYuAEq7ZR8NdGFSshC7a\/vLea7nzx0\/L8153WkIZfufui+Lz1waIjbf\/zP\/9D3bp1oxNOOIEOO+wwevXVV5Vo4wvbb7nlFpo5cybxPaWdO3eOFew0MuWtwqTB1MQHGkwTWvbTgr99pqYWEQNTYnbTg79dnn5rLNimzFtFd\/9tWfk\/8+YDnhb1nrz1wWKibfv27dS9e3f67rvvaNSoUfT666+ry+BZtDVo0IB27NhBfCfo2rVraezYsUrIZfHJW4XJWgzQYMpGBPxl+bN3xEA2BuDvhn9wDRt7YcH2XM9W6m+INjfcC1r17iG94YYbqFOnTjRr1qw9RBtnnDhxIj3yyCPlQi7lImq5g2jTwuQsERpMZ2i1DIO\/FianiRADp3gjjYN\/JCLjBMFdooUEG\/973vpgsZE2T7Rdc801dP3114eKNr5Katq0aTRlyhSqX7++ceDTyJC3CpMGUxMfaDBNaNlPC\/72mZpaRAxMidlND\/72eKr1a39fTXfO+H79WjHBBtFmj32kJW96dOvWrTRmzBillv3ToytWrKDrrruOmjRpoqZPeXNCFh+INtmooMEEf1kC8t7xDcjGAPzt8A8bXWPLd7c7jLq1blzQSd76YLGRNo7A\/PnziadHeRMC\/5kxYwb99re\/pSVLltBTTz2l1rWNHj2azjrrLDu1woGVvFUYBwgTmUSDmQhf4szgnxhhYgOIQWKEiQyAf3x8PLLGx3gMmbms\/Pw1zxqvW3uoQxmdcWjtog7y1geLijaOxJtvvqmO\/Fi27PvdIfzvPB1655130nnnnRe\/RqSQM28VJgWkRi7QYBrhsp4Y\/K0jNTaIGBgjs5oB\/M1xslib9O4XNOndVXuJNbb2u3Ob0rWnNdxjw0EhL3nrg8VFGweCz2vjXaL\/+Mc\/iKdNjz76aLWDdN999zWvDSnnyFuFSRlvpDs0mJGInCYAf6d4tYwjBlqYnCUCfzO0k+etop5PLgrNFLY7NMp63vrgTIi2qKBk+ed5qzBZiwUaTNmIgL8sf\/aOGMjGAPyj+fN6NZ4C9d9o4M+lOxUa5ilvfbCYaNu4cSONHDmS+O+op169emqa9Jhjjsnc6FveKkxUrNL+ORrMtInv6Q\/8ZflDtIG\/PIG9S8DTn\/ynmFDjXEnEmuc1b32wmGjja6tuvPFG+vDDD4l3kPLD4owfnioNe84880x68MEHqWbNmpmpp3mrMJkB\/38FgWiQjQj4y\/KHaAN\/eQLflyBqRM0TalefdIi6N9R\/SG7c98hbHywm2jhAc+fOpV69eqlrqn75y1+W33rAu0b\/8pe\/EJ\/T9tBDD9FRRx1Fzz33nDoShNPefPPNceNrPV\/eKox1gAkNQjQkBJgwO\/gnBGghO2JgAWICE3nnH3aZexhOG6NqYXbz1geLibYtW7ZQ165d6fDDD6fbb7+dKlWqtEc8eHPCrbfeSv\/85z\/VOW3VqlWjIUOGqPPcnnnmmQSfmN2seaswduklt5b3BjM5wWQWwD8ZPxu5EQMbFOPbyBN\/b9rT2\/kZRY2FWv+2zen0FrWtjKpBtBGJiTbvRgQeabvqqqtCY883ITz88MPl11g9\/fTTdN9999Hs2bOj6kpqP4doSw11qKM8NZiypMO9g798VBAD2RhUdP485Tnt76tpwtsrtUB7I2r8t43pzyineeuDxUTb+vXrqUuXLtSiRQu66667qEqVKnvEhqdIBw4cSEuXLqVx48ZRnTp11Ho2PoD3xRdfjIpjaj\/PW4VJDaymo4reYGpiEEsG\/mLoyx0jBrIxqGj8dac7PeoszNoedRBdctzBkQfhuohU3vpgMdHGwePL4IcNG0ZXXnml2pTAZ7Pxw6NwvJZt8uTJav1ajx491Pq3W265RZ3hxuItK0\/eKkxWuHvlqGgNZtb4RpUH\/KMIuf85YuCecTEPpcyfBdr4t1bSu8sBvpR9AAAgAElEQVQ3FjyOI\/juLNKa1Nk97ZnWaFox\/nnrg0VFG4+msWh79NFH6dtvv90jLnywLm9O6NOnjzqHiC+VX7lypRJ6ZWVlsl+pz3veKkxmwP9fQUq5wcwayzjlAf841OzmQQzs8jS1Vir8eZrzs3Xb1C0Ehc5LC3v3rIm0YBnz1geLijYPPh\/\/8corr9DChQvVP\/Fo2jnnnEONGjVS\/79t2zY1+tawYcPMXRyftwpj2qC5Tl8qDaZrDlL2wV+K\/Pd+EQPZGGSJP4+c8cN\/z\/jfNfTB51uMBBrnZZHGGwfOOLSO0w0EtqKWtz44E6LNVvAk7OStwkgwLuYzSw1m1tikUR7wT4NycR+IgWwMJPnz6Nku2kVDZy1XQs0TbbpEvI0CAy5oTo1r7944kMbmAd3y6aTLWx8sKtr4WI9PP\/2Upk+fHnqgLt9D+vnnn9P9999fvt5NJ4hppslbhUmTrY4vyQZTp3wVPQ34y0cYMZCNgUv+\/pEzfssp81bR8q+2GY+eeSNo\/HfnUxvSyc1qlaRAC4t03vpgUdHGO0F79+5NvLYt7OEdpSeeeKLaeMC7R7P45K3CZC0GLhvMrL1rFssD\/vJRQQxkY2CTP4+c8fPU31fTkjVbY4kzT6DxZoGfHV2PLj6uvrJZaiNoulHNWx8sJtr+\/e9\/U7du3dTmggceeICaNm2qNhu0atVK3ZIwadIkGj9+vDpY99hjj9WNX+rp8lZhUgcc4dBmg5m1dyuF8oC\/fJQQA9kYmPD3j5yt3LidHn97Jf1rvfm0pv+NWYzx+rMOJ+4+faEUpziTRDBvfbCYaPMO1+XjPm666SYVs9tuu42+\/PJLNR3KD4\/CVa1alYYOHbrXjQlJgmwzb94qjE12NmyZNJg2\/MHGngTAX75GIAayMQjj760v++a77+iZ+V\/Sp2vjTWkGxRmPnl3a8mAqa3Bg7sRZoSjnrQ8WF22\/\/e1v6fLLL1fxmDhxIj3\/\/PM0ZswYdSk8\/\/8TTzxBjz\/+ePll8rKf597e81ZhssYfHZZsRMBflj97RwzSj4F\/xGzp2q303x98Tut2VI49nRkUZzxa9utzfkj777cvxFlEePPWB4uJtk2bNqnp0BNOOIH69++vwvLyyy+r0TYWa82aNVPijQXb1KlTsREh\/XapJDyiw5INE\/jL8odoc8vfGzH7cvN2evTN3dc4mZxxVqh03vqyC4+pRxce8\/2as4q67sxllCDaXNIN2L733nvVwbr9+vWjDh06qPVtnTp1Ujck\/OxnP6MBAwYQ7zD1rrFKsWjarvJWYbTBpJQQoiEl0AXcgL8sf4i2ZPz9I2Z\/eW+VmsZMusbMK5EnwM46vA5dfGx9qoZRs2TBKpA7b32w2Egb8+fRNl7Pxn+zMKtduzYNHjxYbUBgsVapUiW69dZbqXPnzk6CbcNo3iqMDWY2bUA02KRpbgv8zZnZzoEYFCfqCTO+9PzVj9ZZE2XslYXZN998Qz9utD9ddeoPqWa1qpjOtF3BI+zlrQ8WFW0cCxZnmzdvpho1aiiRxtdZvfnmm\/T6669T27Zt1fQp\/3tWn7xVmKzFAR2WbETAX5Z\/3kfa\/AfKvvXpBpr9yXqroswTZvw3X4p+0bH1aZ9KlfYQZvgGZL+BvPXB4qKtWLhZwG3cuJFq1apFfBdpFp+8VZisxQANpmxEwF+Wf0UVbf6T\/fm\/+cyy\/\/18C320+t\/ORBnvzDzzsDr0kx\/VVkHVPToD34DsN5C3PlhMtHlHfgwaNIjatGkTGvUXXniBhgwZgo0Ist9Epr2jwZQND\/jL8i910eYdJvvB55tpxv+uVTBtLPQPRsW79Lz1YXXoNENRFhVhfANRhNz+HKLNIV++loqnPrdu3UobNmyg++67jy677DJq2bLlXl45La9z47RTpkyh+vV377DJ2pO3CpM1\/mgwZSMC\/rL8syra\/NOWX2zcTq99vE7di2lrkb+furfgn0fKjmtUg45pVJ34v71\/d70jE9+A7DeQtz449ZG2CRMm0O23367WskU9PCXat29fdTRIVte15a3CRMUs7Z+jwUyb+J7+wF+Wf9qiLXgX5qpN2+l\/Pl6v7sN0Icj4\/fyi7HfnNqUqlfdR0HWnL11HCN+Aa8LF7eetD05dtPE9o7xObfXq1dS9e3f6zW9+Q2eeeeZeUeF7R\/m+0ayKNa\/Aeaswsp\/n3t7RYMpGBPxl+bsQbd4o2cer\/03PLPhSvaCLKcugIDu0\/gH046Y1iacwvcf1KJmN6OEbsEExvo289cGpizYvNLzJYN26dermA76qqlSfvFWYrMUJDaZsRMBflr+uaPNPV363a5earnx3+SangswTZd5U5fFNaqrrl\/xirRREWVSE8Q1EEXL787z1wamKNk+o8d+6D0+R1q1bF7tHdYHlLB0aTNmAg78sf\/b+8coN9P6nX1C9evXpzWWb6fMNuy8gdzVd6R8FY0HWvF41ant0Paq1f+VyQVYRxJhuZPEN6JJykw6izQ1XZdXbMbpixQptL40bN8buUW1a+UuIBlM25uBvn7+3boyFj\/ffqzftoCfmfkFL12xNRYzxW7Ega3ZQNTq+SQ06v+ygPV40T6IsKsL4BqIIuf05RJtDvtu2baM5c+YQ7wzVfXjq9PTTT6dq1arpZkk1Xd4qTKpwNZyhwdSA5DAJ+MeD6x11sXHbTnp58Tpa8qV7McYl9S\/qZzF21CHp7rSMRyvbufANyMYnb31wqtOjsqF14z1vFcYNxfhW0WDGZ2cjJ\/h\/T9G\/bmz15u309qcbafEq+4fBhsWtYc3KVLlyZTU6dmSDA+m8soPowCq7DyTPyi5LG\/UtizbwDchGJW99cCZE2\/r16+mNN96g9957j3jXaKtWrejUU09Vu0ez\/uStwmQtHmgwZSNS0fkHT+bnuyv5mIs01ox5kfUOhuW1Y3wwLAsz\/6jZwQcQffHFF9SkSRMl3PCkS6CifwPp0jT3lrc+WFS08YaERx55hIYPH67uHPU\/vAGhV69e1LNnTyXksvrkrcJkLQ5oMGUjUmr8gyKMBRFfkfTax+tp\/mfud1P6o+WJMf67Ue3dC\/r9h8L6hVmxKJdaDGRrrH3v4G+fqYnFvPXBoqJt2rRp1L9\/f2rdujXdfPPN1KxZMxWr5cuX07333qtuTxg6dChdeumlJjFMNW3eKkyqcDWcocHUgOQwSRb4sxDz1mr5D3\/95MutNO+fG1MfFWPcnvg6vUVt+mHd3etxXU1TZiEGDqtY5k2Dv2yI8tYHi4m2LVu2UNeuXal69er04IMP7rXRgDct3HTTTWoEbtSoUZk9yy1vFUb289zbOxpM2Yikwd+\/Vmzd1p00++P1Ti4OL0bSv4Cfzxo7+4i6VNN3xIXuqJiLaKURAxflrig2wV82knnrg8VEm3f8xw033ECdOnUKjfrEiRPV9OnUqVOpQYMGsjWjgPe8VZisBQENpmxE4vL3C7F\/7\/iWZn64lnhkjB\/X54v5ifnF2FmH16GTm9WifSpVKh8ZkxRjupGNGwNd+0hXnAD4y9aQvPXBYqJtzZo1dNVVV6kL43lELezhEbhnnnkGF8bLfhOZ9o4GUzY8fv4rN31TXphZ\/\/iKPvxiizrKQlKIsSj7SYvaSojFWS8mS1fPO74BPU6uUoG\/K7J6diHa9DglTrVz507q3bs3LVq0iMaMGUPNmzffw+ayZcvURfFlZWVqo8J+++2X2KcLA3mrMC4YJrGJBjMJvei8wRGx2Z+sp\/dXbBYXYofUqkot6h+w107KPB76im8guh67TAH+LulG285bHyw20sah+OCDD6hLly7qsN3zzz9fHaLLDx\/A+9JLL6l1bDw9ykeAZPXJW4XJWhzQYOpFxL9r0n\/S\/hcbt9PU91YTXw6uvr2lG\/QMWkoVnJ48tTkv3N99E4CrhfuWip4ZM\/gGZEMB\/rL889YHi4o2DvVHH31EAwcOpAULFtCuXbtU9CtVqkTHH3883XXXXXTEEUfI1ogI73mrMFkLBhrM3RHxj4h9tPrfNP9fm+mfX21LdX2YVzf8Qox3T3rTk\/xz72d5HBFz9e3gG3BFVs8u+OtxcpUqb32wuGjzAsmjbXzILj98qC6PspXCk7cKk7WYVNQG0y\/Cdnz7Hb25dAO9s2yjwp\/mQn2\/0OL\/5nVhZx9eh05sWov23acSff31Nqr27RY65JBD6EcHV89a9chFeSrqN1AqwQN\/2UjlrQ8WE228EWHcuHHqDLbDDz+c+DDdUnzyVmGyFqNSaTD9Imz3yNg2NRqW1jVHwbj5R8N4bVjtAyrTeUfWVWeK+c89ixoRKxX+Wau3NsuDGNikaW4L\/M2Z2cyRtz5YVLTx7lE+SPeggw5S4u26666jxo0bq+nRUnnyVmGyFhfJBtMvxDZv\/4ZeWbROnR8mMRrmHxHzdkmyGOMF+8Frj6KEmEmMJfmblLMip0UMZKML\/rL889YHi4k2DvOOHTvorbfeovHjx6u\/+f95F6l3FEj9+vVla4OG97xVGA0kqSax2WD6T9Pnl5izdD2t2rhDXXOU9pSkB9E\/InZEgwPpmEOq06F82aTDE\/ZNAmiTv4lfpP2eAGIgWxvAX5Z\/3vpgUdHmD3VQwPGRIHzcB+8uvfjiizN7\/2jeKozs57m392INpl+Esfj59rtd9Je\/r6bXP9m9dlJCiPlFWNODqtGJTWvSofUPKNkdk+iw5L8IxEA2BuAvyz9vfXBmRJs\/7Js2baJHH31UjcDVqFEDNyLIfhOZ9O4Jso9XrqfX\/rGaFq7ZvfNYWoidcSgv0q9JVSvvUz4a5o2KZRJkwkKhw0oI0EJ2xMACxAQmwD8BPAtZIdosQIxjgu8iffXVV5VA4yDwc9xxx6mRNj7DrUqVKnHMOs+TtwrjGugbS3afE8ajYq9+vI7e++cm9f9pnh\/mHw1rdlA1OqlZTfpRvd1Tkn4BZnNtmGuuruyjw3JFVt8uYqDPykVK8HdBVd9m3vpg0ZG2oFDjy+F5Tdsvf\/lLuuiii9TRH3Eevmlh+vTp5Vn5wvk2bdoUNbVw4ULq3Lkzbd68+7R37zn55JNp7Nix6mL7sCdvFSZOPDhPcL3Y9AVfpnrpt1+ItWpSk37SohZVr1q5\/NywoCCL+555y4cOSz7iiIFsDMBfln\/e+mAx0eZdGL9ixQriDQfXXHONuoe0UaNGiWoAC7Z58+aVT6nOmjWLunfvTn369KGePXsWtM3p+vbtSxMmTKCWLVtqlyFvFSYKjLej8uud3xILs3+u+9rZKBkLsW+++YYOqVGZjm5Sl84vq0u1qu2+7sw\/CoYRsaioxf85Oqz47GzlRAxskYxnB\/zjcbOVK299sJho44N0J02aRG3btqUf\/ehHVo758EbLhg4dusfIGgu5lStXFh0xGzFiBM2ePbtoGoy07UmABRpPY\/56ymKra8n8o2ItG9egoxtWL3hsBRpMW01fPDvgH4+bzVyIgU2a5rbA35yZzRwQbTZpZsSWjmjjNPzw5fQmTx4qjH9qc8jMZYlGzvyCrO1RB9GlLQ9OdM8kGkyT2mo\/LfjbZ2pqETEwJWY3Pfjb5WlqLQ99sJ+J2EibaWDipucRtGHDhlGxdW3eVC2vW1u8eHG5q3bt2kWKOK\/CPPnkk+pgYH4aNGgQt7iZybdy0zdKTMURaQ1rVlbvcc7hdand8Qer645c3TmJBlO2yoC\/LH\/2jhjIxgD80+fPfbb38BKrjh07qpkyrw9Ov0Tpeaywos1by8Yoo8SXN6167rnnlos0T8g1bNhQayOCP2S8oeHaa69NL4qWPCmhtmEnjZu3kd77\/GstqyzQftxof7qkbPdGDV5f5ok2LQMJE\/GdtXwlGgvlypV3i0U86REA\/\/RYF\/KEGMjGAPzT5\/\/YY4+p9ef+B6It\/Tg48cg7VLt27arWtPFxIiajYJ7wKzZK54208To6bxMF+zDx4+TFNY3u3jiwjYbMXB4p1Hi0jP\/0\/mkT9Xea4qzQ62zbto1Wr16tfsOCaNMMusVk4G8RZkxTiEFMcJaygb8lkAZmeFDFG21755136P7778dImwG\/zCf1RtK6detWdAdp8EV08pXqfDqLtRn\/u5YGTv+kYPxYmPG9lf3bNlciLYu7MDE1Ifv5gb8sf\/aOGMjGAPxl+ZdqHxyXmtj06MaNG2nkyJHqiqpjjjkmtPzvvfce3XfffWrKMsk9pDriK6wAOvlKrcKwWBv20nKa+M4XocxZmJ11eB268oQGdMahtePWq9TyocFMDXWoI\/CX5Q\/RBv7yBGRLUGp9cFJaYqLNWzM2aNCg0INv+aBd3kDwwgsvaE9rFjprLWqas9DPdc5uK5UKw2Ltqfmr6U8vfBpaZ05vUVuNqJWCUPO\/AERD0iYgWX7wT8bPRm7EwAbF+DbAPz47GzlLpQ+28a5sI1XRxpfCDxw4kJ5++mnt8v\/0pz+lBx98kKpVqxaZx1u\/xgm9Wwy80TK+fL7QzQZh697CNieEFSDrFcY77PbSEfP3Kj6Pqv3q9EbENwSUmljzXgYNZuRn4TQB+DvFq2UcMdDC5CwR+DtDq2U4632w1ksYJEpVtHG5li1bRrzzgw\/X5btGjz\/++NBbEPii+FatWtEZZ5yhLo03eYLXWAVvQ+BjQEaPHr3X7QdR+UpNtLFgY7HmnbPmlZ\/FWr82zajjyYeYYM1kWjSYsmEBf1n+7B0xkI0B+Mvyh2hLib\/OmraUipLITVYrDF+8HhxdY7F2+yUt1MhaFjcVxAkEGsw41OzlAX97LONaQgzikrOTD\/ztcIxrJat9cNz3icqX+khbVIFK7edZqzA8qjbt76tp8Iw9166xSHuuZ6sKI9a8eoIGU\/aLAX9Z\/hhpA395ArIlyFof7JpGqqLNG13jl\/rFL35BTzzxBPG\/FXtq1apFPXr0IP47i0+WKkzYdCiLtYc6lJXsmrWomEM0RBFy+3Pwd8tXxzpioEPJXRrwd8dWx3KW+mCd8iZNk6po83aMcqF5c8FNN91EfAVFsYcPTTU9FDcpFJP8WakwhQRbRRxd88cHDaZJbbWfFvztMzW1iBiYErObHvzt8jS1lpU+2LTccdOnKtriFjLL+bJQYfIq2LheoMGU\/TrAX5Y\/vgHwlycgW4Is9MFpEoBoS0hbusKECbZOpxxCD1x1ZMI3K43sEA2ycQJ\/Wf4QbeAvT0C2BNJ9cNpvLybavPVtWNMWP+Rhgu3Onx9KPc5qEt9oieWEaJANGPjL8odoA395ArIlgGhLib+3vg1r2uIBZ8HWa9IimrN0Q7mBbq0b093tDotnsERzQTTIBg78ZflDtIG\/PAHZEkC0yfKnXbt20dq1a2ny5Mn07LPPqrtHC91NKlxU5V6qwoya\/S8aOH1JOQK+hur5Xq2ygCTVMkA0pIp7L2fgL8sfog385QnIlkCqD5Z6a7Hp0agXZvF26623qpsT+ML4\/fbbLyqLyM8lKgyPsh0\/+K3y9+VjPRYMOk3k\/aWdQjTIRgD8ZflDtIG\/PAHZEkj0wZJvnFnRxlCmTJlCDz\/8MI788NWQMMFW0Y\/1KPaBQDRINh\/YvStLf7d3fAOyUQB\/Wf4QbbL8y717l8t\/+OGH9Pjjj1O9evUyUrI9i5FmhQlbx8YjbBXlSqo4AUaDGYeavTzgb49lXEuIQVxydvKBvx2Oca2k2QfHLaPNfGIjbVG7Rz\/99FOaN28ede7cmf74xz9SpUqVbL63NVtpVphJ765Smw+85+Gry+jqkxpYe5dSNIQGUzZq4C\/LHyNt4C9PQLYEafbBsm+627uYaIvaPVqlShW6\/PLLacCAAVSzZs0ssAotQ1oVBuvYwqsARIPspwH+svwh2sBfnoBsCdLqg2Xf8nvvYqItKwCSliOtCnPJw\/PLj\/eo6PeJmsQEosGElv204G+fqalFxMCUmN304G+Xp6m1tPpg03K5Sp850ca7Rjdv3kw1atTI7JSoPxhpVJjgKNtDHY6kjicf4qpOlJRdNJiy4QJ\/Wf4YaQN\/eQKyJUijD5Z9wz29i4o23mzw5z\/\/mXizwYgRI6h69erqiI8uXbrQypUr6fe\/\/z1dcsklmRZvritM8NaDPB\/vEfbhQDTINifgL8sfog385QnIlsB1Hyz7dnt7FxVtjzzyCA0bNkytXRs0aJASbdu2baOZM2fS2LFjacmSJeqMtgsvvDBr3MrL47rCvLFkA106Yn65Pz7e44xDa2eWR9oFg2hIm\/ie\/sBflj9EG\/jLE5Atges+WPbtMiTavBG1ww47jO688869Ds\/lUbjevXur2xFYwLGgy+LjssIEp0XzeutBsbhDNMh+FeAvyx+iDfzlCciWwGUfLPtm4d7FRtq83aO9evWiq666KrR0Tz\/9tLrGaurUqdSgQTaPtnBZYYJHfGCUbe9qAtEg26yAvyx\/iDbwlycgWwKXfbDsm2VMtPEI2jXXXENnn3029e\/fP7R0Q4YModdeey2Xh+tilE3vc4Fo0OPkKhX4uyKrbxcx0GflIiX4u6CqbxOiTZ9VopS8S\/SOO+6gv\/71r3TPPfco8eYdoMs\/Y7HWr18\/uvjii3N5uC5G2fSqFxpMPU6uUoG\/K7L6dhEDfVYuUoK\/C6r6NiHa9FklTrlixQq64YYbaPHixeoAXT7mg5\/t27ertWxHHnkk8WaFxo0bJ\/blyoCLChPcMYq1bIWjhwbTVc3Wswv+epxcpkIMXNKNtg3+0YxcpnDRB7ssb1LbYmvavILzblFes8Z\/Nm3apP6ZBVz79u3Vn2rVqiV9R6f5XVQY7BjVDxkaTH1WLlKCvwuqZjYRAzNetlODv22iZvZc9MFmJUg3tZho4ynQZ599lo4++mjiHaSl+rioMMHbD\/hSeDzhBNBgytYM8Jflz94RA9kYgL8sfxd9sOwbFfcuJtrWrFmjdo1eccUV1LNnzywzKlo22xUmuAEBl8IXrxpoMGU\/HfCX5Q\/RBv7yBGRLYLsPln2baO9ios3bPco3HkC0fR8ojLJFV1p\/CogGM162U4O\/baLm9hADc2Y2c4C\/TZrmtiDazJnFzvHyyy\/TbbfdpkbcLrroIjrggAP2srXvvvtS3bp1if\/O4mOzwgRH2a4+qQHxSBuewgTQYMrWDvCX5Y+RNvCXJyBbApt9sOyb6HkXG2nzDtflHaTFHt45mpfDdXHMh16lxUibOSdXOSDaXJHVt4sY6LNykRL8XVDVtwnRps8qUUreNTpnzhx1vEexp2rVqnT66adndhepzQrjnxrFMR961QsNph4nV6nA3xVZfbuIgT4rFynB3wVVfZs2+2B9r3IpxUba5F7ZrmebFaZu71fLC8c7Rn9Yd3+7ha2A1tBgygYV\/GX5s3fEQDYG4C\/L32YfLPsmet4h2vQ4FUxlq8IMmbmMhsxcrvywWMMxH3qBQYOpx8lVKvB3RVbfLmKgz8pFSvB3QVXfpq0+WN+jbMpURZu3jo1f+cEHH6SbbrqJsKZtdwXwT41iA4L+R4EGU5+Vi5Tg74KqmU3EwIyX7dTgb5uomT2INjNeRqk3btxII0eOVHl+8Ytf0BNPPEH8b8WeWrVqUY8ePYj\/zuJjo8IEd40+17MVnXFo7Sy+bubKhAZTNiTgL8ufvSMGsjEAf1n+Nvpg2Tcw857qSJtZ0UojtY0Kg6nR+LFGgxmfnY2c4G+DYjIbiEEyfklzg39Sgsny2+iDk5Ug3dyioo2vsnrxxRdp9uzZ9Ic\/\/IEOPPBAdf8oH7a733770YABA+iII45Il4ihNxsVBlOjhtB9ydFgxmdnIyf426CYzAZikIxf0tzgn5Rgsvw2+uBkJUg3t6homzFjBvXu3ZtatmxJo0aNojp16qjp0nvuuYeef\/554uM+HnnkEWrVqlW6VAy8Ja0wmBo1gB2SFA1mMn5Jc4N\/UoLJ8yMGyRkmsQD+Seglz5u0D05egnQtiIm2LVu2UNeuXdX5a7wpoXr16nu8+fr166l79+7qlgQWdCzgsvgkrTCYGk0WVTSYyfglzQ3+SQkmz48YJGeYxAL4J6GXPG\/SPjh5CdK1ICbavJ2kN9xwA3Xq1Cn0rSdOnKhG2iryjQg4UDdZhUeDmYxf0tzgn5Rg8vyIQXKGSSyAfxJ6yfNCtCVnqGVhzZo16s7Ryy67TB39EfaMGDGCpk2bRlOmTKH69etr2U07UdIK4z9QF7tGzaOHBtOcmc0c4G+TZjxbiEE8brZygb8tkvHsJO2D43mVyyU20rZz5061nm3RokU0ZswYat68+R4Uli1bRtdffz2VlZXR8OHD1caELD5JKoz\/rlEcqBsvumgw43GzlQv8bZGMbwcxiM\/ORk7wt0Exvo0kfXB8r3I5xUQbv\/IHH3xAXbp0oc2bN9MJJ5xATZs2VSTWrl1Lr7\/+OtWoUYPGjRtHxx57rByhCM9JKgymRpOHFQ1mcoZJLIB\/Enp28iIGdjjGtQL+ccnZyZekD7ZTgnStiIo2ftXPP\/+c7r77bnr55Zdpx44d6u2rVKlC5513Ht1yyy3UqFGjdIkYeotbYYK7Rh++uoz4JgQ8ZgTQYJrxsp0a\/G0TNbeHGJgzs5kD\/G3SNLcVtw8295SNHOKiLRsY4pciboV5Y8kGunTE\/HLHWM8WLwZoMONxs5UL\/G2RjG8HMYjPzkZO8LdBMb6NuH1wfI+yOSHaEvKPW2Fw1EdC8P+XHQ2mHY5xrYB\/XHL28iEG9ljGsQT+cajZyxO3D7ZXgnQtQbQl5B23wvjXs51fdhBNuf64hCXJZ3Y0mLJxB39Z\/uwdMZCNAfjL8o\/bB8uWOr53iLb47FTOuBUGR30kBI+RNjsAE1pBh5UQoIXsiIEFiAlMgH8CeBayxu2DLbgWMQHRlhB7nAqD9WwJofuyo8G0xzKOJfCPQ81uHsTALk9Ta+BvSsxu+jh9sN0SpGsNoi0h7zgVZtycz6nPUx8rzzifLVkA0GAm45c0N\/gnJZg8P2KQnGESC+CfhF7yvHH64ORe5SxAtCVkH6fC+Nez\/fqnP6TbLm6RsLiUCO4AACAASURBVBT5zY4GUzb24C\/Ln70jBrIxAH9Z\/nH6YNkSJ\/MuLtr45oO\/\/e1v9Nlnn4W+Sa1atahHjx7Ef2fxiVNhsJ7NXiTRYNpjGccS+MehZjcPYmCXp6k18DclZjd9nD7YbgnStSYq2mbMmKGusvIO1Q179caNG1eoC+OD69kWDDpNTZHiiUcADWY8brZygb8tkvHtIAbx2dnICf42KMa3AdEWn51Rzi1btlDXrl3pyy+\/pAceeICOPvpoqlSpkpGNLCQ2rTBPzv2Cbpy8WBUd69mSRxANZnKGSSyAfxJ6dvIiBnY4xrUC\/nHJ2cln2gfb8SpnRWykbdWqVdS+fXu65ppr1MXwpfqYVhjcN2o30mgw7fI0tQb+psTsp0cM7DM1sQj+JrTspzXtg+2XIF2LYqJt\/fr16rL4Cy+8MFeizb+erX\/bZtS\/bfN0I17BvKHBlA0o+MvyZ++IgWwMwF+WP0RbivwfeeQRevHFF4n\/rl+\/foqe7bkyqTA4n80ed88SGkz7TE0sgr8JLTdpEQM3XHWtgr8uKTfpTPpgNyVI16rYSNu2bdvotddeo0cffZQWL15MrVu3pho1auz19hVp9yjuG7VfudFg2mdqYhH8TWi5SYsYuOGqaxX8dUm5SQfR5obrXla9NW0rVqwo6rEi7R69YvRC+u+P1qn3Pb1FbXq+V6uUaFdcN2gwZWML\/rL82TtiIBsD8JflD9Emy7\/kvJtUGP8mBKxnsxNqNJh2OMa1Av5xydnLhxjYYxnHEvjHoWYvj0kfbM+rnCWx6VG5V7brWbfCfLbuazp+8Fvlzp\/r2YrOOLS23cLk0BoaTNmgg78sf4y0gb88AdkS6PbBsqW0511ctC1dupQGDx5Mb731Fh188MHqIN0DDjiABgwYQBdccAFdcsklqZ3fxgf9Tp8+vZzuqFGjqE2bNkVp61YYHKprr9L6LUE0uOGqaxX8dUm5S4cYuGOrYxn8dSi5S6PbB7srQbqWRUXbBx98oI792GeffahZs2b0xRdfKNFWtWpV9e+LFi1SB+9GCScbyFiwzZs3r\/z2hVmzZlH37t2pT58+1LNnz4IudCvMpHdXUa9Ji5QdHKprI2K7baDBtMcyjiXwj0PNbh7EwC5PU2vgb0rMbnrdPtiuVzlrYqJt586d6gor3ojAR37Mnz9fjbixaGvQoAFt2rRJnd\/Go2484sVCztWzcOFC6ty5Mw0dOnQPgcjlW7lyJY0dO5aqV68e6l63wuBQXTfRQ4PphquuVfDXJeUuHWLgjq2OZfDXoeQujW4f7K4E6VoWE21r1qyhq666iq644go1ksUjW37RxhjGjBlDjz\/+uNjdozZFG69n43Vt\/GATgr1KjgbTHss4lsA\/DjW7eRADuzxNrYG\/KTG76SHa7PIsaM078uOGG26gTp06hYq2iRMnqlE4b\/QtpaIpNyNGjKBhw4apUb5i07M6FQabENxFDg2mO7Y6lsFfh5LbNIiBW75R1sE\/ipDbn+v0wW5LkK51sZE278L4evXq0fDhw9VBu2HTo\/vttx+NHj2aDjzwwFTIeGvZ2Fm7du1U2Yo9XoV58sknic+U44end\/3P28u30KUj5pf\/07wBJ6l1bXiSE0CDmZxhEgvgn4SenbyIgR2Oca2Af1xy8fPxoI\/38BKrjh070uzZs8v74PiWs59TTLQxmhkzZqiF\/jzS1rRpUxo5cqQa2eIgPPTQQ2ojwpAhQ9QUatqPJyp5TVuxkT5PtPnLx+vjrr322vJ\/en7RFrrt5bXq\/xvWrEzPX7tb3OFJTmD79u3EU+0slCtXrpzcICwYEQB\/I1xOEiMGTrBqGwV\/bVTWEj722GM0YcKEPexBtFnDW9jQrl27FHjeALB169Y9Eu67775qowJPn\/J\/SzzeBoVu3boV3EHqiTZ+h0aNGqlisoDwj7ZdNvoDeuefW9TP+CaEv\/yqTOJ1KqRPvg5t9erV6jcsiLb0Qwz+6TMPekQMZGMA\/unz55E2b7TtnXfeofvvvx8jbWmGgXeKzp07V\/3ZsWMH\/fjHP6YzzjiD6tSpk2Yx9vJlItqKqXzchOAujJiacMdWxzL461BymwYxcMs3yjr4RxFy+3OsaXPLN5PWeR1b37591ahfy5Yty8vorW8rthkhqsJgE4LbkKPBdMs3yjr4RxFy\/3PEwD3jYh7AX5Z\/VB8sWzr73kXXtPHr8BTpxx9\/TE899RRt3rxZvSFvTrj88supefPm9t84xKK3fo1\/5J3J5o2ylZWVJTqnLXgTAq6vshtSNJh2eZpaA39TYvbTIwb2mZpYBH8TWvbTQrTZZ1rQIk+L\/v73v1cbEli8+Z9KlSpRhw4d6NZbb6UqVaqkUqrgNVZRtyFwoaIqjF+04SYE+2FEg2mfqYlF8Deh5SYtYuCGq65V8Ncl5SZdVB\/sxqucVbGRNhZpvHh\/3Lhx1KtXL7XbsmbNmooEizk+WJfPaLv55pvVZoSsPlEVBtdXuY0cGky3fKOsg38UIfc\/RwzcMy7mAfxl+Uf1wbKls+9dTLStXbuWrrnmGjrppJPo9ttv3+tSeBZ1PMrG95OysJPelFAIfVSFwfVV9iut3yIaTLd8o6yDfxQh9z9HDNwzhmiTZVzMe1QfnN2SxyuZmGjzbkTgUTa+zirsef7559VonMSNCLo4oyoMrq\/SJRkvHTqseNxs5QJ\/WyTj20EM4rOzkRP8bVCMbyOqD45vOZs5xUQbH0h40003qfVqfOtAcN0aH\/0xcOBANVX64IMPOr0wPkloilWY4M7RBYNOw00ISWCH5EWDaRmooTnwNwTmIDli4ACqgUnwN4DlIClEmwOohUzyzQe8Xq1JkybUr18\/atasGe2zzz7q0Dy+EYGP3Lj77rvpmGOOKTfBB+3Wr18\/xVIWd1WswmDnqPswocF0z7iYB\/CX5c\/eEQPZGIC\/LH+ItpT4e9OjLNxMHj75ng+yzcpTrMJgE4L7KKHBdM8Yok2WcZR3fANRhNz+HPzd8o2yDtEWRcjSz\/nqjzlz5hBPk5o8VatWpfPOO88ki9O0xSrMkJnLaMjM5co\/jvtwEwY0mG646loFf11S7tIhBu7Y6lgGfx1K7tJAtLljWyEtF6sw2DnqPuRoMN0zxkibLOMo7\/gGogi5\/Tn4u+UbZR2iLYqQg5+vX7+e3njjDXrvvffUhoRWrVrRqaeemtljPvwIilUY7Bx1UFkCJtFgumcM0SbLOMo7voEoQm5\/Dv5u+UZZh2iLImTx599++606QJd3j\/J\/+x\/ecMDHgfTs2TO1GxHivFqhCoOdo3FomudBg2nOzGYO8LdJM54txCAeN1u5wN8WyXh2INricYuVa9q0adS\/f39q3bq1uvmAd4\/ys3z5crr33nvpzTffVOe0XXrppbHsp5GpUIXBztE06GPnXDqUC3tBhyUdAXwD0hHANyAbAYi2lPh7l7RXr15dncNWrVq1PTzzRgU+x41H4EaNGlVy57QFR9rWDT8nJbL5coMGUzbe4C\/Ln70jBrIxAH9Z\/hBtKfH3jvzgc9o6deoU6nXixIlq+rQUb0TAztF0KhIazHQ4F\/IC\/rL8IdrAX56AbAkg2lLiv2bNGnV91WWXXaZG1MIeHoF75plnaMqUKZk6UNdf1kIVBjtH06lIEA3pcIZok+VczDu+AdnYgL8sf4i2lPjv3LmTevfuTYsWLaIxY8ZQ8+bN9\/C8bNkyuv7666msrExtVNhvv\/1SKpmZGx3R1r9tM+rfds\/3M\/OC1BAN2awD6LDk44IYyMYA\/GX5Q7SlyP+DDz6gLl26qAN2zz\/\/fDr99NOVdz5096WXXlLr2Hh6lI8AyepTqMLU7f1qeZFx56i76KHBdMdWxzL461BymwYxcMs3yjr4RxFy+3OINrd897L+0UcfqYvhFyxYQLt27VI\/r1SpEh1\/\/PF011130RFHHJFyiczchVWY4CaE53q2ojMOrW1mGKm1CKDB1MLkLBH4O0OrbRgx0EblJCH4O8GqbRSiTRuV3YQ82saH7PJTp06dzO4WDb51WIUJHveBkTa7dcVvDQ2mO7Y6lsFfh5LbNIiBW75R1sE\/ipDbn0O0ueVb4axHiTbcOeo25Ggw3fKNsg7+UYTc\/xwxcM+4mAfwl+UP0SbLv+S8h1UYHPeRXhjRYKbHOswT+MvyZ++IgWwMwF+WP0SbLP+S8x5WYXDcR3phRIOZHmuINlnWhbzjG5CNC\/jL8odok+Vfct6jRBuO+3AbUjSYbvlGWQf\/KELuf44YuGeM6VFZxsW8Q7RlNzaZLFlYhfEf94Gdo27Dhg7LLd8o6+AfRcj9zxED94wh2mQZQ7R9T6DSLu+cjezGJNMlC4o2HPeRbrjQYaXLO+gN\/GX5s3fEQDYG4C\/LHyNtsvxLznuwwuC4j3RDiAYzXd4QbbK8w7zjG5CNCfjL8odok+Vfct6LiTYc9+E+nGgw3TPG1JAs4yjv+AaiCLn9Ofi75RtlHaItihB+vgeBYIXBcR\/pVhA0mOnyxkibLG+MtIF\/9gjIlgiiTZZ\/yXkPVphekxbRpHdXqfc4vUVter5Xdu9NLTnYIQWGaJONIvjL8mfviIFsDMBflj9Emyz\/kvMerDD+M9quPqkBPXx1Wcm9UykVGA2mbLTAX5Y\/RBv4yxOQLQFEmyz\/kvMerDDHD36LeAcpPzjuw304IRrcMy7mAfxl+UO0gb88AdkSQLTJ8i857\/4K890B9YhFm\/dAtLkPJ0SDe8YQbbKMo7zjG4gi5Pbn4O+Wb5R1iLYoQvj5HgQg2mQrBBpM8JclIO8d34BsDMBflj9Emyz\/kvNeTLQtGHQa8bEfeNwRQIPpjq2OZfDXoeQ2DWLglm+UdfCPIuT25xBtbvlWOOv+CvP6F5WJd4\/ygzPa0gk1Gsx0OBfyAv6y\/Nk7YiAbA\/CX5Q\/RJsu\/5Lz7K8wTH+6kITOXQ7SlGEU0mCnCDnEF\/rL8IdrAX56AbAkg2mT5l5x3f4W5+\/XNOKMt5QhCNKQMPOAO\/GX5Q7SBvzwB2RJAtMnyLznv\/grT49k1NGfpBvUOOKMtnVBCNKTDGdOjspyLecc3IBsb8JflD9Emy7\/kvPsrzMXj\/1V+Rlv\/ts2of9vmJfc+pVZgNJiyEQN\/Wf4YaQN\/eQKyJYBok+Vfct79Fea44Z+Ul59vQuDRNjxuCUA0uOUbZR38owi5\/zli4J4xRjplGRfzDtGW3dhksmRehZk4fRbxSJv34GDddMKFDisdzoW8gL8sf4y0gb88AdkSQLTJ8i8574VEG85oSyeUEA3pcIZok+WMkR7wzy4B2ZJBtMnyLznvXoW5+9HniDcieM+64eeU3LuUYoEh2mSjBv6y\/DHSBv7yBGRLANEmy7\/kvHsV5qLfTyA+p40fHKybXhghGtJjHeYJ\/GX5Q7SBvzwB2RJAtMnyLznvYaLt9Ba16flerUruXUqxwBANslEDf1n+EG3gL09AtgQQbbL8S867V2Fqd7ifln9dXZUfoi29MEI0pMcaI22yrAt5xzcgGxfwl+UP0SbLv+S8h4m2J391LF1wdL2Se5dSLDAaTNmogb8sf4y0gb88AdkSQLTJ8i85716F2dRmCH13wG6hhoN10wsjREN6rDHSJssaI23gn00CsqWCaJPlX3LeucJ06PprYtHmPTijLb0wQrSlxxqiTZY1RBv4Z5OAbKkg2mT5l5x3iDbZkEG0gb8sAXnv+AZkYwD+svwh2mT5l5z3MNGGg3XTCyMazPRYY6RNljVG2sA\/mwRkSwXRJsu\/5LxzhWn\/69tpyxn9ysuOg3XTCyNEW3qsIdpkWUO0gX82CciWCqJNln\/JeecKc\/kdU+jrIy9VZcfBuumGEKItXd5Bb+Avy5+9IwayMQB\/Wf4QbbL8S857ULThjLZ0Q4gGM13eEG2yvDHaCf7ZIyBbIog2Wf4l550rzKUjFtA39Y5QZYdoSzeEEG3p8oZok+UN0Qb+2SMgWyKINln+Jec9KNrGdDqK\/uOEH5Tce5RqgSHaZCMH\/rL8MT0K\/vIEZEsA0SbLv+S8c4W5ePy\/cLCuUOQgGoTA\/59b8JflD9EG\/vIEZEsA0SbLv+S8c4W5cOq28nI\/fHUZXX1Sg5J7j1ItMESDbOTAX5Y\/RBv4yxOQLQFEmyz\/kvMeFG24DSHdEEI0pMs76A38ZflDtIG\/PAHZEkC0yfIvOe8QbbIhg2gAf1kC8t7xDcjGAPxl+UO0yfIvOe9B0YaDddMNIRrMdHljpE2Wd5h3fAOyMQF\/Wf4QbbL8S867X7ThYN30w4cGM33mfo\/gL8sf06PgL09AtgQQbbL8E3vfsmULde3alebOnVtuq0+fPtSzZ8+ithcuXEidO3emzZs375Hu5JNPprFjx1L16tVD80O0JQ5ZIgMQDYnwJc4M\/okRJjaAGCRGmMgA+CfClzgzRFtihHIGVq1aRe3bt6eGDRuWCy1PjJ177rk0fPjwgoWbNWsW9e3blyZMmEAtW7bUfgm\/aMPButrYrCVEg2kNZSxD4B8Lm9VMiIFVnMbGwN8YmdUMEG1WcaZrrJDwGjFiBE2ePJmmTp1KDRqEH8fBaWbPnl10VC3sbSDa0o1x0BsaTPCXJSDvHd+AbAzAX5Y\/RJssfyfeWZCNHj266Cha7969le9io3FRoq1\/22bUv21zJ+8Ao+EE0GDK1gzwl+XP3hED2RiAvyx\/iDZZ\/k68syCbN29ewZE2b1qV160tXry4vAzt2rWLFHH+kTaINifhK2oUDWb6zP0ewV+WP0Qb+MsTkC0BRJssf+veecq0e\/fuVGwzQti6t7D1cVEjbbecWUPdhlBoCtb6y8EgRhmE6wBEm3AAMNImHgB8A+mHgPtn71mxYgV17NhRLW9q3Lhx+oVJ2WOlXbt27UrZZ2ruPDFWVlZmvFaNC+kJvlGjRlGbNm1Cy+0faav+xj1Uee1Hahfqtddem9p75tnR9u3bac2aNUooV65cOc8oRN4d\/EWw7+EUMZCNAfinz\/+xxx5Ty538D0Rb+nGw6jGpYOPCeDa6detW8MgQv2gb+fP61LTqFiUgMNpmNZwFjW3bto1Wr16tfsOCaEuHud8L+KfPPOgRMZCNAfinz59H2rzRtnfeeYfuv\/9+jLSlHwZ7Hr0Rsqgz1qI8moq2BYNOIz5gF096BJYvX07jx49XZ\/PlYWg8PbJ6nsBfj5PLVIiBS7rRtsE\/mpHLFFjT5pJuCrY9waazicArTqFpUJ2z2\/wjbbjCKoUAB1zk7YNNn3Bxj+AvHxHEQDYG4A\/+aRKoUGvadA\/SDQL2blFYuXJl+Q5TXVueaMMVVmlW2+99ocGU4e55BX9Z\/uwdMZCNAfiDf5oEKpRo46M9pk+fXpCft6Gg0Lltwfw61195om2frWuJp0fxpEvA2zn05JNPYno0XfTKG\/gLQA+4RAxkYwD+2eCPjQiycSgZ7\/zBHjf8E7VrlHeP4gEBEAABEAABEEiPwCmnnEKTJk1Kz6Ggpwo10ibFkYUb\/8EDAiAAAiAAAiCQLgHehJaXjWgQbenWLXgDARAAARAAARAAgVgEINpiYUMmEAABEAABEAABEEiXAERburzhDQRAAARAAARAAARiEYBoi4UNmUAABEAABEAABEAgXQIQbenyhjcQAAEQAAEQAAEQiEUAoi0WNmQCARAAARAAARAAgXQJQLSlyxveQAAEQAAEQAAEQCAWAYi2WNiQCQRAAARAAARAAATSJQDRli5veAMBEAABEAABEACBWAQg2mJhI\/IumZ87d66ywKcxT506lRo0aBDTIrKFEfDfB1ujRg2aMGECtWzZsiisWbNmUffu3cvTIDbJ6lacGPg9rlq1itq3b08dOnSgnj17JitMDnPH4R9snxibd\/dyDhEmfuU4MfDqvXdbDtqhxGEoaoBjxM\/w4cPdOhK2DtEWIwBeg9iwYcPyCsIVZt68eRBuMXgWysJMV65cSWPHjqXq1avTiBEjaPTo0UWFG6cZNmzYHh0U23nllVe0BJ\/F4lcIU3FiEHxxr8Pr06cPRJthrYjD3xML3D75v53gd2FYlNwmTxKDE088EX1ECjXHa\/fbtWsH0ZYC75JzESYeMJpgN4zeaJl\/dCBMLPu9Fvo5YhMvNnFiEPTkH\/WEaDOLQ1z+3D5Nnjx5j18go74ds5LlJ3WSGAR\/wVy4cCF17tyZunXrhl9eLFWh4IgyRJslsBXNTPA3L+\/9Cv17RXv\/NN4nrONhv4X+vViZPNHm\/603jXcodR9JY+Bx79q1qxrxwfSoWY2Iw9\/rxFq3bg1hYIY7NHWcGHjtFESbhQAUMeHVdZ6NGT9+PA0cOJD8s19uvctZx\/SoIftiv7FiitQQZpHkhQSwzhRp0Cx+w40XlyQx8H8n\/fr1w5q2GCGIw98TyoMGDaIlS5aopQL86K4HjVHMCp0lTgwYSNgvilim4a6q5GkkGaLNsB4VqxxxRoEM3ecmeaHGkqcr+vbtq70+zf\/bGDaKmFWfJDHwfwvsFRsRzNhz6jj8vV9QNm\/eTP6porC1nuYlyl+OODHwU\/JvYDj55JPL1xjmj6TbN4Zoc8u3pK1DtKUTvqSNpVdKr9HEzjnzuMWNgScchg4dSm3atCkfdcD0qFkM4vD32JeVle0hELx2i0vgbU4wK00+U8eJAZPy1sL513FCOLurQxBt7tiWvGVMj6YTwrjTEmG\/5UKwxYtZnBiEfR\/YCJIef0+0nXvuuXvtosNMgHkcknwDYQIZ657NY6CTA6JNh1KO02Ajgvvgx10AzCXz7yiCYIsfqzgx8E\/PhXnGWVX68YjDv9imG4g2ffZeyjgxwGyMOeekOSDakhKs4PnDPmSMJtgNetjaNZ0P00uzaNEi7XVvdktecazFjUGQAL6NeHUiLv+wDVE63068UlbsXHFiUGwqGiNtbupLnuo3NiLEqENhh1di52gMkEWyhG0g0Nk5ih1a9uIQNwYQbXZiEJd\/2BSpzrdjp9QVy0rcGGBNW7r1AKItXd4l6S14qB+mfdyE0b\/7KuzYAr9YXr16tTq8knfOhT3YvRUvRiYxCLvGDSNt8bh7ueLwD16hhCM\/0o9BcKkAYpAsBsVyQ7S5YwvLIAACIAACIAACIAACMQhgejQGNGQBARAAARAAARAAgbQJQLSlTRz+QAAEQAAEQAAEQCAGAYi2GNCQBQRAAARAAARAAATSJgDRljZx+AMBEAABEAABEACBGAQg2mJAQxYQAAEQAAEQAAEQSJsARFvaxOEPBEAABEAABEAABGIQgGiLAQ1ZQAAEQAAEQAAEQCBtAhBtaROHPxAAARAAARAAARCIQQCiLQY0ZAGBUiHw7bff0rPPPkvbtm2jX\/ziF8bF\/vzzz2ns2LHUs2dPql+\/vnH+JBlefvlluu2222jlypXUqFEjmjx5svrb9Cl2ibqprSykD9520KdPHxUfiSd46v+oUaOoTZs2EkWBTxDIBQGItlyEGS+ZVwJJr5DiOytZLE2dOpXCrqhyxXXTpk10\/fXXK8HWvXt3+sEPfkCnn346VatWzdhlRRVthxxyiLq27bDDDqPDDz\/cmIuNDOvXr6d33nmH\/v73vytxD9FmgypsgEBhAhBtqB0gUIEJlKposym0bNrKQlXJ4vt4F6RDtGWhhqAMFZkARFtFji7erUIT2LVrFz3\/\/PN077330ooVK2i\/\/faj448\/nm6\/\/XY64ogjqNiF1XzB8kMPPUTTp0+nL7\/8kipVqkSNGzemm2++mS655BL1\/\/6Lyhlku3btaPjw4YrpRx99RH\/84x\/VCAs\/xx13HPXr149OOeWUSOY85frggw\/SX\/\/6V9q6dSs1bNiQfvWrX6np2ypVqpAnAPyGik0BMocXX3yR\/vznP9Onn36qOJx22mk0aNAgatGiBXkip1WrVnTyySfTAw88QGvWrFF+ucze+7K\/HTt20BNPPEHjx49XTNn2wQcfTF26dKHrrrtOlY8fbwTyl7\/8pfL73Xff0R133EGXX345LV26VP33m2++Sfvuuy9dffXVdOyxx6q4TJgwgVq2bKls8JQ1+xk3bhx99dVXdNBBB9F\/\/Md\/0I033kjVq1cvyDFMtPkvzD777LPpzjvvpLVr16p6MHDgQDVKyTEt9BQaUdUdaYVoi6z2SAACVghAtFnBCCMgkD6BF154QYmss846i9q2bUsbNmyg\/\/qv\/1Kd85NPPkk1atRQAmjYsGH0k5\/8hH72s58pUXXggQcqQfbaa6\/RFVdcQSeddBItW7aMJk2apMTD6NGjlc333ntPiQwWH7\/97W\/pyCOPpB\/\/+MfKJudv1qwZderUSb34xIkTacmSJUrUXXjhhQVhsBBi8cOiif9u2rSpEo5z5syh9u3bK2HDYuPVV1+l++67j370ox8VnQJkUcWjO\/yOZWVlqjwsarg8devWpUcffVQJJ7bN\/85Titdeey1VrVpViSUuBzNjBmxr6NChNGbMGCXkWrduTRs3blS2WAzeeuutqiyeaGOxXKdOHbrhhhuUj5\/+9KdKMPJ7cSy4LDVr1lTCjAUaC0JPtLHIYnE2b9688hi8++67NG3aNCW8WSxx3rCnmGhbtGiRKsOVV16p2LI\/LjsL1WJrzSDa0v9+4REE4hCAaItDDXlAIAMEWDjxqBWLDK+Df+ONN5SgGjx4sOqkw6ZHeZSMF67zqI5\/ATuLBh7x4jVk3r8HO3Nea8aihNe3sWjx1pixIGG\/vAaNRUqY4GBRxMKHN0awYGIByA9vlmDRxSNcnoDSnQL87LPPqGPHjkrosGD0RsLefvttuummm+iWW25RgpVFG4+GPf7449S8eXPl13tfXjvHaVksduvWTa0P45EyFj\/8eD54lM4baWQuXOa7776brrrqKpXOez8e9XvkkUeIR\/b48QtVT7SxUOWRSrZzxhlnlNcmLjeLQPbPI5umom3BggV7CGdec8ajhCzkC8XFE6Fhaxcx0paBDx1FAAEfAYg2VAcQKFECLJpYsLHo4BGgMBzxbgAABm5JREFUsN2dJmvavLQXXXQR9e\/fX1EJdtrz589XI1UXX3yxmnLzPzwCx6N3\/ilA\/89ZFF1zzTVKNN1\/\/\/3loojT8AgRvwOP\/LFvXdHGO0xZYPJUb6GRJP\/0KI\/eedOEuj68qUceufPKzVx4RNL\/rsXej6eDeaE+p+epZB655JFMHm3jEVHv4RE6LuOZZ55ZLhCD1bPYSBunZT\/+6VUuK9cTFqzHHHNMaG3HSFuJNgIodu4IQLTlLuR44YpCgEfZvM6f34nXXvEUKIsfbzSpmGjbvn27mjpjwTR37lziUToeKfOvXQt25mHrzfw8WRCxmDnvvPP2whwmCr1EQSGiK6hYjLBgKyQU2X4hW4X+nUcNme0\/\/vEPJaxef\/11xYmnUD1BFCbaOA+PuvEfHrkLCtq+ffuqcvI6u65duyrmhR4eHWSOPJVtItp4nZ43Gujl01lvBtFWUVoFvEdFJwDRVtEjjPer0AR4So4FxZQpU2jmzJnlGxK8tWVhoo1FCU\/t8XovnprkqUw+\/4wXyL\/00kt7jPIUEm1xdgm6EG1h4klH5ISJOWbJoorXtfEGiQMOOECtgTvxxBOVwOKRzGKibfny5WozRYcOHbREW9iomE5ljdqIECbaevToUVBMs0+INh3ySAMC8gQg2uRjgBKAgDUCH3\/8sVqTxovQWVjxmiZez8VCwlun9t\/\/\/d\/EnTivderVq1f5VJonOnizgn\/tln+tE4sXXtPGU6TeFKpu4XWmR3mUkEepdEfaCk2P8gYDnoo9\/\/zzlZBiBiy+\/IIm6MObouX3\/9Of\/qQ2GfDjrQvjzQvFRFux9\/OLy6OOOop+85vfqN29vI6PN3SYPMVEG68x5LhzWb2HfbNAZ1+FznMrlIZ31\/I6u6hz+nRG80zeEWlBAATCCUC0oWaAQAkS2Lx5M\/3ud79TI2Xc4XobAtatW6cEVb169QqKNk7PHftjjz1WvlieR5m4Y+bjIX7+858XFG3eRgRee8X5mzRpoujx6B3n5SNAeDOBNz3rRxu1EYGFBW9Q4KlBXdFWaCPCc889p47z4F2TvIZMR7R5woPXn\/G6Pu\/haWMWvCy2iom2QhsRPNHHI6L+jQi885ent3ldm7fOjqdjeTMEHxPCPw97onaP+jd58HEuXB946pxjXuhwYt4hy0J15MiRahcsP\/\/6179U3m+++QairQTbCBS5YhKAaKuYccVb5YAA71DkaU7ehclCi59nnnlGjeB4Rzx4oz\/8Mx5d4wXu77\/\/vhIGPCXKo23ckfOOTh5F4x2WLFi8ESmvM+fOm48VYV8zZsxQO0V5sTvb4YX0nJ8FBwtJHukrdCaYzpEfvANUV7T5j\/zgETLeyPDJJ5+oRfcnnHCCErS8kUBHtPFIG78nM+D34unQ2bNnqyNOdu7cqQRuMdHGjHm0kkf4+IgPPvKDbbCI4vfef\/\/9y0Ubi18Wgm+99ZYSScyc19A9\/fTTVLt2bbXTk8\/NMxVtHEPeMOH3zb78u1k9ceo\/+857d94xyzuI+WFRzmVh0emNtHlx4bVz\/g0PGGnLQYODV8wEAYi2TIQBhQABcwI8ysZTl9whsyjgJ3iYKosaFjAs7v7973+rdU3nnnuuOpT3nnvuURsPeO0Wn8vGoo53pPLZZN7xELy4ngUanyfGB9Z6i+P56iLOzwKQRQ4LDJ7yY\/HIZ5YVe8IO1+Up0csuu6z8yA5d0cZ+gofrsghlkeYJS5ONCHwmHR+XwseieIcV8wYCHrl75ZVX1Pl3P\/zhD5UYDO4e9d6ZD9f1Rh29w3W5TDw96d8w4R1w\/NRTT6nz8TgO55xzDg0YMKDoHavFRtqYBceSRy15NJSFKx8fwvXCe8JEG\/\/M\/+4s\/HhXMsf1P\/\/zPyHazD9P5AABJwQg2pxghVEQAAEQ+J4AT7nyZhH+E+fSez9L0yM\/0ogDRtrSoAwfIEAE0YZaAAIgAAIWCPARKrw+bZ999lEjlt5ZaTw9yaNWPGUcPEMtjluItjjUkAcEKgYBiLaKEUe8BQiAQAYI8PQnT6+eeuqp6rw73qChe8WXbvE90cbHkfBu28MOO0zdo8pnv\/FjQxjqloXXu\/FUOW9AYb9xjoLR9YV0IAACGGlDHQABEAABawR4nSFvyuCbE3idIY+68e5VXhfHGyWKXdquWwhPtHnrGHlDAYs3CdHGm17YN+9m5geiTTeKSAcC8Qj8P77GZvSHJYtCAAAAAElFTkSuQmCC","height":299,"width":497}}
%---
%[output:2a3c823f]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:31cd044d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:0ead980c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
%[output:09269494]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"inv_t","value":"                   Vth: 5.500000000000000e+00\n               Vce_sat: 2\n                Rce_on: 1.500000000000000e-04\n            Vdon_diode: 1.750000000000000e+00\n            Rdon_diode: 1.500000000000000e-04\n                   Eon: 1.950000000000000e-01\n                  Eoff: 2.600000000000000e-01\n                  Erec: 1.450000000000000e-01\n        Voff_sw_losses: 600\n         Ion_sw_losses: 1800\n    JunctionTermalMass: 2\n                  Rtim: 1.000000000000000e-02\n         Rth_switch_JC: 1.730000000000000e-02\n         Rth_switch_CH: 1.140000000000000e-02\n         Rth_switch_JH: 3.870000000000000e-02\n         Lstray_module: 1.800000000000000e-08\n                   Irr: 1150\n              Csnubber: 5.464876033057851e-08\n              Rsnubber: 2200"}}
%---
%[output:6bec205b]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"inv_t","value":"                   Vth: 5.500000000000000e+00\n               Vce_sat: 2\n                Rce_on: 1.500000000000000e-04\n            Vdon_diode: 1.750000000000000e+00\n            Rdon_diode: 1.500000000000000e-04\n                   Eon: 1.950000000000000e-01\n                  Eoff: 2.600000000000000e-01\n                  Erec: 1.450000000000000e-01\n        Voff_sw_losses: 600\n         Ion_sw_losses: 1800\n    JunctionTermalMass: 2\n                  Rtim: 1.000000000000000e-02\n         Rth_switch_JC: 1.730000000000000e-02\n         Rth_switch_CH: 1.140000000000000e-02\n         Rth_switch_JH: 3.870000000000000e-02\n         Lstray_module: 1.800000000000000e-08\n                   Irr: 1150\n              Csnubber: 5.464876033057851e-08\n              Rsnubber: 1.666666666666667e+06"}}
%---
%[output:845c2e86]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"inv","value":"                   Vth: 5.500000000000000e+00\n               Vce_sat: 2\n                Rce_on: 1.500000000000000e-04\n            Vdon_diode: 1.750000000000000e+00\n            Rdon_diode: 1.500000000000000e-04\n                   Eon: 1.950000000000000e-01\n                  Eoff: 2.600000000000000e-01\n                  Erec: 1.450000000000000e-01\n        Voff_sw_losses: 600\n         Ion_sw_losses: 1800\n    JunctionTermalMass: 2\n                  Rtim: 1.000000000000000e-02\n         Rth_switch_JC: 1.730000000000000e-02\n         Rth_switch_CH: 1.140000000000000e-02\n         Rth_switch_JH: 3.870000000000000e-02\n         Lstray_module: 1.800000000000000e-08\n                   Irr: 1150\n              Csnubber: 5.464876033057851e-08\n              Rsnubber: 2200"}}
%---
%[output:28f81047]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"inv","value":"                   Vth: 5.500000000000000e+00\n               Vce_sat: 2\n                Rce_on: 1.500000000000000e-04\n            Vdon_diode: 1.750000000000000e+00\n            Rdon_diode: 1.500000000000000e-04\n                   Eon: 1.950000000000000e-01\n                  Eoff: 2.600000000000000e-01\n                  Erec: 1.450000000000000e-01\n        Voff_sw_losses: 600\n         Ion_sw_losses: 1800\n    JunctionTermalMass: 2\n                  Rtim: 1.000000000000000e-02\n         Rth_switch_JC: 1.730000000000000e-02\n         Rth_switch_CH: 1.140000000000000e-02\n         Rth_switch_JH: 3.870000000000000e-02\n         Lstray_module: 1.800000000000000e-08\n                   Irr: 1150\n              Csnubber: 5.464876033057851e-08\n              Rsnubber: 1.666666666666667e+06"}}
%---
