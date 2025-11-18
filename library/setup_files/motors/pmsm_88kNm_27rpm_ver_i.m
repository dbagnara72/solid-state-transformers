clc

%% PMSM paramerters full electrification

%% inverter
Vdc_bez = 660;
Vdc = Vdc_bez;
Vdc_max = Vdc_bez*1.25;
udc_bez = Vdc_bez;

%% PMSM paramerters
number_poles = 80;
pp = number_poles/2;
omega_m_bez = 27; %rpm
omega_bez = omega_m_bez/60*2*pi*number_poles/2;
ibez = 400*sqrt(2); %A
eta_inv = 0.98;
Pbez = 250e3/eta_inv; %W
tau_bez = Pbez*eta_inv/(omega_m_bez/60*2*pi)

psi_m = tau_bez/(3/2*number_poles/2*ibez)
ubez = psi_m*omega_bez
La = 1.25e-3;
Lb = 500e-6;
Ld = 3/2*(La-Lb)
Lq = 3/2*(La+Lb)
Ls = Lq;
Lalpha = (Ld+Lq)/2;
Lbeta = Lalpha;


Lalpha = 3/2*La;
Lbeta = Lalpha;

Rs = 15e-3;
Lmu = 5e-6;
Jm = 1; %kgm^2

%% per-unit model
Xbez=ubez/ibez;
Lbez=Xbez/omega_bez;
Rs_norm = Rs/Xbez;
Ld_norm = Ld/Lbez;
Lq_norm = Lq/Lbez;
Lalpha_norm = Lalpha/Lbez;
Lbeta_norm = Lbeta/Lbez;
La_norm = La/Lbez;
Lb_norm = Lb/Lbez;
Ls_norm = Ls/Lbez;
psi_bez = ubez/omega_bez;
psi_m_norm = psi_m/psi_bez;
Jm_norm = 1/2*Jm*omega_bez/pp/tau_bez;

%% double check parameter motor and dclink voltage
omega_e = omega_bez
omega_m = omega_e/(number_poles/2)
psi = psi_m;
id = 0;
iq = ibez;
torque = 3/2*number_poles/2*psi*iq
power_shaft = torque*omega_m
vg_dclink = sqrt(3)*sqrt((omega_e*psi+omega_e*Ld*id+Rs*iq)^2+(omega_e*Lq*iq)^2)
vg_phph_rms = sqrt(3/2)*sqrt((omega_e*psi+omega_e*Ld*id+Rs*iq)^2+(omega_e*Lq*iq)^2)


