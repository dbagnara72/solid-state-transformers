clc

%% EQ PMSM paramerters
number_poles = 2;
pp = number_poles/2;
rpm_bez = 3000;
rpm_base = rpm_bez;
omega_m_bez = rpm_bez/60*2*pi;
omega_bez = omega_m_bez*pp;
ibez = 370*sqrt(2);
eta_generator = 0.96;
eta_inverter = 0.96;
Pbez = 250e3/eta_generator/eta_inverter;
tau_bez = Pbez/(omega_m_bez)

psi_m = tau_bez/(3/2*pp*ibez);
ubez = psi_m*omega_bez;

Lq = 0.00125;
Ld = 0.001;
Ls = Ld;
Lalpha = (Ld+Lq)/2;
Lbeta = Lalpha;
La = 1/3*(Lq+Ld);
Lb = 1/3*(Lq-Ld);

Rs = 0.00525;
Lmu = 5e-6;
Jm = 1; %kgm^2

%% per-unit model
Xbez=ubez/ibez;
Lbez=Xbez/omega_bez;
Rs_norm = Rs/Xbez;
Ld_norm = Ld/Lbez;
Lq_norm = Lq/Lbez;
Ls_norm = Ls/Lbez;
Lalpha_norm = Lalpha/Lbez;
Lbeta_norm = Lbeta/Lbez;
La_norm = La/Lbez;
Lb_norm = Lb/Lbez;
psi_bez = ubez/omega_bez;
psi_m_norm = psi_m/psi_bez;
Jm_norm = 1/2*Jm*omega_bez/pp/tau_bez;

%% double check parameter motor and dclink voltage
omega_e = omega_bez;
omega_m = omega_e/pp;
psi = psi_m;
id = -ibez*0.35;
iq = ibez;
torque = 3/2*pp*psi*iq;
power_shaft = torque*omega_m;
vg_dclink = sqrt(3)*sqrt((omega_e*psi+omega_e*Ld*id+Rs*iq)^2+(omega_e*Lq*iq)^2)
vg_phph_rms = sqrt(3/2)*sqrt((omega_e*psi+omega_e*Ld*id+Rs*iq)^2+(omega_e*Lq*iq)^2);


