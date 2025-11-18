
%% PMSM paramerters
number_poles = 104;
pp = number_poles/2;
rpm_bez = 17.8;
rpm_base = rpm_bez;
omega_m_bez = rpm_bez/60*2*pi;
omega_bez = omega_m_bez*pp;
ibez = 2916/n_sys;
eta_generator = 0.94;
eta_inverter = 0.98;
Pbez = 1500e3/eta_generator/eta_inverter/n_sys;
tau_bez = Pbez/(omega_m_bez)

psi_m = tau_bez/(3/2*pp*ibez);
ubez = psi_m*omega_bez;

h5_percent_psm = 2; 
h7_percent_psm = 0.5; 

Lq = 0.00126*n_sys;
Ld = 0.00104*n_sys;
Ls = Ld;
Lalpha = (Ld+Lq)/2;
Lbeta = Lalpha;
La = 1/3*(Lq+Ld);
Lb = 1/3*(Lq-Ld);

Rs = 0.00525*n_sys;
Lmu = 5e-6;
Jm = 1940; %kgm^2

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


