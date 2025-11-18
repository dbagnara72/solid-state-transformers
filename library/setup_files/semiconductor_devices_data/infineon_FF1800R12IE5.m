
%% FF180R12IE5 (IGBT)
Vth = 5.5;                                              % [V]
Rce_on = 0.15e-3;                                       % [Ohm]
Vce_sat = 2.00;                                         % [V]
Vdon_diode = 1.75;                                      % [V]
Rdon_diode = 0.15e-3;                                   % [Ohm]
Eon = 195e-3;                                           % [J] @ Tj = 125°C
Eoff = 260e-3;                                          % [J] @ Tj = 125°C
Erec = 145e-3;                                          % [J] @ Tj = 125°C
Voff_sw_losses = 600;                                   % [V]
Ion_sw_losses = 1800;                                   % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_switch_JC = 17.3/1000;                              % [K/W]
Rth_switch_CH = 11.4/1000;                              % [K/W]
Rth_switch_JH = Rtim + Rth_switch_JC + Rth_switch_CH;   % [K/W]
Rth_diode_JC = 28.3/1000;                               % [K/W]
Rth_diode_CH = 13.2/1000;                               % [K/W]
Rth_diode_JH = Rtim + Rth_diode_JC + Rth_diode_CH;      % [K/W]
Lstray_module = 18e-9;                                  % [H]
Irr = 1150;                                             % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
gate_charge = 8.65e-6;                                  % [C]
% ------------------------------------------------------------