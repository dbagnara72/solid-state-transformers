
%% FF1200R17IP5 (IGBT)
Vth = 5.5;                                              % [V]
Rce_on = 0.15e-3;                                       % [Ohm]
Vce_sat = 2.15;                                         % [V]
Vdon_diode = 1.75;                                      % [V]
Rdon_diode = 0.15e-3;                                   % [Ohm]
Eon = 400e-3;                                           % [J] @ Tj = 125°C
Eoff = 400e-3;                                          % [J] @ Tj = 125°C
Erec = 245e-3;                                          % [J] @ Tj = 125°C
Voff_sw_losses = 900;                                   % [V]
Ion_sw_losses = 1200;                                   % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_switch_JC = 24.3/1000;                              % [K/W]
Rth_switch_CH = 19.6/1000;                              % [K/W]
Rth_switch_JH = Rtim + Rth_switch_JC + Rth_switch_CH;   % [K/W]
Rth_diode_JC = 43.9/1000;                               % [K/W]
Rth_diode_CH = 22.9/1000;                               % [K/W]
Rth_diode_JH = Rtim + Rth_diode_JC + Rth_diode_CH;      % [K/W]
Lstray_module = 18e-9;                                  % [H]
Irr = 1100;                                             % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
% ------------------------------------------------------------