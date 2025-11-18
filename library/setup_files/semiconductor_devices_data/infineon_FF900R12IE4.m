%% FF900R12IE4 (IGBT)
Vth = 5.5;                                              % [V]
Rce_on = 0.15e-3;                                       % [Ohm]
Vce_sat = 2.05;                                         % [V]
Vdon_diode = 1.85;                                      % [V]
Rdon_diode = 0.15e-3;                                   % [Ohm]
Eon = 70e-3;                                            % [J] @ Tj = 125°C
Eoff = 120e-3;                                          % [J] @ Tj = 125°C
Erec = 80e-3;                                           % [J] @ Tj = 125°C
Voff_sw_losses = 600;                                   % [V]
Ion_sw_losses = 900;                                    % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_switch_JC = 29.5/1000;                              % [K/W]
Rth_switch_CH = 14/1000;                                % [K/W]
Rth_switch_JH = Rtim + Rth_switch_JC + Rth_switch_CH;   % [K/W]
Rth_diode_JC = 53.5/1000;                               % [K/W]
Rth_diode_CH = 25.5/1000;                               % [K/W]
Rth_diode_JH = Rtim + Rth_diode_JC + Rth_diode_CH;      % [K/W]
Lstray_module = 18e-9;                                  % [H]
Irr = 475;                                              % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
% ------------------------------------------------------------