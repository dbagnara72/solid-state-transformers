
%% CAB006A12GM3 (MOSFET)
Vth = 2.5;                                              % [V]
Rds_on = 8.5e-3;                                        % [Ohm]
Vdon_diode = 4.4;                                       % [V]
Vgamma = Vdon_diode;                                    % [V]
Rdon_diode = 0.85e-3;                                   % [Ohm]
Eon = 5e-3;                                             % [J] @ Tj = 125°C
Eoff = 0.5e-3;                                          % [J] @ Tj = 125°C
Eerr = 0.5e-3;                                          % [J] @ Tj = 125°C
Voff_sw_losses = 600;                                   % [V]
Ion_sw_losses = 200;                                    % [A]
JunctionTermalMass = 2;                                 % [J/K]
Rtim = 0.01;                                            % [K/W]
Rth_mosfet_JC = 100/1000;                               % [K/W]
Rth_mosfet_CH = 32/1000;                                % [K/W]
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH;   % [K/W]
Lstray_module = 7.1e-9;                                 % [H]
Irr = 275;                                              % [A]
Csnubber = 12e-12;                                      % [F]
Rsnubber = 2200;                                        % [Ohm]
% ------------------------------------------------------------