function [SC, sc_name] = bldc_scenario(SCEN)
%BLDC_SCENARIO  Test scenarios shared by all controllers.
%   [SC, SC_NAME] = BLDC_SCENARIO(SCEN) returns the scenario struct SC with fields
%     T       stop time (s)
%     t_ref   breakpoints of the speed reference (s),  w_ref  values (rad/s, mechanical)
%     t_load  breakpoints of the load torque (s),      TL     values (N.m)
%   Both references are piecewise constant (held between breakpoints).
%
%   SCEN : 'S1' no-load start-up         'S2' load step
%          'S3' speed reversal           'S4' staircase speed reference
%          'CAL' short run used for look-up-table calibration (Compensated FOC)
%
%   See also BLDC_PARAMS, BLDC_RUN_SCENARIO.

RPM = 2*pi/60;
switch SCEN
    case 'S1'
        sc_name = 'No-load start-up';
        SC = struct('T',0.4, 't_ref',[0 0.02], 'w_ref',[0 100]*RPM, 't_load',0, 'TL',0);
    case 'S2'
        sc_name = 'Load step';
        SC = struct('T',0.8, 't_ref',[0 0.02], 'w_ref',[0 100]*RPM, ...
                    't_load',[0 0.3 0.6], 'TL',[0 1 0]);
    case 'S3'
        sc_name = 'Speed reversal';
        SC = struct('T',0.7, 't_ref',[0 0.02 0.3], 'w_ref',[0 100 -100]*RPM, 't_load',0, 'TL',0);
    case 'S4'
        sc_name = 'Staircase speed reference';
        SC = struct('T',1.25, 't_ref',[0 0.25 0.5 0.75 1.0], ...
                    'w_ref',[30 60 100 60 30]*RPM, 't_load',0, 'TL',0.5);
    case 'CAL'
        sc_name = 'Look-up table calibration';
        SC = struct('T',0.26, 't_ref',[0 0.01], 'w_ref',[0 100]*RPM, 't_load',[0 0.1], 'TL',[0 1]);
    otherwise
        error('bldc_scenario: unknown scenario "%s"', SCEN);
end
if numel(SC.t_ref)  < 2, SC.t_ref  = [SC.t_ref 1e3];  SC.w_ref = SC.w_ref([1 1]); end
if numel(SC.t_load) < 2, SC.t_load = [SC.t_load 1e3]; SC.TL    = SC.TL([1 1]);    end
end
