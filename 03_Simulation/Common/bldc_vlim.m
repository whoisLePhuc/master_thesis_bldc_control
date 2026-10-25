function [ud, uq] = bldc_vlim(ud_r, uq_r, th, P)
%BLDC_VLIM  Voltage limiter of the d-q current controllers (P.vlim selects the strategy).
%   [UD, UQ] = BLDC_VLIM(UD_R, UQ_R, TH, P): requested d-q voltages UD_R, UQ_R (V),
%   electrical angle TH (rad, the angle used by SVPWM); called by the VLim block
%   (MATLAB Function) of Compensated_FOC.slx. Code-generation compatible.
%   P.vlim:
%   0  clamp u_d and u_q separately to +/-Vm             (original model)
%   1  limit the vector magnitude |u_dq| <= Vm, keep the angle
%   2  inverse Park, clamp each phase voltage to +/-Vm, back to d-q (as PVC)
%   3  limit only by the inverter hexagon: max(v_abc)-min(v_abc) <= Vdc, keep the angle
%#codegen
Vm = P.Vm;
switch P.vlim
    case 1
        m = sqrt(ud_r^2 + uq_r^2);
        k = 1;
        if m > Vm, k = Vm/m; end
        ud = k*ud_r; uq = k*uq_r;
    case 2
        c = cos(th); s = sin(th);
        al = ud_r*c - uq_r*s; be = ud_r*s + uq_r*c;
        v = [al; -al/2 + sqrt(3)/2*be; -al/2 - sqrt(3)/2*be];
        v = min(max(v, -Vm), Vm);
        al = 2/3*(v(1) - v(2)/2 - v(3)/2); be = (v(2) - v(3))/sqrt(3);
        ud = al*c + be*s; uq = -al*s + be*c;
    case 3
        c = cos(th); s = sin(th);
        al = ud_r*c - uq_r*s; be = ud_r*s + uq_r*c;
        v = [al; -al/2 + sqrt(3)/2*be; -al/2 - sqrt(3)/2*be];
        span = max(v) - min(v);
        k = 1;
        if span > P.Vdc, k = P.Vdc/span; end
        ud = k*ud_r; uq = k*uq_r;
    otherwise
        ud = min(max(ud_r, -Vm), Vm); uq = min(max(uq_r, -Vm), Vm);
end
end
