function Ft = Compute_Thrust(tav,param)
% COMPUTE_THRUST: returns thrust of each rotor when given the force and
% moments generated, when assuming Ft(i) = kF*w_i^2, where w_i is the
% rotational speed of rotor i
%
% tav = [ Fz ; Mx; My; Mz ] in body-fixed frame
% param: struct with
%           kF: thrust coefficient, Ft(i) = kF*wi^2
%           kM: moment coefficient, i.e. Mz = kM*(w1^2-w2^2+w3^2-w4^2);
%           La: length of quadcopter arm
%

a1 = 1/(2*param.La);
a2 = param.kF/(4*param.kM);

Ft = [ 0.25 0 -a1 a2 ; 0.25 a1 0 -a2 ; 0.25 0 a1 a2 ; 0.25 -a1 0 -a2 ]*tav;
end