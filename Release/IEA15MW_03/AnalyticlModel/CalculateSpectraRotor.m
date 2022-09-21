% -----------------------------
% Function: calculates spectra for NREL 5MW
% ------------
% History:
% v01:	David Schlipf on 20-Aug-2022
% ----------------------------------
function [f,S_RR] = CalculateSpectraRotor(T,dt,URef)

%% Kaimal spectrum
% frequency vector
f_min               = 1/T;
f_max               = 1/dt*1/2;
df                  = f_min;
f                   = [f_min:df:f_max];
% from [IEC 61400-1 third edition 2005-08 Wind turbines - Part 1: Design requirements 2005]
Lambda_1            = 42;
a                   = 12;
L_1                 = 8.1   *Lambda_1;
sigma_1             = 0.16*(0.75*URef+5.6);
L_c                 = L_1;
kappa               = a*((f/URef).^2+(0.12/L_c).^2).^0.5;
S_uu                = 4*L_1/URef./((1+6*f*L_1/URef).^(5/3))*sigma_1^2;

%% Points in rotor disc
% turbine: NREL 5MW
R                   = 63;                   % [m]   rotor radius
[Y,Z]               = meshgrid(-64:4:64);   % [m]   grid to calculate spectra: should be larger than rotor
DistanceFromHub     = (Z(:).^2+Y(:).^2).^0.5;
PointsInRotorDisc   = DistanceFromHub<=R;
nPoints             = length(DistanceFromHub);
nPointsInRotorDisc  = sum(PointsInRotorDisc);

%% Auto-spectrum Rotor
% initialization
SumGamma            = zeros(size(f));

% loop over ...
for iPoint=1:1:nPoints                     % ... all iPoints
    if PointsInRotorDisc(iPoint)
        for jPoint=1:1:nPoints             % ... all jPoints
            if PointsInRotorDisc(jPoint)
                Distance   	= ((Y(jPoint)-Y(iPoint))^2+(Z(jPoint)-Z(iPoint))^2)^0.5;
                SumGamma  	= SumGamma + exp(-kappa.*Distance);
            end
        end
	end
end

% sum over loop
S_RR               	= SumGamma.*S_uu/nPointsInRotorDisc^2;

end
