function Turbulence = CalculateKaimalSpectra(Turbulence)

%% Cacculate turbulence spectra based on the parameters
% based on the IEC standard 61400-1

% Developed within the LIKE (Lidar Knowledge Europe) project, funded by the European Union's Horizon, 2020 research
% innovation programme under the Marie-Curie grant agreement No. 858358

% Authors: 		
% Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI


% deliver parameters
L_1         = Turbulence.L_1;
L_2         = Turbulence.L_2;
L_3         = Turbulence.L_3;
L_c         = Turbulence.L_c;
sigma_1     = Turbulence.sigma_1;
sigma_2     = Turbulence.sigma_2;
sigma_3     = Turbulence.sigma_3;
URef        = Turbulence.URef;
a_yz        = Turbulence.a_yz;
a_x         = Turbulence.a_x;
b_x         = Turbulence.b_x;


%% Configurate Lidar

f                           = Turbulence.Field.f;
Turbulence.Spectra.f        = f;

% now the u v w and uw
Turbulence.Spectra.S_uu     = (4*L_1/URef./((1+6*f*L_1/URef).^(5/3))*sigma_1^2);
Turbulence.Spectra.S_vv     = (4*L_2/URef./((1+6*f*L_2/URef).^(5/3))*sigma_2^2);
Turbulence.Spectra.S_ww     = (4*L_3/URef./((1+6*f*L_3/URef).^(5/3))*sigma_3^2);

Turbulence.Spectra.kappa_yz =  a_yz*((f/URef).^2+(0.12/L_c).^2).^0.5;
Turbulence.Spectra.kappa_x  =  a_x*((f/URef).^2+(b_x).^2).^0.5;

Turbulence.Variance         = trapz(Turbulence.Spectra.f,Turbulence.Spectra.S_uu);
Turbulence.TI               = sqrt(Turbulence.Variance)/Turbulence.URef;

