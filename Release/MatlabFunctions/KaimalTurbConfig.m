%% Configurate a Kaimal spectra and exponential cohernece model based turbulence
% based on the IEC standard 61400-1

% Developed within the LIKE (Lidar Knowledge Europe) project, funded by the European Union's Horizon, 2020 research
% innovation programme under the Marie-Curie grant agreement No. 858358

% Authors: 		
% Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI


%% Parameter Definition

function Turbulence = KaimalTurbConfig(TurbName,URef,varargin)


switch TurbName
    
case 'IEC_Class_1A'   
    %% Turbulence Configuration
    
    Turbulence.URef              = URef;   % reference wind speed usually consider the hub height mean wind
    Turbulence.IRef              = 0.16;   % reference TI  for IEC Class 1A
    Turbulence.sigma_1           = Turbulence.IRef*(0.75*Turbulence.URef+5.6); % std. of u 
    Turbulence.sigma_2           = Turbulence.sigma_1*0.8; % std v 
    Turbulence.sigma_3           = Turbulence.sigma_1*0.5; % std w 
    Turbulence.L_1               = 8.1*42;   % integral length scale u
    Turbulence.L_2               = 2.7*42;   % integral length scale v
    Turbulence.L_3               = 0.66*42;  % integral length scale w
    Turbulence.a_yz              = 12;                 % u component coherence decay constant yz plane
    Turbulence.L_c               = Turbulence.L_1;     % u component coherence intercept constant yz plane
    Turbulence.a_x               = 1;                  % u component coherence decay constant in longitudinal direction
    Turbulence.b_x               = 0;                  % u component coherence intercept in longitudinal direction
    Turbulence.a_yz2             = 'default';          % v component coherence decay constant in yz plane, by default, there is no correlation
    Turbulence.L_c2              = 'default';          % v component coherence intercept constant in yz plane, by default, there is no correlation
    Turbulence.a_yz3             = 'default';          % w component coherence decay constant in yz plane, by default, there is no correlation
    Turbulence.L_c3              = 'default';          % w component coherence intercept constant in yz plane, by default, there is no correlation
    Turbulence.alpha_shear       = 0.2;                % shear exponent  
    Turbulence.TurbulenceName    = TurbName;
       
end
Turbulence.Type = 'IEC_Kaimal';
end