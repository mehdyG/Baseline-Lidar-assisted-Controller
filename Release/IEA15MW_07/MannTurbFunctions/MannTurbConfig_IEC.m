%% Configurate a 4D Mann model based turbulence, scale to IEC turbulence intensity
% 
% by Feng Guo on 13 Jan 2022
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

% Developed within the LIKE (Lidar Knowledge Europe) project, funded by the European Union's Horizon, 2020 research
% innovation programme under the Marie-Curie grant agreement No. 858358
% according to the paper



function Turbulence = MannTurbConfig_IEC(TurbName,URef,varargin)


switch TurbName
    
    case 'Neutral_IEC1B'   
    %% Turbulence Configuration
    
    Turbulence.URef              = URef;   % reference wind speed usually consider the hub height mean wind
    Turbulence.IRef              = 0.14;   % IEC class 1B reference turbulence intensity
    Turbulence.alphaeps          = 0.2882; % energy level constant
    Turbulence.L                 = 49;     % turbulence length scale 
    Turbulence.Gamma             = 3.1;    % anisotropy
    Turbulence.gamma             = 400;    % time constant for eddy life time
    Turbulence.factor(1)         = 1;      % eddy life time slope
    Turbulence.factor(2)         = 3.5;    % eddy life time slope
    Turbulence.alpha_shear       = 0.14;   % shear exponent  according to IEC 61400 offshore case
    Turbulence.TurbulenceName    = TurbName;       
    
  
    otherwise
        error('the configuration is not defined.')
    
end


if isempty(varargin)
else
   Turbulence.gamma =  varargin{1};
end
Turbulence.Type = 'Mann';
end