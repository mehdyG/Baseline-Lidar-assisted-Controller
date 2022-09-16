%% Configurate a 4D Kaimal model based turbulence
% 
% Developed within the LIKE (Lidar Knowledge Europe) project, funded by the European Union's Horizon, 2020 research
% innovation programme under the Marie-Curie grant agreement No. 858358

% Authors: 		
% Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

%% Parameter Definition
%  Nx       Number of points in x direction
%           simulation time   T = Nx*delta_x/Uh    [s]
%           minimal frequency f = Uh/(Nx*delta_x)  [-]
%           Time Step dt = T/Nx =  delta_x/Uh;     [s]
%  Ny       Number of points in y direction        [-]
%  Nz       Number of points in z direction        [-]
%  delta_x  spatial step length in x direction  [m]
%  delta_y  spatial step length in y direction  [m]
%  delta_z  spatial step length in z direction  [m]
%  Uh       Hub height mean wind speed in Longitudinal direction 
%           (also mean speed of turbulence box) [m/s]





function Turbulence = KaimalTurbFieldConfig(TurbSizeName,Turbulence,x_planes,seednum)

switch TurbSizeName
    
case '4096x25x25'   
    %% Turbulence Configuration
    
    %% Turbulence Configuration
    Turbulence.Field.NComp       = 3;    % number of velocity component, fixed
    Turbulence.Field.n1          = 4096; % spatial step length in x direction
    Turbulence.Field.n2          = 25;   % spatial step length in y direction
    Turbulence.Field.n3          = 25;   % spatial step length in z direction
    Turbulence.Field.x_planes    = x_planes;  % if evolving turbulence is 
    Turbulence.Field.n_planes    = length(Turbulence.Field.x_planes); % length of time slots, also the number of upstream yz planes
    Turbulence.Field.dx          = 9;    % steps size in x
    Turbulence.Field.dy          = 5.5;    % steps size in y
    Turbulence.Field.dz          = 5.5;    % steps size in z
    Turbulence.Field.seed        = seednum;    % seed number
    Turbulence.Field.HRef        = 90;         % Hub height, also the center height of the wind field

    Turbulence.Field.dt          = Turbulence.Field.dx/Turbulence.URef;   % time step
    Turbulence.Field.T           = (Turbulence.Field.n1-1)*Turbulence.Field.dt; % simulation time length
    Turbulence.Field.LMax        = [Turbulence.Field.n1*Turbulence.Field.dx (Turbulence.Field.n2-1)*Turbulence.Field.dy (Turbulence.Field.n3-1)*Turbulence.Field.dz]; % field length in three directions
    Turbulence.Field.x_planes    = Turbulence.Field.x_planes; % longitudinal position of upstream yz plane (used for lidar measurement simulation)
    Turbulence.Field.y           = linspace(-Turbulence.Field.LMax(2)/2,Turbulence.Field.LMax(2)/2,Turbulence.Field.n2); % position vector in y
    Turbulence.Field.z           = linspace(-Turbulence.Field.LMax(3)/2,Turbulence.Field.LMax(3)/2,Turbulence.Field.n3); % position vector in z
    Turbulence.Field.x           = 0:Turbulence.Field.dx:(Turbulence.Field.n1-1)*Turbulence.Field.dx;                    % position vector in x
    Turbulence.Field.t           = 0:Turbulence.Field.dt:(Turbulence.Field.n1-1)*Turbulence.Field.dt;                    % time vector
    Turbulence.Field.df          = 1/Turbulence.Field.n1/Turbulence.Field.dt;
    Turbulence.Field.f           = Turbulence.Field.df:Turbulence.Field.df:1/Turbulence.Field.dt/2;

    otherwise
    error('The turbublence field case does not exist, check the input arguement!')

end

Turbulence.TurbSizeName = TurbSizeName;
end