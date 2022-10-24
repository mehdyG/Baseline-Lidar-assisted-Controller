%% Configurate a 4D Mann model based turbulence
% 
% by Feng Guo on 13 Jan 2022?? @FUAS
% Developed within the LIKE (Lidar Knowledge Europe) project, funded by the European Union??s Horizon, 2020 research
% innovation programme under the Marie-Curie grant agreement No. 858358
% according to the paper?? Space time structure of turbulence

%% Parameter Definition
%  Nx    Number of points in x direction
%        simulation time   T = Nx*delta_x/Uh    [s]
%        minimal frequency f = Uh/(Nx*delta_x)  [-]
%        Time Step dt = T/Nx =  delta_x/Uh;     [s]

%  Ny    Number of points in y direction        [-]
%  Nz    Number of points in z direction        [-]
%  delta_x  spatial step length in x direction  [m]
%  delta_y  spatial step length in y direction  [m]
%  delta_z  spatial step length in z direction  [m]
%  Uh       Hub height mean wind speed in Longitudinal direction 
%           (also mean speed of turbulence box) [m/s]

%  HH       Hub height  [m]
%  TurbClass  Turbulence Class specified in IEC 61400-1:2019, A+ A B C
%  gamma   % Shear parameter to fit other Spectrum
%  alpha   % vertical shear factor



function Turbulence = MannTurbFieldConfig(TurbSizeName,Turbulence,t_upstream,seednum)

switch TurbSizeName
    

case '4096x32x40_8m_H150'   
    %% Turbulence Configuration
    
    %% Turbulence Configuration
Turbulence.Field.NComp       = 3;    % number of velocity component, fixed
Turbulence.Field.n1          = 4096; % steps in x
Turbulence.Field.n2          = 64;   % steps in y
Turbulence.Field.n3          = 128;   % steps in z
Turbulence.Field.t_upstream  = t_upstream; % time slots
Turbulence.Field.n_upstream  = length(Turbulence.Field.t_upstream); % length of time slots, also the number of upstream yz planes
Turbulence.Field.dt          = 0.5;   % time step
Turbulence.Field.dx          = Turbulence.URef*Turbulence.Field.dt;    % steps size in x
Turbulence.Field.dy          = 8;    % steps size in y
Turbulence.Field.dz          = 8;    % steps size in z
Turbulence.Field.seed        = seednum;    % seed number
Turbulence.Field.HRef        = 150;  % height that the wind speed close to URef

Turbulence.Field.n2_use      = 32;   % actual domain size, we only need 32 points for the simulation, 64
Turbulence.Field.n3_use      = 40;   % steps in z



Turbulence.Field.T           = (Turbulence.Field.n1-1)*Turbulence.Field.dt; % length in time
Turbulence.Field.LMax        = [Turbulence.Field.n1*Turbulence.Field.dx Turbulence.Field.n2*Turbulence.Field.dy Turbulence.Field.n3*Turbulence.Field.dz]; % field length in three directions
Turbulence.Field.x_upstream  = Turbulence.Field.t_upstream*Turbulence.URef; % position of upstream yz plane
Turbulence.Field.y           = linspace(-Turbulence.Field.LMax(2)/Turbulence.Field.n2*(Turbulence.Field.n2/2-1),Turbulence.Field.LMax(2)/2,Turbulence.Field.n2); % position vector in y
Turbulence.Field.z           = linspace(-Turbulence.Field.LMax(3)/Turbulence.Field.n3*(Turbulence.Field.n3/2-1),Turbulence.Field.LMax(3)/2,Turbulence.Field.n3); % position vector in z
Turbulence.Field.x           = 0:Turbulence.Field.dx:(Turbulence.Field.n1-1)*Turbulence.Field.dx;
Turbulence.Field.t           = 0:Turbulence.Field.dt:(Turbulence.Field.n1-1)*Turbulence.Field.dt;

Turbulence.Field.y_use       = linspace(-Turbulence.Field.LMax(2)/Turbulence.Field.n2*(Turbulence.Field.n2_use/2-1),Turbulence.Field.LMax(2)/Turbulence.Field.n2*(Turbulence.Field.n2_use/2),Turbulence.Field.n2_use); % position vector in y
Turbulence.Field.z_use       = linspace(-Turbulence.Field.LMax(3)/Turbulence.Field.n3*(Turbulence.Field.n3_use/2-1),Turbulence.Field.LMax(3)/Turbulence.Field.n3*(Turbulence.Field.n3_use/2),Turbulence.Field.n3_use); % position vector in z

    otherwise
        error('Turbulence field configuration is not defined.')
end

Turbulence.TurbSizeName = TurbSizeName;
end