clear all;
close all;
clc;

%%
FileName = 'UniformConstant_URef_18';

%% Preprocessing
% time
T           = 40;                   % [s]   simulation length
dt          = 1/80;               	% [s]   simulation time step

% wind
V_hub       = 18;                   % [m/s] mean wind speed at hub height 

% calculation of wind signals
t           = 0:dt:T-dt;

u           = ones(1, length(t)).*V_hub;



figure()
plot(t,u)


% Some variables required in the Type 4 wind: Bladed style
HubHeight   = 90;
dy          = 64;
dz          = 64;
Ny          = 3;
Nz          = 3;
URef        = V_hub;
zOffset     = HubHeight;
z0          = 0.1;          % the rougthness length, not really used


SummVars(1) = HubHeight;    % HubHeight
SummVars(3) = URef;

velocity    = zeros(max(size(t)),3,Ny,Nz);
% prepare to write out
for iy = 1:Ny
    for iz = 1:Nz
        velocity(:,1,iy,iz) = u;
    end
end

% here we use std u for v and w components, to avoid dividing by zero
SummVars(4) = 0.1;
SummVars(5) = 0.1;
SummVars(6) = 0.1;

disp('Exporting rotor plane wind field as ".wnd" binary files...')
% write the .wnd
WriteBLgrid([FileName '.wnd'], velocity, dy, dz, dt, zOffset, z0, SummVars)

% Now write a sum file
fileID    = fopen([FileName '.sum'],'w');
fprintf(fileID,'This summary file is not complete it only contains required information for the OpenFAST');
fprintf(fileID,'\n');
fprintf(fileID,'F        Clockwise rotation when looking downwind?\n');
fprintf(fileID,[num2str(SummVars(1)) '  Hub height [m] \n']);
fprintf(fileID,['UBar   =  ' num2str(SummVars(3)) ' m/s \n']);
fprintf(fileID,['TI(u)  =  ' num2str(SummVars(4)) ' %%\n']);
fprintf(fileID,['TI(v)  =  ' num2str(SummVars(5)) ' %%\n']);
fprintf(fileID,['TI(w)  =  ' num2str(SummVars(6)) ' %%\n']);
fprintf(fileID,['Height Offset =  ' num2str(0) ' m\n']);
fprintf(fileID,'Creating a PERIODIC output file.');
fclose(fileID);

