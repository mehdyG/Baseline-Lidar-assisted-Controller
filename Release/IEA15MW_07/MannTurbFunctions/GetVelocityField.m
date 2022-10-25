function Turbulence = GetVelocityField(Turbulence,TargetDir,Addshear)

% This code read in the 4D Mann turb or IEC Kaimal Turb into matlab and add shear (for IEC Kaimal the shear already exist)
% by Feng Guo on 13 Jan 2022?? @FUAS
% Developed within the LIKE (Lidar Knowledge Europe) project, funded by the European Union??s Horizon, 2020 research
% innovation programme under the Marie-Curie grant agreement No. 858358

%TargetDir  contains the mt4d to convert and the resulting binary will be written in this folder
FileName    = [ TargetDir '\' Turbulence.Field.CaseName];

n1          = Turbulence.Field.n1;

if isfield(Turbulence.Field,'n2_use')&&isfield(Turbulence.Field,'n3_use')
       n2          = Turbulence.Field.n2_use;
       n3          = Turbulence.Field.n3_use;
else
       n2          = Turbulence.Field.n2;
       n3          = Turbulence.Field.n3;
end


nUnfrozen   = Turbulence.Field.n_upstream;
shearExp    = Turbulence.alpha_shear;

% write out the bladed style wnd
Turbulence.Field.nn          = [Turbulence.Field.n1 Turbulence.Field.n2 Turbulence.Field.n3];
Turbulence.Field.LMax        = Turbulence.Field.LMax;


if isfield(Turbulence.Field,'z_use')
    z           = Turbulence.Field.z_use;
else
    z           = Turbulence.Field.z;
end


HubHeight   = Turbulence.Field.HRef;
URef        = Turbulence.URef;
%dt          = Turbulence.dt;
Xunfrozen   = Turbulence.Field.x_upstream;

if strcmp(Turbulence.Type,'Mann')
    fileID      = fopen([FileName  '.mt4d']);
    uvw         = fread(fileID,'real*4');
    fclose(fileID);

    uvw         = reshape(uvw, [n3 n2 n1 nUnfrozen 3]);  % the dimention and sequnce should be fixed here

    u           = squeeze(uvw(:,:,:,:,1));
    v           = squeeze(uvw(:,:,:,:,2));
    w           = squeeze(uvw(:,:,:,:,3));
    u           = permute(u ,[4 3 2 1]);  %Nunfrozen Nx Ny Nz   We actually use Nx as time in the simulation
    v	        = permute(v ,[4 3 2 1]);
    w	        = permute(w ,[4 3 2 1]);
    % flip
    u           = flip(flip(u,2),3);
    v           = flip(flip(v,2),3);
    w           = flip(flip(w,2),3);

    if Addshear ==1
        for iz = 1:length(z)
            Hz           = max(z(iz)+HubHeight,0);
            Uz           = URef*(Hz/HubHeight)^shearExp;
            u(:,:,:,iz)  = u(:,:,:,iz)+Uz;
        end
    end
    
    
end


if strcmp(Turbulence.Type,'IEC_Kaimal')
   u         = zeros(nUnfrozen, n1,n2,n3); % Nunfrozen Nx Ny Nz Ncom
   v         = zeros(nUnfrozen, n1,n2,n3); % Nunfrozen Nx Ny Nz Ncom
   w         = zeros(nUnfrozen, n1,n2,n3); % Nunfrozen Nx Ny Nz Ncom
   
   [velocity, y, z, ~, ~, ~, ~, ~, ~, ~, ~,Scale,Offset] = readBLgrid_evo([FileName  '.wnd']);
   % velocity: t com y z
   u(1,:,:,:) = squeeze(velocity(:,1,:,:));
   v(1,:,:,:) = squeeze(velocity(:,2,:,:));
   w(1,:,:,:) = squeeze(velocity(:,3,:,:));
   
   if Turbulence.Field.n_upstream>1
   fileID      = fopen([FileName  '.evo']);
   uvw         = fread(fileID,'int16');
   uvw         = uvw(Turbulence.Field.n_upstream+1:end);  % squence: com y z t Xunfrozen
   uvw         = reshape(uvw,[3 Turbulence.Field.n2 Turbulence.Field.n3 Turbulence.Field.n1 Turbulence.Field.n_upstream-1]);
   %data2export = permute(data2export,[5 1 2 3 4]);    %after permute: com y z t Xunfrozen
   fclose(fileID);
   end
   for i = 1:Turbulence.Field.n_upstream-1
       u(i+1,:,:,:) = permute(squeeze(uvw(1,:,:,:,i)),[3 1 2]).*Scale(1)+Offset(1);
       v(i+1,:,:,:) = permute(squeeze(uvw(2,:,:,:,i)),[3 1 2]).*Scale(2)+Offset(2);
       w(i+1,:,:,:) = permute(squeeze(uvw(3,:,:,:,i)),[3 1 2]).*Scale(3)+Offset(3);
   end
   
   %a=1;
%fileID      = fopen([FileName  '.mt4d']);
% uvw         = fread(fileID,'real*4');
% fclose(fileID);
end


Turbulence.windfield.u = u;
Turbulence.windfield.v = v;
Turbulence.windfield.w = w;
Turbulence.windfield.x = Turbulence.Field.x;

if isfield(Turbulence.Field,'n2_use')&&isfield(Turbulence.Field,'n3_use')
    Turbulence.windfield.y = Turbulence.Field.y_use;
    Turbulence.windfield.z = Turbulence.Field.z_use;
else
    Turbulence.windfield.y = Turbulence.Field.y;
    Turbulence.windfield.z = Turbulence.Field.z;
end
Turbulence.windfield.t = Turbulence.Field.t;
Turbulence.windfield.x_evo = Xunfrozen;