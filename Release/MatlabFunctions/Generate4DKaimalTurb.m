%% This function generates 4D Turbulence based on two steps Cholesky Decomposition using Turbsim
% See https://wes.copernicus.org/preprints/wes-2021-91/ for more detail,
% Modified based on evoTurb evoTurb --- 4D wind field generator DOI: 10.5281/zenodo.5028595

% Developed within the LIKE (Lidar Knowledge Europe) project, funded by the European Union's Horizon, 2020 research
% innovation program under the Marie-Curie grant agreement No. 858358

% Authors: 		
% Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI


function Turbulence = Generate4DKaimalTurb(Turbulence,TargetDir,ExeDir,Delete3D,ParallelCompute)

%TargetDir where to put the results
%ExeDir:   where the Turbsim.exe exist

if ~exist(TargetDir, 'dir')
       mkdir(TargetDir)
end

%% Step 1. Configurate seeds for upstream Turbulence
if Turbulence.Field.n_planes>1
    rng(Turbulence.Field.seed)
    upstream_seed = randi([1 10^5],1,Turbulence.Field.n_planes-1);
    seeds_all     =  [Turbulence.Field.seed upstream_seed];
else
    seeds_all     =  [Turbulence.Field.seed];
end
RunCase           = cell(Turbulence.Field.n_planes,1);  % independent 3D turbulence 


% configure run cases for parallel Turbsim calling
for iCase   = 1:length(seeds_all)

    TextCell{1} = num2str(Turbulence.Field.n1);
    TextCell{2} = num2str(Turbulence.Field.n2);
    TextCell{3} = num2str(Turbulence.Field.n3);
    TextCell{4} = num2str(Turbulence.Field.LMax(1));
    TextCell{5} = num2str(Turbulence.Field.LMax(2));
    TextCell{6} = num2str(Turbulence.Field.LMax(3));
    TextCell{7} = num2str(Turbulence.Field.n_planes);

for it = 1:Turbulence.Field.n_planes  % longitudinal position of 
    TextCell{7+it} = num2str(round(Turbulence.Field.x_planes(it),2));    
end

Text_temp= [TextCell{1} 'x' TextCell{2} 'x' TextCell{3} '_' TextCell{4}...
                'x' TextCell{5} 'x' TextCell{6} 'm_xup'];
for it = 1:Turbulence.Field.n_planes
    Text_temp = [Text_temp  strrep(TextCell{7+it},'.','d') '_'];                
end
Text_temp = ['Uref_' num2str(Turbulence.URef) '_' Text_temp  Turbulence.TurbulenceName '_Seed' num2str(seeds_all(iCase))];

if iCase == 1
    Turbulence.Field.CaseName        =   [Text_temp];          
end

     % if the wnd for the required 3D field does not exist
    if ~isfile([TargetDir '\' Turbulence.Field.CaseName '.wnd']) 
        copyfile([ExeDir '\TurbSim2aInputFileTemplate.inp'],[Text_temp '.inp'])
        copyfile([ExeDir '\UserDefinedPSDTemplate.psd'],[Text_temp '_psd.inp'])

        ManipulateFASTinput([Text_temp '.inp'], 'RandSeed1',num2str(seeds_all(iCase))); 
        ManipulateFASTinput([Text_temp '.inp'], 'URef',num2str(Turbulence.URef)); 
        ManipulateFASTinput([Text_temp '.inp'], 'NumGrid_Z',num2str(Turbulence.Field.n2)); 
        ManipulateFASTinput([Text_temp '.inp'], 'NumGrid_Y',num2str(Turbulence.Field.n3)); 
        ManipulateFASTinput([Text_temp '.inp'], 'HubHt',num2str(Turbulence.Field.HRef)); 
        ManipulateFASTinput([Text_temp '.inp'], 'RefHt',num2str(Turbulence.Field.HRef)); 
        ManipulateFASTinput([Text_temp '.inp'], 'TimeStep',num2str(Turbulence.Field.dt));            
        ManipulateFASTinput([Text_temp '.inp'], 'AnalysisTime',num2str(Turbulence.Field.T));            
        ManipulateFASTinput([Text_temp '.inp'], 'UsableTime','ALL');            
        ManipulateFASTinput([Text_temp '.inp'], 'GridHeight',num2str(Turbulence.Field.LMax(3)));            
        ManipulateFASTinput([Text_temp '.inp'], 'GridWidth',num2str(Turbulence.Field.LMax(2)));       
        ManipulateFASTinput([Text_temp '.inp'], 'InCDec1',['"' num2str(Turbulence.a_yz) ' ' num2str(0.12/Turbulence.L_c) '"']); 
        if strcmp(num2str(Turbulence.a_yz2),'default')&&strcmp(num2str(Turbulence.L_c2),'default')...
                &&strcmp(num2str(Turbulence.a_yz3),'default')&&strcmp(num2str(Turbulence.L_c3),'default')
            ManipulateFASTinput([Text_temp '.inp'], 'InCDec2',['"default"']); 
            ManipulateFASTinput([Text_temp '.inp'], 'InCDec3',['"default"']); 
        else
            ManipulateFASTinput([Text_temp '.inp'], 'InCDec2',['"' num2str(Turbulence.a_yz2) ' ' num2str(0.12/Turbulence.L_c2) '"']); 
            ManipulateFASTinput([Text_temp '.inp'], 'InCDec3',['"' num2str(Turbulence.a_yz3) ' ' num2str(0.12/Turbulence.L_c3) '"']); 
        end
        ManipulateFASTinput([Text_temp '.inp'], 'TurbModel','USRINP');    
        ManipulateFASTinput([Text_temp '.inp'], 'UserFile',[Text_temp '_psd.inp']);    

        spectra_temp = [Turbulence.Spectra.f' Turbulence.Spectra.S_uu' Turbulence.Spectra.S_vv' Turbulence.Spectra.S_ww'];
        PasteUserDefinedSpectrum([Text_temp '_psd.inp'],spectra_temp)    
        ManipulateFASTinput([Text_temp '_psd.inp'], 'NumUSRf',num2str(max(size(Turbulence.Spectra.f)))); 

        RunCase{iCase} = [Text_temp];


    else
        fprintf('The turbulence box with the specified seed already exist! \n')
    end

end

%% Step 2. Batch run turbsim to get the turbulence fields with the required seeds
if ~isempty(RunCase{iCase})
    if ~isfile([RunCase{iCase} '.wnd'])
        if ParallelCompute == 1
            parfor iCase   = 1:length(seeds_all)
            dos([ExeDir '\TurbSim_x64.exe',' ' RunCase{iCase} '.inp']);
            end
            poolobj = gcp('nocreate');
            %Shut down the current pool by using the delete function.
            delete(poolobj);
        else
            for iCase   = 1:length(seeds_all)
            dos([ExeDir '\TurbSim_x64.exe',' ' RunCase{iCase} '.inp']);
            end
        end
    end
end


%% Step 3. Generate 4D turbulence field with longitudinal coherence by independent 3D turbulence fields
if Turbulence.Field.n_planes>1 &&~isfile([TargetDir '\' Turbulence.Field.CaseName '.evo'])
    % Only generate '.evo' file if n_planes>1 and the '.evo' file does not exist

    U = nan(Turbulence.Field.n3,Turbulence.Field.n2,Turbulence.Field.n1,Turbulence.Field.n_planes);   
    V = nan(Turbulence.Field.n3,Turbulence.Field.n2,Turbulence.Field.n1,Turbulence.Field.n_planes);
    W = nan(Turbulence.Field.n3,Turbulence.Field.n2,Turbulence.Field.n1,Turbulence.Field.n_planes);
    for i = 1:Turbulence.Field.n_planes

         % u v w binary file name           
         wndName   = fullfile([RunCase{i} '.wnd']);

         % read .wnd in matlab
         [velocity, ~, ~, ~, ~, ~, ~, ~, ~, ~, ~,Scale,Offset] = readBLgrid_evo(wndName);
         U(:,:,:,i)           = permute(squeeze(velocity(:,1,:,:)),[3 2 1]);    %velocity: t com y z, after permute z,y,t
         V(:,:,:,i)           = permute(squeeze(velocity(:,2,:,:)),[3 2 1]);   
         W(:,:,:,i)           = permute(squeeze(velocity(:,3,:,:)),[3 2 1]);   

         if i == 1   % set the offset and Scale factor based on the first yz plane
            Turbulence.Field.binary_Scale            = Scale;
            Turbulence.Field.binary_Offset           = Offset;
         end

    end

    Nt       = Turbulence.Field.n1 ;
    Ny       = Turbulence.Field.n2 ;
    Nz       = Turbulence.Field.n3 ;  
    Nplanes  = Turbulence.Field.n_planes;
    nf       = length(Turbulence.Field.f); % number of frequency

    % calculate longitudinal coherence for the u component
    r_x = reshape(pdist2(Turbulence.Field.x_planes(:),Turbulence.Field.x_planes(:)),[],1);
    Cohx_squared = exp(-Turbulence.a_x.*sqrt((Turbulence.Field.f.*r_x./Turbulence.URef).^2+...
                (Turbulence.b_x.*r_x).^2));
    Cohx_u = reshape(sqrt(Cohx_squared),[Turbulence.Field.n_planes,Turbulence.Field.n_planes,length(Turbulence.Field.f)]);

    % Cholesky decomposition
    Hx_u = nan(Turbulence.Field.n_planes,Turbulence.Field.n_planes,nf);
    for i = 1:nf
        Hx_u(:,:,i) = chol(Cohx_u(:,:,i),'lower');
    end

    disp('Turbulence unfreezing started...')

    % Get the mean value of the u component
    U_yz_mean = mean(U,3); 
    % fluctuation of the u component 
    u_yz = U-U_yz_mean;              
    % two sided Fourier coefficient u 
    FC_yz_u_2side = fft(reshape(permute(u_yz,[4,1,2,3]),[Nplanes*Ny*Nz,Nt]),[],2);      
    % one sided Fourier coefficient u   
    FC_yz_u_1side = reshape(FC_yz_u_2side(:,1:nf),[Nplanes,Ny*Nz,nf]);

    % introduce the longitudinal coherence in the 3D wind fields
    if exist('pagemtimes','builtin') % this function was introduced in R2020b  
        FC_xyz_u = pagemtimes(Hx_u,real(FC_yz_u_1side))+pagemtimes(Hx_u,imag(FC_yz_u_1side))*1i;         
    else        
        FC_xyz_u = nan(Nplanes,Ny*Nz,nf);
        for i = 1:nf
            FC_xyz_u(:,:,i) = Hx_u(:,:,i)*real(FC_yz_u_1side(:,:,i))+Hx_u(:,:,i)*imag(FC_yz_u_1side(:,:,i))*1i;
        end
    end

    % apply iFFT
    u_xyz = ifft(reshape(FC_xyz_u,[Nplanes*Ny*Nz,nf]),Nt,2,'Symmetric');        
    % add the mean value
    U = permute(reshape(u_xyz,[Nplanes,Nz,Ny,Nt]),[3 2 4 1])+U_yz_mean; % after permute y z t Nplane

    clear U_yz_mean u_xyz velocity


    disp('4D turbulence simulation finished!')

    ubin  = int16((U-Turbulence.Field.binary_Offset(1))./Turbulence.Field.binary_Scale(1));
    vbin  = int16((V-Turbulence.Field.binary_Offset(2))./Turbulence.Field.binary_Scale(2));
    wbin  = int16((W-Turbulence.Field.binary_Offset(3))./Turbulence.Field.binary_Scale(3));

    % export the unfrozen planes except the turbine plane
    data2export = cat(5,ubin(:,:,:,2:end),vbin(:,:,:,2:end),wbin(:,:,:,2:end));    
    data2export = permute(data2export,[5 1 2 3 4]);    %after permute: com y z t Xunfrozen

    disp('Exporting 4D wind field as binary files...')


    fid  = fopen([RunCase{1} '.evo'],'w'); 
    fwrite(fid,int16(Turbulence.Field.n_planes-1),'int16');     % write the head line with the number of unfrozen planes
    fwrite(fid,int16(Turbulence.Field.x_planes(2:end)),'int16');   % write the x positions of unfrozen planes        
    fwrite(fid,data2export(:),'int16');        
    fclose('all');
    disp('Binary file exported!')
   
    movefile([RunCase{1} '.evo'], [TargetDir '\'])
else
    if Turbulence.Field.n_planes == 1
        disp(['There is no upstream plane turbulence required, no ".evo" file is exported!'])
        if ~isfile([TargetDir '\' Turbulence.Field.CaseName '.wnd'])
        movefile([RunCase{1} '.wnd'], [TargetDir '\']) 
        movefile([RunCase{1} '.sum'], [TargetDir '\']) 
        movefile( [RunCase{1} '.inp'], [TargetDir '\']) 
        movefile( [RunCase{1} '_psd.inp'], [TargetDir '\']) 
        end
    end
    
%     &&~isfile([TargetDir '\' Turbulence.Field.CaseName '.evo'])
%     disp('There is no upstream plane turbulence, no ''.evo'' file exported!')


end


if Turbulence.Field.n_planes>1
    if Delete3D == 1  % if we do not need the independent 3D fields any more, delete them
       for iCase = 2: Turbulence.Field.n_planes
           delete([RunCase{iCase} '.wnd'])
           delete([RunCase{iCase} '.sum'])
           delete([RunCase{iCase} '.inp'])
           delete([RunCase{iCase} '_psd.inp'])
       end
    end
end

end

