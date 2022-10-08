% LAC Test IEA15MW_04:  IEA 15 MW UMaineSemi + Realistic wind preview
% Purpose:
% Here, we use a realistic wind preview to demonstrate that the collective
% pitch feedforward controller together with the correct filtering provides
% the reduction in rotor speed variation as predicted by the linear model
% and the coherence. In this example, we assume frozen turbulence, only one 
% 3D turbulence field (y,z,t) at rotor plane is generated.
% Result:
% Change in rotor speed standard deviation:  -19.6 %
% Authors:
% David Schlipf, Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

%% Setup
clearvars;
close all;
clc;
addpath('..\MatlabFunctions')
addpath('..\MatlabFunctions\AnalyticlModel')

% Seeds (can be adjusted, but will provide different results)
nSample             = 5;                        % [-]           number of stochastic turbulence field samples
Seed_vec            = [1:nSample];              % [-]           vector of seeds

% Parameters postprocessing (can be adjusted, but will provide different results)
t_start             = 10;                       % [-]           ignore data before for STD and spectra
nDataPerBlock       = 2^14;                     % [-]           data per block, here 2^14/80 s = 204.8 s, so we have a frequency resolution of 1/204.8 Hz = 0.0049 Hz  
vWindow             = hamming(nDataPerBlock);   % [-]           window for estimation
nFFT                = [];                       % [-]           number of FFT, default: nextpow2(nDataPerBlock); 
nOverlap            = [];                       % [-]           samples of overlap, default: 50% overlap

% Files (should not be be changed)
TurbSimExeFile      = 'TurbSim_x64.exe';
FASTexeFile         = 'openfast_x64.exe';
FASTmapFile         = 'MAP_x64.dll';
SimulationName      = 'IEA-15-240-RWT-UMaineSemi';
TurbSimTemplateFile = 'TurbSim2aInputFileTemplateIEA15MW.inp';
if ~exist('TurbulentWind','dir')
    mkdir TurbulentWind
end
if ~exist('SimulationResults','dir')
    mkdir SimulationResults
end
%% Preprocessing: generate turbulent wind field
    
% Copy the adequate TurbSim version to the example folder 
copyfile(['..\TurbSim\',TurbSimExeFile],['TurbulentWind\',TurbSimExeFile])
    
% Generate all wind fields
for iSample = 1:nSample        
    Seed                = Seed_vec(iSample);
    TurbSimInputFile  	= ['TurbulentWind\URef_18_Seed_',num2str(Seed,'%02d'),'.ipt'];
    TurbSimResultFile  	= ['TurbulentWind\URef_18_Seed_',num2str(Seed,'%02d'),'.wnd'];
    if ~exist(TurbSimResultFile,'file')
        copyfile([TurbSimTemplateFile],TurbSimInputFile)
        ManipulateTXTFile(TurbSimInputFile,'MyRandSeed1',num2str(Seed));% adjust seed
        dos(['TurbulentWind\',TurbSimExeFile,' ',TurbSimInputFile]);
    end
end
    
% Clean up
delete(['TurbulentWind\',TurbSimExeFile])

%% Processing: run simulations

% Copy the adequate OpenFAST version to the example folder
copyfile(['..\OpenFAST\',FASTexeFile],FASTexeFile)
copyfile(['..\OpenFAST\',FASTmapFile],FASTmapFile)

% Simulate with all wind fields
for iSample = 1:nSample
    
    % Adjust the InflowWind file
    Seed                = Seed_vec(iSample);
    WindFileRoot        = ['TurbulentWind\URef_18_Seed_',num2str(Seed,'%02d')];
    ManipulateTXTFile('IEA-15-240-RWT_InflowFile.dat','MyFilenameRoot',WindFileRoot);
    
    % Run FB    
    FASTresultFile      = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_0.outb'];
    ROSCOresultFile     = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_0.dbg'];
    if ~exist(FASTresultFile,'file')    
        ManipulateTXTFile('ROSCO_v2d6.IN','1 ! FlagLAC','0 ! FlagLAC'); % disable LAC
        dos([FASTexeFile,' ',SimulationName,'.fst']);
        movefile([SimulationName,'.outb'],FASTresultFile)
        movefile([SimulationName,'.RO.dbg'],ROSCOresultFile)   % store rosco output file
    end
   
    % Run FB+FF    
    FASTresultFile      = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_1.outb'];
    ROSCOresultFile     = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_1.dbg'];
    if ~exist(FASTresultFile,'file')    
        ManipulateTXTFile('ROSCO_v2d6.IN','0 ! FlagLAC','1 ! FlagLAC'); % enable LAC
        dos([FASTexeFile,' ',SimulationName,'.fst']);
        movefile([SimulationName,'.outb'],FASTresultFile)
        movefile([SimulationName,'.RO.dbg'],ROSCOresultFile)   % store rosco output file
    end    
    
    % Reset the InflowWind file again
    ManipulateTXTFile('IEA-15-240-RWT_InflowFile.dat',WindFileRoot,'MyFilenameRoot');
end


% Clean up
delete(FASTexeFile)
delete(FASTmapFile)

%% Postprocessing: evaluate data

for iSample = 1:nSample    

    % Load data
    Seed                = Seed_vec(iSample);
    FASTresultFile      = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_0.outb'];
    [FB_Data,   ~, ~, ~, ~]             = ReadFASTbinary(FASTresultFile);
    FASTresultFile      = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_1.outb'];
    [FBFF_Data, ChannelName, ~, ~, ~] 	= ReadFASTbinary(FASTresultFile);

    % Get signals from FB Data
    Time_FB         = FB_Data(:,strcmp(ChannelName,'Time'));
    RotSpeed_FB     = FB_Data(:,strcmp(ChannelName,'RotSpeed'));
    PtfmPitch_FB    = FB_Data(:,strcmp(ChannelName,'PtfmPitch'));

    % Get signals from FBFF Data
    Time_FBFF     	= FBFF_Data(:,strcmp(ChannelName,'Time'));
    RotSpeed_FBFF 	= FBFF_Data(:,strcmp(ChannelName,'RotSpeed'));  
    PtfmPitch_FBFF    = FBFF_Data(:,strcmp(ChannelName,'PtfmPitch'));

    
    wind = FBFF_Data(:,strcmp(ChannelName,'Wind1VelX'));

    % Plot time results
%     figure('Name',['Seed ',num2str(Seed)])
%     hold on; grid on; box on
%     plot(Time_FB,  RotSpeed_FB);
%     plot(Time_FBFF,RotSpeed_FBFF);
%     ylabel('RotSpeed [rpm]');
%     legend('feedback only','feedback-feedforward')
%     xlabel('time [s]')

    % Estimate spectra
    Fs                                      = 80; % [Hz]  sampling frequenzy, same as in *.fst
    [S_RotSpeed_FB_est(iSample,:),f_est]	= pwelch(detrend(RotSpeed_FB  (Time_FB>t_start)),  vWindow,nOverlap,nFFT,Fs);
    [S_RotSpeed_FBFF_est(iSample,:),~]      = pwelch(detrend(RotSpeed_FBFF(Time_FBFF>t_start)),vWindow,nOverlap,nFFT,Fs);
    [S_PtfmPitch_FB_est(iSample,:),f_est]	= pwelch(detrend(PtfmPitch_FB  (Time_FB>t_start)),  vWindow,nOverlap,nFFT,Fs);
    [S_PtfmPitch_FBFF_est(iSample,:),~]     = pwelch(detrend(PtfmPitch_FBFF(Time_FBFF>t_start)),vWindow,nOverlap,nFFT,Fs);
    
    
    
    % Calculate standard deviation
    STD_RotSpeed_FB  (iSample)              = std(RotSpeed_FB   (Time_FB>t_start));
    STD_RotSpeed_FBFF(iSample)              = std(RotSpeed_FBFF (Time_FB>t_start));
    STD_PtfmPitch_FB  (iSample)             = std(PtfmPitch_FB   (Time_FB>t_start));
    STD_PtfmPitch_FBFF(iSample)             = std(PtfmPitch_FBFF (Time_FB>t_start));
    

end

%% Calculate rotor speed spectra by analytical model

% steady state operating point for 
theta_OP                 = 0.2714;
Omega_OP                 = 0.7920;
v_0_OP                   = 18;
f_delay                  = 0.08;
ROSCOInFileName          = 'ROSCO_v2d6.IN';
RotorPerformanceFile     = 'Cp_Ct_Cq.IEA15MW.txt';
LidarInputFileName       = 'MolasNL400_1G_LidarFile.dat';
LDPInputFileName         = 'LDP_v2.IN';
SpectralModelFileName    = 'LidarRotorSpectra_IEA15MW_MolasNL400.mat';
AnalyticalModel          = AnalyticalRotorSpeedSpectrum(v_0_OP,theta_OP,Omega_OP,f_delay,...
    ROSCOInFileName,RotorPerformanceFile,LidarInputFileName,LDPInputFileName,SpectralModelFileName);



%% Plot spectra
figure('Name','Simulation results: Rotor Speed')

hold on; grid on; box on
p1 = plot(f_est ,mean(S_RotSpeed_FB_est,1),'r-');
p2 = plot(f_est ,mean(S_RotSpeed_FBFF_est,1),'b-');

set(gca,'Xscale','log')
set(gca,'Yscale','log')
xlabel('frequency [Hz] ')
ylabel('Spectra RotSpeed [(rmp)^2Hz^{-1}]')
legend([p1 p2],'FB-only Estimated','FBFF Estimated')

figure('Name','Simulation results: Platform Pitch')

hold on; grid on; box on
p1 = plot(f_est ,mean(S_PtfmPitch_FB_est,1),'r-');
p2 = plot(f_est ,mean(S_PtfmPitch_FBFF_est,1),'b-');

set(gca,'Xscale','log')
set(gca,'Yscale','log')
xlabel('frequency [Hz] ')
ylabel('Spectra Platform Pitch [(deg)^2Hz^{-1}]')
legend([p1 p2],'FB-only Estimated','FBFF Estimated')




% display results

fprintf('Change in rotor speed standard deviation:  %4.1f %%\n',...
    (mean(STD_RotSpeed_FBFF)/mean(STD_RotSpeed_FB)-1)*100)       

fprintf('Change in platform pitch standard deviation:  %4.1f %%\n',...
    (mean(STD_PtfmPitch_FBFF)/mean(STD_PtfmPitch_FB)-1)*100)       
