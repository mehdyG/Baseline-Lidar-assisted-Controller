% LAC Test IEA15MW_07:  IEA 15 MW + Realistic wind preview using 4D Mann
% turbulence model(include turbulence evolution)
% Purpose:
% Here, we use a realistic wind preview to demonstrate that the collective
% pitch feedforward controller together with the correct filtering provides
% the reduction in rotor speed variation as predicted by the linear model
% and the coherence. In this example, we assume frozen turbulence, only one 
% 3D turbulence field (y,z,t) at rotor plane is generated.
% Result:
% Change in rotor speed standard deviation:  -22.3 %
% Authors:
% Feng Guo, Wei Fu, David Schlipf
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

%% Setup
clearvars;
close all;
clc;
addpath('..\MatlabFunctions')
addpath('..\MatlabFunctions\AnalyticlModel')
addpath('MannTurbFunctions')

% Seeds (can be adjusted, but will provide different results)
nSample             = 1;                        % [-]           number of stochastic turbulence field samples
Seed_vec            = [1:nSample];              % [-]           vector of seeds

% Parameters postprocessing (can be adjusted, but will provide different results)
t_start             = 10;                       % [-]           ignore data before for STD and spectra
nDataPerBlock       = 2^14;                     % [-]           data per block, here 2^14/80 s = 204.8 s, so we have a frequency resolution of 1/204.8 Hz = 0.0049 Hz  
vWindow             = hamming(nDataPerBlock);   % [-]           window for estimation
nFFT                = [];                       % [-]           number of FFT, default: nextpow2(nDataPerBlock); 
nOverlap            = [];                       % [-]           samples of overlap, default: 50% overlap

% Files (should not be be changed)
MannTurbExeFile     = 'MannTurb4D_v2.exe';
FASTexeFile         = 'openfast_x64.exe';
FASTmapFile         = 'MAP_x64.dll';
SimulationName      = 'IEA-15-240-RWT-Monopile';
MannTurbTemplateFile = 'InputSample4DMann.inp';
if ~exist('TurbulentWind','dir')
    mkdir TurbulentWind
end
if ~exist('SimulationResults','dir')
    mkdir SimulationResults
end
%% Preprocessing: generate turbulent wind field
    
% Copy the adequate TurbSim version to the example folder 
copyfile(['..\MannTurb4D\',MannTurbExeFile],['TurbulentWind\',MannTurbExeFile])
copyfile(['..\MannTurb4D\',MannTurbExeFile],['TurbulentWind\',MannTurbTemplateFile])
      
% Generate all wind fields
for iSample = 1:nSample        
    Seed             = Seed_vec(iSample);
    TurbSimResultFile= ['TurbulentWind\Mann_URef_18_Seed_',num2str(Seed,'%02d'),'.wnd'];
    MannTurb         = MannTurbConfig_IEC('Neutral_IEC1B',18);
%     MannTurb         = GetMannTensorAndLifeTime(MannTurb,25);
    %MannTurb         = MannTurbConfig('Stable_IEC1A_L30',16);
    fd               = 240;
    MannTurb         = MannTurbFieldConfig('4096x32x40_8m_H150',MannTurb,[0 fd/MannTurb.URef],iSample); % Unfrozen at certain planes!! FG, please adjust df according to your lidar      
    
    if ~exist(TurbSimResultFile,'file')
    MannTurb         = GenerateMann4DTurb(MannTurb,'TurbulentWind','TurbulentWind');
    MannTurb         = GetVelocityField(MannTurb,'TurbulentWind',1);
                       MannTurbtoBladedEvo(MannTurb,'TurbulentWind');                   
    % rename file
    movefile(['TurbulentWind\' MannTurb.Field.CaseName '.wnd'], ['TurbulentWind\Mann_URef_18_Seed_' num2str(Seed,'%02d'),'.wnd']); 
    movefile(['TurbulentWind\' MannTurb.Field.CaseName '.sum'], ['TurbulentWind\Mann_URef_18_Seed_' num2str(Seed,'%02d'),'.sum']); 
    movefile(['TurbulentWind\' MannTurb.Field.CaseName '.evo'], ['TurbulentWind\Mann_URef_18_Seed_' num2str(Seed,'%02d'),'.evo']); 
    
    delete(['TurbulentWind\' MannTurb.Field.CaseName '.mt4d'])
    delete(['TurbulentWind\' MannTurb.Field.CaseName '.inp'])
    end
%     TurbSimInputFile  	= ['TurbulentWind\Mann_URef_18_Seed_',num2str(Seed,'%02d'),'.ipt'];
%     TurbSimResultFile  	= ['TurbulentWind\Mann_URef_18_Seed_',num2str(Seed,'%02d'),'.wnd'];
%     if ~exist(TurbSimResultFile,'file')
%         copyfile([TurbSimTemplateFile],TurbSimInputFile)
%         ManipulateTXTFile(TurbSimInputFile,'MyRandSeed1',num2str(Seed));% adjust seed
%         dos(['TurbulentWind\',TurbSimExeFile,' ',TurbSimInputFile]);
%     end
%     
%     MannTurb         = MannTurbConfig_IEC('Neutral_IEC1B',18);
%     Turbulence.alpha_shear = 0.14; % for offshore according to IEC
%     MannTurb         = GetMannTensorAndLifeTime(MannTurb,25);
%     %MannTurb         = MannTurbConfig('Stable_IEC1A_L30',16);
%     MannTurb         = MannTurbFieldConfig('4096x64x64_8m_H150',MannTurb,[0 Lidar.Trajectory.RangeX(1)./MannTurb.URef],iSeed); % Unfrozen at certain planes [0 50 110 170]         
%     MannTurb         = GenerateMann4DTurb(MannTurb,'TurbResultsMann','MannTurb4D');
%     MannTurb         = GetVelocityField(MannTurb,'TurbResultsMann',1);
%                        MannTurbtoBladedEvo(MannTurb,'TurbResultsMann');
end
    
% Clean up
delete(['TurbulentWind\',MannTurbExeFile])
delete(['TurbulentWind\',MannTurbTemplateFile])

%% Processing: run simulations

% Copy the adequate OpenFAST version to the example folder
copyfile(['..\OpenFAST\',FASTexeFile],FASTexeFile)
copyfile(['..\OpenFAST\',FASTmapFile],FASTmapFile)

% Simulate with all wind fields
for iSample = 1:nSample
    
    % Adjust the InflowWind file
    Seed                = Seed_vec(iSample);
    WindFileRoot        = ['TurbulentWind\Mann_URef_18_Seed_',num2str(Seed,'%02d')];
    ManipulateTXTFile('IEA-15-240-RWT_InflowFile.dat','MyFilenameRoot',WindFileRoot);
    
    % Run FB    
    FASTresultFile      = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_0.outb'];
    if ~exist(FASTresultFile,'file')    
        ManipulateTXTFile('ROSCO_15MP.IN','1 ! FlagLAC','0 ! FlagLAC'); % disable LAC
        dos([FASTexeFile,' ',SimulationName,'.fst']);
        movefile([SimulationName,'.outb'],FASTresultFile)
    end
   
    % Run FB+FF    
    FASTresultFile      = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_1.outb'];
    if ~exist(FASTresultFile,'file')    
        ManipulateTXTFile('ROSCO_15MP.IN','0 ! FlagLAC','1 ! FlagLAC'); % enable LAC
        dos([FASTexeFile,' ',SimulationName,'.fst']);
        movefile([SimulationName,'.outb'],FASTresultFile)
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
    FB                  = ReadFASTbinaryIntoStruct(FASTresultFile);
    FASTresultFile      = ['SimulationResults\URef_18_Seed_',num2str(Seed,'%02d'),'_FlagLAC_1.outb'];
    FBFF                = ReadFASTbinaryIntoStruct(FASTresultFile);

    % Plot time results
    figure('Name',['Seed ',num2str(Seed)])
    hold on; grid on; box on
    plot(FB.Time,       FB.RotSpeed);
    plot(FBFF.Time,     FBFF.RotSpeed);
    ylabel('RotSpeed [rpm]');
    legend('feedback only','feedback-feedforward')
    xlabel('time [s]')

    % Estimate spectra
    Fs                                      = 80; % [Hz]  sampling frequenzy, same as in *.fst
    [S_RotSpeed_FB_est(iSample,:),f_est]	= pwelch(detrend(FB.RotSpeed  (FB.Time  >t_start)),vWindow,nOverlap,nFFT,Fs);
    [S_RotSpeed_FBFF_est(iSample,:),~]      = pwelch(detrend(FBFF.RotSpeed(FBFF.Time>t_start)),vWindow,nOverlap,nFFT,Fs);

    % Calculate standard deviation
    STD_RotSpeed_FB  (iSample)              = std(FB.RotSpeed  (FB.Time  >t_start));
    STD_RotSpeed_FBFF(iSample)              = std(FBFF.RotSpeed(FBFF.Time>t_start));

end

%% Calculate rotor speed spectra by analytical model

% steady state operating point for 
theta_OP                 = 0.2714;
Omega_OP                 = 0.7920;
v_0_OP                   = 18;
f_delay                  = 0.025;
ROSCOInFileName          = 'ROSCO_15MP.IN';
RotorPerformanceFile     = 'Cp_Ct_Cq.IEA15MW.txt';
LidarInputFileName       = 'MolasNL400_1G_LidarFile.dat';
LDPInputFileName         = 'LDP_NL400.IN';
SpectralModelFileName    = 'LidarRotorSpectra_IEA15MW_MolasNL400.mat';
AnalyticalModel          = AnalyticalRotorSpeedSpectrum(v_0_OP,theta_OP,Omega_OP,f_delay,...
    ROSCOInFileName,RotorPerformanceFile,LidarInputFileName,LDPInputFileName,SpectralModelFileName);

%% Plot spectra
figure('Name','Simulation results')

hold on; grid on; box on
p1 = plot(AnalyticalModel.f,AnalyticalModel.S_Omega_r_FB.*(radPs2rpm(1))^2,'--','Color',[0 0.4470 0.7410]);
p2 = plot(AnalyticalModel.f,AnalyticalModel.S_Omega_r_FF.*(radPs2rpm(1))^2,'--','Color',[0.8500 0.3250 0.0980]);
p3 = plot(f_est ,mean(S_RotSpeed_FB_est,1),'-','Color',[0 0.4470 0.7410]);
p4 = plot(f_est ,mean(S_RotSpeed_FBFF_est,1),'-','Color',[0.8500 0.3250 0.0980]);

set(gca,'Xscale','log')
set(gca,'Yscale','log')
xlabel('frequency [Hz] ')
ylabel('Spectra RotSpeed [(rpm)^2Hz^{-1}]')
legend([p1 p2 p3 p4],'FB-only Analytical','FBFF Analytical','FB-only Estimated','FBFF Estimated')

% display results
fprintf('Change in rotor speed standard deviation:  %4.1f %%\n',...
    (mean(STD_RotSpeed_FBFF)/mean(STD_RotSpeed_FB)-1)*100)   

