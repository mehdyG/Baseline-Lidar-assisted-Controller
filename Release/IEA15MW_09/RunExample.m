% IEA15MW_09: IEA 15 MW monopile + realistic wind preview  from a 
% nacelle-based lidar system, full DLC 1.2. 
% Purpose:
% Here, we use a realistic wind preview to demonstrate that the collective
% pitch feedforward controller together with the correct filtering provides
% the reduction in rotor speed variation as predicted by the linear model
% and the coherence. In this example, we assume frozen turbulence, only one 
% 3D turbulence field (y,z,t) at rotor plane is generated.
% Result:
% Change in rotor speed standard deviation:   %
% Change in DLC 1.2 :   %
% Authors:
% David Schlipf, Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

%% Setup
clearvars;
close all;
clc;
addpath('..\MatlabFunctions')

% Parameters (can be adjusted, but will provide different results)
HWindSpeed_vec      = 3:2:30;           % [m/s]         range of wind speeds (operation points)
V_rated             = 10.59;            % [m/s]
nSample             = 6;                % [-]           number of stochastic turbulence field samples
Seed_vec            = [1:nSample];      % [-]           vector of seeds

% Postprocessing Parameter for Weibul distribution and Fatigue calculation
C                               = 2/sqrt(pi)*10;% [m/s] TC I
k                               = 2;            % [-]
WoehlerExponent                 = 4;            % [-]   for steel
N_REF                           = 2e6/(20*8760);% [-]   fraction of 2e6 in 20 years for 1h
Distribution    = k/C*(HWindSpeed_vec/C).^(k-1).*exp(-(HWindSpeed_vec/C).^k);
Weights         = Distribution/sum(Distribution); % relative frequency

% Parameters postprocessing (can be adjusted, but will provide different results)
t_start             = 30;                       % [-]           ignore data before for STD and spectra
nDataPerBlock       = 2^14;                     % [-]           data per block, here 2^14/80 s = 204.8 s, so we have a frequency resolution of 1/204.8 Hz = 0.0049 Hz  
vWindow             = hamming(nDataPerBlock);   % [-]           window for estimation
nFFT                = [];                       % [-]           number of FFT, default: nextpow2(nDataPerBlock); 
nOverlap            = [];                       % [-]           samples of overlap, default: 50% overlap

% Files (should not be be changed)
TurbSimExeFile      = 'TurbSim_x64.exe';
FASTexeFile         = 'openfast_x64.exe';
FASTmapFile         = 'MAP_x64.dll';
SimulationName      = 'IEA-15-240-RWT-Monopile';
TurbSimTemplateFile = 'TurbSim2aInputFileTemplateIEA15MW.inp';
SteadyStateFile     = 'SteadyStatesIEA15MW_Monopile_ROSCO_FAST.mat';
EDFile              = 'IEA-15-240-RWT-Monopile_ElastoDyn.dat';
InflowFile          = 'IEA-15-240-RWT_InflowFile.dat';
SDFile              = 'IEA-15-240-RWT-Monopile_ServoDyn.dat';
LidarFile           = 'MolasNL400_1G_LidarFile.dat';

if ~exist('TurbulentWind','dir')
    mkdir TurbulentWind
end
if ~exist('SimulationResults','dir')
    mkdir SimulationResults
end

%% Preprocessing: generate turbulent wind field

% Copy the adequate TurbSim version to the example folder 
copyfile(['..\TurbSim\',TurbSimExeFile],['TurbulentWind\',TurbSimExeFile])
    
% Generate all wind fields for Different HWindSpeed and seeds

n_HWindSpeed     	= length (HWindSpeed_vec);
for i_HWindSpeed    = 1:n_HWindSpeed

    % Adjust the TurbSim input file
    HWindSpeed      = HWindSpeed_vec(i_HWindSpeed);

    for iSample = 1:nSample
        Seed                = Seed_vec(iSample);
        TurbSimInputFile  	= ['TurbulentWind\HWindSpeed_',num2str(HWindSpeed,'%02d'),'_Seed_',num2str(Seed,'%02d'),'.ipt'];
        TurbSimResultFile  	= ['TurbulentWind\HWindSpeed_',num2str(HWindSpeed,'%02d'),'_Seed_',num2str(Seed,'%02d'),'.wnd'];
        if ~exist(TurbSimResultFile,'file')
            copyfile([TurbSimTemplateFile],TurbSimInputFile)
            ManipulateTXTFile(TurbSimInputFile,'MyHWindSpeed',num2str(HWindSpeed,'%4.1d'));  % adjust HWindSpeed to creat turbulent wind field
            ManipulateTXTFile(TurbSimInputFile,'MyRandSeed1',num2str(Seed));% adjust seed
            dos(['TurbulentWind\',TurbSimExeFile,' ',TurbSimInputFile]);
        end
    end

end

% Clean up
delete(['TurbulentWind\',TurbSimExeFile])

% Initial values and run time
load(SteadyStateFile,'v_0','theta','Omega','x_T','M_g','Info'); 
ManipulateTXTFile([SimulationName,'.fst'],'580   TMax','60   TMax');  % [s]     Set simulation length Based on David's PHD thesis

%% Processing: run simulations

% Copy the adequate OpenFAST version to the example folder
copyfile(['..\OpenFAST\',FASTexeFile],FASTexeFile)
copyfile(['..\OpenFAST\',FASTmapFile],FASTmapFile)

% Simulate with all wind fields

for i_HWindSpeed    = 1:n_HWindSpeed
    HWindSpeed      = HWindSpeed_vec(i_HWindSpeed);

    % Set initial values
    MyBlPitch   = num2str(rad2deg  (interp1(v_0,theta,HWindSpeed)),'%5.2f');
    MyRotSpeed  = num2str(radPs2rpm(interp1(v_0,Omega,HWindSpeed)),'%5.2f');
    MyTTDspFA   = num2str(         (interp1(v_0,x_T  ,HWindSpeed)),'%5.2f');
    MyM_g       = num2str(         (interp1(v_0,M_g  ,HWindSpeed)),'%5.2f');

    ManipulateTXTFile(EDFile,'MyBlPitch',   MyBlPitch);
    ManipulateTXTFile(EDFile,'MyRotSpeed',  MyRotSpeed);
    ManipulateTXTFile(EDFile,'MyTTDspFA',   MyTTDspFA);
    ManipulateTXTFile(SDFile,'MyM_g',       MyM_g);

    for iSample = 1:nSample

        % Adjust the InflowWind file
        Seed                = Seed_vec(iSample);
        WindFileRoot        = ['TurbulentWind\HWindSpeed_',num2str(HWindSpeed,'%02d'),'_Seed_',num2str(Seed,'%02d')];
        ManipulateTXTFile('IEA-15-240-RWT_InflowFile.dat','MyFilenameRoot',WindFileRoot);

        % Run FB
        FASTresultFile      = ['SimulationResults\HWindSpeed_',num2str(HWindSpeed,'%02d'),...
            '_Seed_',num2str(Seed,'%02d'),'_FlagLAC_0.outb'];
        if ~exist(FASTresultFile,'file')
            ManipulateTXTFile('ROSCO_v2d6.IN','1 ! FlagLAC','0 ! FlagLAC'); % disable LAC
            dos([FASTexeFile,' ',SimulationName,'.fst']);
            movefile([SimulationName,'.outb'],FASTresultFile)
        end

        % Run FB+FF if HWindSpeed > V_rated
        if HWindSpeed > V_rated
            FASTresultFile      = ['SimulationResults\HWindSpeed_',num2str(HWindSpeed,'%02d'),...
                '_Seed_',num2str(Seed,'%02d'),'_FlagLAC_1.outb'];
            if ~exist(FASTresultFile,'file')
                ManipulateTXTFile('ROSCO_v2d6.IN','0 ! FlagLAC','1 ! FlagLAC'); % enable LAC
                dos([FASTexeFile,' ',SimulationName,'.fst']);
                movefile([SimulationName,'.outb'],FASTresultFile)
            end
        end
        % Reset the InflowWind file again
        ManipulateTXTFile('IEA-15-240-RWT_InflowFile.dat',WindFileRoot,'MyFilenameRoot');
    end

    % Reset the ElastoDyn and ServoDyn file again
    ManipulateTXTFile(EDFile,MyBlPitch, 'MyBlPitch');
    ManipulateTXTFile(EDFile,MyRotSpeed,'MyRotSpeed');
    ManipulateTXTFile(EDFile,MyTTDspFA, 'MyTTDspFA');
    ManipulateTXTFile(SDFile,MyM_g,     'MyM_g');

end

% Clean up
delete(FASTexeFile)
delete(FASTmapFile)

% Reset .fst file
ManipulateTXTFile([SimulationName,'.fst'],'60   TMax','580   TMax');  

%% Postprocessing: evaluate data

for i_HWindSpeed    = 1:n_HWindSpeed

    HWindSpeed      = HWindSpeed_vec(i_HWindSpeed);
    for iSample = 1:nSample

        % Load data
        Seed                = Seed_vec(iSample);

        FASTresultFile      = ['SimulationResults\HWindSpeed_',num2str(HWindSpeed,'%02d'),...
                                    '_Seed_',num2str(Seed,'%02d'),'_FlagLAC_0.outb'];
        FB                  = ReadFASTbinaryIntoStruct(FASTresultFile);

        if HWindSpeed > V_rated
            FASTresultFile  = ['SimulationResults\HWindSpeed_',num2str(HWindSpeed,'%02d'),...
                '_Seed_',num2str(Seed,'%02d'),'_FlagLAC_1.outb'];
            FBFF            = ReadFASTbinaryIntoStruct(FASTresultFile);
        else
            FBFF            = FB;
        end

        P_Seedmean_FB(iSample)      = mean(FB.GenPwr(FB.Time  >t_start));
        MyT_FB(:,iSample)           = FB.TwrBsMyt(FB.Time  >t_start);

        P_Seedmean_FBFF(iSample)    = mean(FBFF.GenPwr(FB.Time  >t_start));
        MyT_FBFF(:,iSample)         = FBFF.TwrBsMyt(FB.Time  >t_start);

        % Calculate standard deviation
        STD_RotSpeed_FB  (iSample)              = std(FB.RotSpeed  (FB.Time  >t_start));
        STD_RotSpeed_FBFF(iSample)              = std(FBFF.RotSpeed(FBFF.Time>t_start));

    end

    MyT_FB_SeedAve              = mean (MyT_FB,2);  % Average of MyT time seiries over all seed results, the result will be a time series to calculate DEL
    MyT_FBFF_SeedAve            = mean (MyT_FBFF,2);% ...

    % DEL calculation, Could be a function
    RainF_FB                    = rainflow(MyT_FB_SeedAve);
    Count_FB                    = RainF_FB(:,1);
    Range_FB                    = RainF_FB(:,2);
    DEL_MyT_FB(i_HWindSpeed)    = (sum(Range_FB.^WoehlerExponent.*Count_FB)/N_REF).^(1/WoehlerExponent);

    RainF_FBFF                  = rainflow(MyT_FBFF_SeedAve);
    Count_FBFF                  = RainF_FBFF(:,1);
    Range_FBFF                  = RainF_FBFF(:,2);
    DEL_MyT_FBFF(i_HWindSpeed)  = (sum(Range_FBFF.^WoehlerExponent.*Count_FBFF)/N_REF).^(1/WoehlerExponent);

    % Mean Power calculation
    P_mean_FB(i_HWindSpeed)     = mean(P_Seedmean_FB);
    P_mean_FBFF(i_HWindSpeed)   = mean(P_Seedmean_FBFF);

    %Standard Deviation calculation 
    STD_RotSpeed_FB_seedAve  (i_HWindSpeed) = mean(STD_RotSpeed_FB);
    STD_RotSpeed_FBFF_seedAve(i_HWindSpeed) = mean(TD_RotSpeed_FBFF);

end

%% Calculate DLC 1.2 
% Calculate Annual energy production and lifetime-weighted DEL
AEP_FB  = sum(Weights.*P_mean_FB)*8760;                                     
DEL_FB  = sum(Weights.*DEL_MyT_FB.^WoehlerExponent).^(1/WoehlerExponent);   

AEP_FBFF  = sum(Weights.*P_mean_FBFF)*8760;                                     
DEL_FBFF  = sum(Weights.*DEL_MyT_FBFF.^WoehlerExponent).^(1/WoehlerExponent);   

% display results
fprintf('Change in AEP:  %4.1f %%\n',...
    (AEP_FBFF-AEP_FB)/AEP_FB*100)   
fprintf('Change in DEL:  %4.1f %%\n',...
    (DEL_FBFF-DEL_FB)/DEL_FB*100)   

%% Plot 

% Plot STD Rotor Speed
figure('Name','Rotor Speed STD mean Value')
hold on; grid on; box on
plot(HWindSpeed_vec,       STD_RotSpeed_FB_seedAve);
plot(HWindSpeed_vec,     STD_RotSpeed_FBFF_seedAve);
ylabel('STD [rpm]');
legend('feedback only','feedback-feedforward')
xlabel('Mean Wind Speed [m/s]')



