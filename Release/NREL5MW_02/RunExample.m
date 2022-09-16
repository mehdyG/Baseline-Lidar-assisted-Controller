% LAC Test NREL5MW_02:  NREL 5 MW + Realistic wind preview
% Purpose:
% Here, we use a realistic wind preview to demonstrate that the collective
% pitch feedforward controller together with the correct filtering provides
% the reduction in rotor speed variation as predicted by the linear model
% and the coherence.
% Result:       
% Change in rotor speed standard deviation:  -18.4 %
% Authors: 		
% David Schlipf, Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

%% Setup
clearvars; 
close all; 
clc;
addpath('..\MatlabFunctions')

set(groot,'defaultTextInterpreter','latex')
set(groot,'defaultTextFontWeight','bold')
set(groot,'defaultAxesTickLabelInterpreter','latex')
set(groot,'defaultAxesFontWeight','bold')
set(groot,'defaultLegendInterpreter','latex')
set(groot,'defaultFigureColor','w')
set(groot,'defaultTextFontSize',12)
set(groot,'defaultAxesFontSize',12)
set(groot,'defaultLineLineWidth',1.2)

orange    = [0.8500 0.3250 0.0980];
green     = [0.4660 0.6740 0.1880];
yellow    = [0.9290, 0.6940, 0.1250];
lightblue = [0.3010, 0.7450, 0.9330];

rng(12)
nSamples      = 6;                    % number of stochastic turbulence field samples
Seed_vec      = randi(10^5,[1 6]);    % A vector contains all seeds
FB_Data       = cell(size(Seed_vec)); % Allocating
FBFF_Data     = cell(size(Seed_vec)); % Allocating


for iSample = 1:nSamples
    

% Copy the adequate OpenFAST version to the example folder
FASTexeFile     = 'openfast_x64.exe';
FASTmapFile     = 'MAP_x64.dll';
SimulationName  = 'NREL-5.0-126-RWT';
copyfile(['..\OpenFAST\',FASTexeFile],FASTexeFile)
copyfile(['..\OpenFAST\',FASTmapFile],FASTmapFile)
copyfile(['..\TurbSim2a'],'TurbSim2a')


%% Preprocessing to generate turbulent wind field
URef           = 18;        % reference mean wind speed [m/s]
TurbRNGSeed    = Seed_vec(iSample);
KaimalTurb     = KaimalTurbConfig('IEC_Class_1A',URef);  % Adjust the parameter in this configuration function for different turbulence characteristics
x_planes       = [0];                                    % longitudinal coordinate of x planes
                                                         % In this example, we assume frozen turbulence, 
                                                         % only one 3D turbulence field (y,z,t) at rotor plane is generated
                                                         % If evolving turbulence is required, set x_unfrozen     = [0 120], 
                                                         % then another 3D turbulence field at x = 120m will also be generated
KaimalTurb     = KaimalTurbFieldConfig('4096x25x25',KaimalTurb,x_planes,TurbRNGSeed);      
KaimalTurb     = CalculateKaimalSpectra(KaimalTurb);  
WindFileDir    = 'Wind';
KaimalTurb     = Generate4DKaimalTurb(KaimalTurb,'Wind','TurbSim2a',0,0);

% adjust the OpenFAST input file to let it be able to find the wind field input
ManipulateTXTFile('NREL-5.0-126-RWT_InflowFile.dat','not specified',...
                  [WindFileDir '\' KaimalTurb.Field.CaseName]);        % Change the name of of the turbulent wind input
                                                    

%% Run FB
ManipulateTXTFile('ROSCO2.IN','1                   ! FlagLAC',...
                              '0                   ! FlagLAC'); % disable LAC


if ~exist('SimResults','dir')
mkdir SimResults
end

if ~exist(['SimResults\FB_only_SampleNum_' num2str(iSample) '.outb'],'file')||...
    ~exist(['SimResults\FB_only_SampleNum_' num2str(iSample) '.RO.dbg'],'file')
    dos([FASTexeFile,' ',SimulationName,'.fst']);
    movefile([SimulationName,'.outb'],['SimResults\FB_only_SampleNum_' num2str(iSample) '.outb'])
    movefile([SimulationName,'.RO.dbg'],['SimResults\FB_only_SampleNum_' num2str(iSample) '.RO.dbg'])
end

[FB_Data{iSample}, ~, ~, ~, ~]               = ReadFASTbinary(['SimResults\FB_only_SampleNum_' num2str(iSample) '.outb']);

%% Run FBFF 
ManipulateTXTFile('ROSCO2.IN','0                   ! FlagLAC',...
                              '1                   ! FlagLAC'); % enable LAC
 
if ~exist(['SimResults\FBFF_SampleNum_' num2str(iSample) '.outb'],'file')||...
    ~exist(['SimResults\FBFF_SampleNum_' num2str(iSample) '.RO.dbg'],'file')
    dos([FASTexeFile,' ',SimulationName,'.fst']);
    movefile([SimulationName,'.outb'],['SimResults\FBFF_SampleNum_' num2str(iSample) '.outb'])
    movefile([SimulationName,'.RO.dbg'],['SimResults\FBFF_SampleNum_' num2str(iSample) '.RO.dbg'])
end

[FBFF_Data{iSample}, ChannelName, ~, ~, ~] 	= ReadFASTbinary(['SimResults\FBFF_SampleNum_' num2str(iSample) '.outb']);
                          

%% Restore Inflow file to default
ManipulateTXTFile('NREL-5.0-126-RWT_InflowFile.dat',[WindFileDir '\' KaimalTurb.Field.CaseName],...
                  'not specified'); % Change the name of of the turbulent wind input



end


%% Clean up and back to default status
delete(FASTexeFile)
delete(FASTmapFile)
delete('TurbSim2a\*')
rmdir TurbSim2a



%% Estimate Spectra from the simulated time series

% parameters for 'pwelch' method
Tstart           = 5;      % drop the initial 5 second
nFFT             = 16384;  % number of FFT 
nDataPerBlock    = 16384;  % data per block
vWindow          = hamming(nDataPerBlock); % hamming window
noverlap         = [];     % 50% over lap
Fs               = 1/(FB_Data{1}(2,strcmp(ChannelName,'Time'))-FB_Data{1}(1,strcmp(ChannelName,'Time')));

Std_Omega_r_FB      = zeros(nSamples,1);
Std_Omega_r_FBFF    = zeros(nSamples,1);

for iSample = 1:nSamples
    if iSample == 1
        [S_temp ,f_est]            = pwelch(detrend(FB_Data{iSample}(:,strcmp(ChannelName,'RotSpeed'))),vWindow,noverlap,nFFT,Fs); 
        S_Omega_r_FB_est           = zeros(nSamples,max(size(S_temp)));
        S_Omega_r_FBFF_est         = zeros(nSamples,max(size(S_temp)));
        
        [S_Omega_r_FB_est(iSample,:),~]  = pwelch(detrend(FB_Data{iSample}(:,strcmp(ChannelName,'RotSpeed'))),vWindow,noverlap,nFFT,Fs);
        [S_Omega_r_FBFF_est(iSample,:),~]    = pwelch(detrend(FBFF_Data{iSample}(:,strcmp(ChannelName,'RotSpeed'))),vWindow,noverlap,nFFT,Fs); 
       
    else
        [S_Omega_r_FB_est(iSample,:),~]  = pwelch(detrend(FB_Data{iSample}(:,strcmp(ChannelName,'RotSpeed'))),vWindow,noverlap,nFFT,Fs);
        [S_Omega_r_FBFF_est(iSample,:),~]    = pwelch(detrend(FBFF_Data{iSample}(:,strcmp(ChannelName,'RotSpeed'))),vWindow,noverlap,nFFT,Fs); 
    end
    
    Std_Omega_r_FB(iSample)              = std(FB_Data{iSample}(:,strcmp(ChannelName,'RotSpeed')));
    Std_Omega_r_FBFF(iSample)            = std(FBFF_Data{iSample}(:,strcmp(ChannelName,'RotSpeed')));
end


%% Load analytical model  FG: to be further improved
load('AnalyticalModel.mat')
Turbine.Parameter.Turbine.i = 1/97;


%% Plot         
ScreenSize = get(0,'ScreenSize');
figure('Name','Simulation results','position',[.1 .1 .8 .8].*ScreenSize([3,4,3,4]))


figure(1)
hold on
p1 = plot(AnalyticalModel.f, AnalyticalModel.S_Omega_FB.*radPs2rpm(1).^2.*Turbine.Parameter.Turbine.i.^2 ,'--','Color',orange);
p2 = plot(AnalyticalModel.f, AnalyticalModel.S_Omega_FBFF.*radPs2rpm(1).^2.*Turbine.Parameter.Turbine.i.^2,'--','Color',lightblue);
p3 = plot(f_est ,mean(S_Omega_r_FB_est,1),'-','Color',orange);
p4 = plot(f_est ,mean(S_Omega_r_FBFF_est,1),'-','Color',lightblue);
grid on 
set(gca,'Xscale','log')
set(gca,'Yscale','log')
xlabel('$f$ [Hz] ')
ylabel({'$S_{\Omega}$';'[rmp$^2$Hz$^{-1}$]'})
legend([p1 p2 p3 p4],'FB-only Analytical','FBFF Analytical','FB-only Estimated','FBFF Estimated')

% linkaxes(MyAxes,'x');

% display results
RatedRotorSpeed = 12.1;
fprintf('Change in rotor speed standard deviation:  %4.1f %%\n',...
    (mean(Std_Omega_r_FBFF)/...
     mean(mean(abs(Std_Omega_r_FB)))-1)*100)