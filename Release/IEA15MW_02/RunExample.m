% LAC Test IEA15MW_02:  IEA 15 MW offshore reference model on UMaine 
% VolturnUS-S semi-submersible floating platform with perfect wind preview 
% from a single point lidar system
% Origin and changes in files: see ChangeLog.txt.
% Purpose:
% Here, we use a perfect wind preview to demonstrate that the collective
% pitch feedforward controller (designed with SLOW) together with a motion
% compensation (MC) is able to reduce significantly the rotor speed 
% variation when OpenFAST is disturbed by an Extreme Operating Gust. 
% Here, all DOFs are enabled. If no MC is applied, the system is instable.
% Result:       
% Change in platform pitch amplitude (max-min) from FB to FBFF:  200.5 %
% Change in platform pitch amplitude (max-min) from FB to FBFFMC:  -63.5 %
% Authors: 		
% David Schlipf, Feng Guo, Frank Lemmer
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI
% and sowento GmbH

%% Setup
clearvars; 
close all; 
clc;
addpath('..\MatlabFunctions')

% Copy the adequate OpenFAST version to the example folder
FASTexeFile     = 'openfast_x64.exe';
FASTmapFile     = 'MAP_x64.dll';
SimulationName  = 'IEA-15-240-RWT-UMaineSemi';
copyfile(['..\OpenFAST\',FASTexeFile],FASTexeFile)
copyfile(['..\OpenFAST\',FASTmapFile],FASTmapFile)

%% Run FB
ManipulateTXTFile('ROSCO_v2d6.IN','1 ! FlagLAC','0 ! FlagLAC');     % disable LAC
dos([FASTexeFile,' ',SimulationName,'.fst']);                       % run OpenFAST
movefile([SimulationName,'.outb'],[SimulationName,'_FB.outb'])      % store results

%% Run FBFF without motion compensation 
ManipulateTXTFile('ROSCO_v2d6.IN','0 ! FlagLAC','1 ! FlagLAC');     % enable LAC
ManipulateTXTFile('LDP_v2.IN',    '1 ! MC_Mode','0 ! MC_Mode');     % disable MC
dos([FASTexeFile,' ',SimulationName,'.fst']);                       % run OpenFAST
movefile([SimulationName,'.outb'],[SimulationName,'_FBFF.outb'])    % store results

%% Run FBFF with motion compensation 
ManipulateTXTFile('LDP_v2.IN',    '0 ! MC_Mode','1 ! MC_Mode');     % enable MC
dos([FASTexeFile,' ',SimulationName,'.fst']);                       % run OpenFAST
movefile([SimulationName,'.outb'],[SimulationName,'_FBFFMC.outb']) 	% store results

%% Clean up
delete(FASTexeFile)
delete(FASTmapFile)

%% Comparison
% read in data
FB              = ReadFASTbinaryIntoStruct([SimulationName,'_FB.outb']);
FBFF            = ReadFASTbinaryIntoStruct([SimulationName,'_FBFF.outb']);
FBFFMC          = ReadFASTbinaryIntoStruct([SimulationName,'_FBFFMC.outb']);

% Plot         
ScreenSize = get(0,'ScreenSize');
figure('Name','Simulation results','position',[.1 .1 .8 .8].*ScreenSize([3,4,3,4]))

MyAxes(1) = subplot(4,1,1);
hold on; grid on; box on
plot(FB.Time,       FB.Wind1VelX);
plot(FBFF.Time,     FBFF.VLOS01LI);
plot(FBFFMC.Time,	FBFFMC.VLOS01LI);
legend('Hub height wind speed','Vlos')
ylabel('[m/s]');
legend('Wind1VelX','VLOS01LI','VLOS01LI with MC')

MyAxes(2) = subplot(4,1,2);
hold on; grid on; box on
plot(FB.Time,       FB.BldPitch1);
plot(FBFF.Time,     FBFF.BldPitch1);
plot(FBFFMC.Time,	FBFFMC.BldPitch1);
ylabel('BldPitch1 [deg]');
legend('feedback only','feedback-feedforward','feedback-feedforward with MC')

MyAxes(3) = subplot(4,1,3);
hold on; grid on; box on
plot(FB.Time,       FB.RotSpeed);
plot(FBFF.Time,     FBFF.RotSpeed);
plot(FBFFMC.Time,	FBFFMC.RotSpeed);
ylabel('RotSpeed [rpm]');

MyAxes(4) = subplot(4,1,4);
hold on; grid on; box on
plot(FB.Time,       FB.PtfmPitch);
plot(FBFF.Time,     FBFF.PtfmPitch);
plot(FBFFMC.Time,	FBFFMC.PtfmPitch);
ylabel('PtfmPitch [deg]');

xlabel('time [s]')
linkaxes(MyAxes,'x');
xlim([10 150])

%% display results
fprintf('Change in platform pitch amplitude (max-min) from FB to FBFF:  %4.1f %%\n',...
    ((max(FBFF.PtfmPitch)   -min(FBFF.PtfmPitch))/...
     (max(FB.PtfmPitch)     -min(FB.PtfmPitch))-1)*100)
fprintf('Change in platform pitch amplitude (max-min) from FB to FBFFMC:  %4.1f %%\n',...
    ((max(FBFFMC.PtfmPitch) -min(FBFFMC.PtfmPitch))/...
     (max(FB.PtfmPitch)     -min(FB.PtfmPitch))-1)*100) 