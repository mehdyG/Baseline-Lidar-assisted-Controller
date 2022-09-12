% Example #01:  NREL 5 MW + Perfect wind preview
% Purpose:
% Here, we use a perfect wind preview to demonstrate that the collective
% pitch feedforward controller (designed with SLOW) is able to reduce
% significantely the rotor speed variation when OpenFAST is disturbed by an
% Extreme Operating Gust. 
% Result:       
% Change in rotor over speed:  -95.3 %
% Authors: 		
% David Schlipf, Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

%% Setup
clearvars; 
close all; 
clc;
addpath('MatlabFunctions')

% Copy the adequate OpenFAST version to the example folder
FASTexeFile     = 'openfast_x64.exe';
FASTmapFile     = 'MAP_x64.dll';
SimulationName  = 'NREL-5.0-126-RWT';
copyfile(['..\OpenFAST\',FASTexeFile],FASTexeFile)
copyfile(['..\OpenFAST\',FASTmapFile],FASTmapFile)

%% Run FB
ManipulateTXTFile('ROSCO2.IN','1                   ! FlagLAC',...
                              '0                   ! FlagLAC');                          
dos([FASTexeFile,' ',SimulationName,'.fst']);
[FB_Data, ~, ~, ~, ~]               = ReadFASTbinary([SimulationName,'.outb']);

%% Run FBFF  
ManipulateTXTFile('ROSCO2.IN','0                   ! FlagLAC',...
                              '1                   ! FlagLAC');                          
dos([FASTexeFile,' ',SimulationName,'.fst']);
[FBFF_Data, ChannelName, ~, ~, ~] 	= ReadFASTbinary([SimulationName,'.outb']);

%% Clean up
delete(FASTexeFile)
delete(FASTmapFile)

%% Comparison
% FB Data
Time_FB         = FB_Data  (:,strcmp(ChannelName,'Time'));
Wind1VelX_FB    = FB_Data  (:,strcmp(ChannelName,'Wind1VelX'));  
BldPitch1_FB    = FB_Data  (:,strcmp(ChannelName,'BldPitch1')); 
GenTq_FB        = FB_Data  (:,strcmp(ChannelName,'GenTq'));
RotSpeed_FB     = FB_Data  (:,strcmp(ChannelName,'RotSpeed'));
TwrBsMyt_FB     = FB_Data  (:,strcmp(ChannelName,'TwrBsMyt')); 

% FBFF Data
Time_FBFF     	= FBFF_Data(:,strcmp(ChannelName,'Time'));
Wind1VelX_FBFF	= FBFF_Data(:,strcmp(ChannelName,'Wind1VelX'));  
BldPitch1_FBFF	= FBFF_Data(:,strcmp(ChannelName,'BldPitch1')); 
GenTq_FBFF     	= FBFF_Data(:,strcmp(ChannelName,'GenTq'));
RotSpeed_FBFF 	= FBFF_Data(:,strcmp(ChannelName,'RotSpeed'));
TwrBsMyt_FBFF 	= FBFF_Data(:,strcmp(ChannelName,'TwrBsMyt')); 

% Plot         
ScreenSize = get(0,'ScreenSize');
figure('Name','Simulation results','position',[.1 .1 .8 .8].*ScreenSize([3,4,3,4]))

MyAxes(1) = subplot(5,1,1);
hold on; grid on; box on
plot(Time_FB,  Wind1VelX_FB);
plot(Time_FBFF,Wind1VelX_FBFF);
ylabel('Wind1VelX [m/s]');

MyAxes(2) = subplot(5,1,2);
hold on; grid on; box on
plot(Time_FB,  BldPitch1_FB);
plot(Time_FBFF,BldPitch1_FBFF);
ylabel('BldPitch1 [deg]');

MyAxes(3) = subplot(5,1,3);
hold on; grid on; box on
plot(Time_FB,  GenTq_FB);
plot(Time_FBFF,GenTq_FBFF);
ylabel('GenTq [kNm]');

MyAxes(4) = subplot(5,1,4);
hold on; grid on; box on
plot(Time_FB,  RotSpeed_FB);
plot(Time_FBFF,RotSpeed_FBFF);
ylabel('RotSpeed [rpm]');

MyAxes(5) = subplot(5,1,5);
hold on; grid on; box on
plot(Time_FB,  TwrBsMyt_FB);
plot(Time_FBFF,TwrBsMyt_FBFF);
ylabel('TwrBsMyt [kNm]');

legend('feedback only','feedback-feedforward')
xlabel('time [s]')

linkaxes(MyAxes,'x');

% display results
RatedRotorSpeed = 12.1;
fprintf('Change in rotor over speed:  %4.1f %%\n',...
    (max(abs(RotSpeed_FBFF-RatedRotorSpeed))/...
     max(abs(RotSpeed_FB  -RatedRotorSpeed))-1)*100)