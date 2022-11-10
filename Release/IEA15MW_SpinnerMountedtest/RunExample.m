% LAC Test IEA15MW +single beam pulsed lidar mounted on hub
% the wind field is assimed to be uniformly sheared in z direction, the
% wind speed does not change with time
% Purpose:
% Here, we use simple example to verify the implementation of
% spinner-mounted lidar
% Result:
% Sinosodal LOS measurement
% Authors:
% Feng Guo
% Copyright (c) 2022 Flensburg University of Applied Sciences, WETI

%% Setup
clearvars;
close all;
clc;
addpath('..\MatlabFunctions')


FASTexeFile         = 'openfast_x64.exe';
FASTmapFile         = 'MAP_x64.dll';
SimulationName      = 'IEA-15-240-RWT-Monopile';
if ~exist('SimulationResults','dir')
    mkdir SimulationResults
end      
%% Processing: run simulations

% Copy the adequate OpenFAST version to the example folder
copyfile(['..\OpenFAST_sml\',FASTexeFile],FASTexeFile)  %OpenFAST_sml the folder with spinner mounted lidar exe
copyfile(['..\OpenFAST_sml\',FASTmapFile],FASTmapFile)

% Simulate 
WindFileRoot        = ['Wind\UniformShearedSteady_URef_18'];
ManipulateTXTFile('IEA-15-240-RWT_InflowFile.dat','MyFilenameRoot',WindFileRoot);

% Run FB    
FASTresultFile      = ['SimulationResults\UniformShearedSteady_URef_18.outb'];
if ~exist(FASTresultFile,'file')    
    dos([FASTexeFile,' ',SimulationName,'.fst']);
    movefile([SimulationName,'.outb'],FASTresultFile)
end
   
% Clean up
delete(FASTexeFile)
delete(FASTmapFile)

%% Postprocessing: evaluate data

SimDATA                  = ReadFASTbinaryIntoStruct(FASTresultFile);

% Plot time results
figure('Name',['LOS speed'])
hold on; grid on; box on
plot(SimDATA.Time,SimDATA.VLOS01LI);
ylabel('vlos [m/s]');
xlabel('time [s]')

figure('Name',['RotSpeed'])
hold on; grid on; box on
plot(SimDATA.Time,SimDATA.RotSpeed);
ylabel('RotSpeed [rpm]');
xlabel('time [s]')


figure('Name',['Rotor Azimuth'])
hold on; grid on; box on
plot(SimDATA.Time,SimDATA.Azimuth);
ylabel('Azimuth Angle [deg]');
xlabel('time [s]')
