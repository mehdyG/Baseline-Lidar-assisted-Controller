# Baseline-Lidar-assisted-Controller
A reference baseline lidar-assisted wind turbine controller for the wind energy community. 

The code uses Bladed style interface, which is responsible for exchanging variables between the OpenFAST executable and the external controllers compiled as
Dynamic Link Library(DLL). The functions of each DLL is described below

DISCON.dll:  The master (wrapper) DLL. It calls sub-DLLs by a specific sequence specified in the "DISCON.IN" input file.
LDP.dll:     The LDP module read in LOS measurements from lidar and estimate the rotor effective wind speed which will eventually be written to the avrSWAP array.
FFP.dll:     The FFP module read in the rotor effective wind speed from the avrSWAP array, then filter the signal and shift it in time. It eventually returns the feed-forward pitch time derivative (rate), which is written into the avrSWAP array.     
ROSCO.dll:   The ROSCO. Version 2.4.1 controller, https://github.com/NREL/ROSCO, 2021. by NREL.  The pitch controller is modified to accept the pitch forward pitch signal.

Author: Feng Guo from Flensburg University of Applied Sciences, funded by LIKE -- Lidar Knowledge Europe, grant agreement No. 858358.   
Target: This code aims to provide a reference Lidar-assisted control package for the community. Please cite the following paper if this code is helpful for your research:
See the paper: "Guo, F., Schlipf, D., and Cheng, P. W.: Evaluation of lidar-assisted wind turbine control under various turbulence characteristics, Wind Energ. Sci. Discuss. [preprint], https://doi.org/10.5194/wes-2022-62, in review, 2022. " for more details.   
    
! License: MIT License
! Copyright (c) 2022 Flensburg University of Applied Sciences, WETI