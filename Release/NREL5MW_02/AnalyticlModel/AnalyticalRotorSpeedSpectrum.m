function AnalyticalModel = AnalyticalRotorSpeedSpectrum(v_0_OP,theta_OP,Omega_OP,ROSCOInFileName,RotorPerformanceFile,SpectralModelFileName)

% Baseline Spectrum for rotor effective wind speed (avege u components in the swept area)
URef                    = v_0_OP;
T                       = 600;
dt                      = 1/4;
[f,S_RR]              	= CalculateSpectraRotor(T,dt,URef);
S_RR                    = S_RR';
% DS: should be removed and f, S_RR, S_RL, and S_LL loaded from a file.

% Get some parameters from Rosco
Parameter.General.rho           = GetParametersFromROSCO(ROSCOInFileName,'WE_RhoAir');       % [kg/m^3]  air density
[theta,lambda,c_P,c_T]          = GetPowerAndThrustCoefficients(RotorPerformanceFile);
Parameter.Turbine.SS            = struct('theta',theta,'lambda',lambda,'c_P',c_P,'c_T',c_T); % Cp Ct
Parameter.Turbine.i_GB          = 1/GetParametersFromROSCO(ROSCOInFileName,'WE_GearboxRatio');% [-]  gearbox ratio
Parameter.Turbine.R             = GetParametersFromROSCO(ROSCOInFileName,'WE_BladeRadius');  % [m]   Rotor radius
Parameter.Generator.eta_el      = GetParametersFromROSCO(ROSCOInFileName,'VS_GenEff');       % [-]   Generator efficiency
Parameter.Turbine.J             = GetParametersFromROSCO(ROSCOInFileName,'WE_Jtot');         % [kg m^2] Total drivetrain inertia, including blades, hub and casted generator inertia to LSS, 
Parameter.Generator.P_a_rated   = GetParametersFromROSCO(ROSCOInFileName,'VS_RtPwr')/Parameter.Generator.eta_el;   % [W] Rated aerodynamic power

% Pitch control parameters from Rosco
PC_GS_angles            = GetParametersFromROSCO(ROSCOInFileName,'PC_GS_angles');  % Gain-schedule table: pitch angles [rad].
PC_GS_KP                = GetParametersFromROSCO(ROSCOInFileName,'PC_GS_KP'); % Gain-schedule table: pitch controller kp gains [s].
PC_GS_KI                = GetParametersFromROSCO(ROSCOInFileName,'PC_GS_KI'); % Gain-schedule table: pitch controller ki gains [-].
kp                      = -interp1(PC_GS_angles,PC_GS_KP,theta_OP);
KI                      = -interp1(PC_GS_angles,PC_GS_KI,theta_OP);
Ti                      = kp/KI;

% wind turbine
[A,B,C,D]               = LinearizeTurbine1DOF(theta_OP,Omega_OP,v_0_OP,Parameter);
WT_1DOF                 = ss(A,B,C,D);
WT_1DOF.InputName       = {'theta','M_g','v_0'};
WT_1DOF.OutputName      = {'Omega_g','Omega_r'};   

% feedback pitch controller
FB                      = ss(0,1,kp/Ti,kp); 
FB.InputName            = {'Omega_g_f'};    
FB.OutputName           = {'theta_FB'}; 
FB.StateName            = {'IntegratorPI'};

% torque controller
TC                      = ss(-Parameter.Generator.P_a_rated/(Omega_OP/Parameter.Turbine.i_GB)^2); 
TC.InputName            = {'Omega_g_f'};    
TC.OutputName           = {'M_g'};

% Low Pass filter for generator torque control
w_cutoff                = GetParametersFromROSCO(ROSCOInFileName,'F_LPFCornerFreq');
LP                      = ss(-w_cutoff,w_cutoff,1,0); 
LP.InputName            = {'Omega_g'};
LP.OutputName           = {'Omega_g_f'};
LP.StateName            = {'Omega_g_f_dot'};

% pitch actuator
omega                   = GetParametersFromROSCO(ROSCOInFileName,'PA_CornerFreq');
xi                      = GetParametersFromROSCO(ROSCOInFileName,'PA_Damping');
PA                      = ss([0 1 ;-omega^2 -2*omega*xi],[0;omega^2],[1 0],0);
PA.InputName            = {'theta_c'}; 
PA.OutputName           = {'theta'}; 
PA.StateName            = {'theta';'theta_dot'};

% closed loop for FB only control
Sum_FB                  = sumblk('theta_c = theta_FB');
CL_FB                   = connect(WT_1DOF,PA,FB,LP,TC,Sum_FB,{'v_0'},{'Omega_g','Omega_r'});

% Define first Order LPF
load(SpectralModelFileName,'S_RL','S_LL');
G_RL                    = abs(S_RL)./S_LL;
w_cutoff                = interp1(G_RL,f,db2mag(-3),'linear')*2*pi;
LP_FF                   = ss(-w_cutoff,w_cutoff,1,0);                      
LP_FF.InputName         = {'v_0L'};
LP_FF.OutputName        = {'v_0Lf'};  
[~,phase_LP_FF,~]      	= bode(LP_FF,f*2*pi);
TimeDelay               = squeeze(-phase_LP_FF)./360./f';

% Get a time delay for the interested frequency 0.08 is the frequency where rotor speed fluctuates the most
T_filter                = interp1(f,TimeDelay,0.08,'linear');               % DS: Can we make this also adjustable?
T_lead                  = 120/URef;                                         % DS: should be extracted from the lidar file or adjustable
[~,phase_PA,~]          = bode(PA,f*2*pi);
T_pitch_over_f          = squeeze(-phase_PA)./360./f';    
T_pitch                 = interp1(f,T_pitch_over_f,0.08,'linear');          
T_scan                  = 0.5;  %half lidar full scan time                  % DS: should be extracted from the lidar file or adjustable
T_buffer                = T_lead-T_scan-T_pitch-T_filter;

% Define normal feedforward collective pitch controller
FF                      = tf(-B(3)/B(1));    %dtheta/dv0
FF.InputName            = {'v_0Lf'};
FF.OutputName           = {'theta_FF'};  

% Closed Loop FB + normal feedforward with a low-pass filter
Sum_FBFF                = sumblk('theta_c = theta_FB+theta_FF');
CL_FBFF                 = connect(WT_1DOF,PA,FB,FF,LP,LP_FF,TC,Sum_FBFF,{'v_0','v_0L'},{'Omega_g','Omega_r'});

% frequecy domain response of FB only 
[MAG,~]                 = bode(CL_FB,2*pi*f); 

% cross spectrum between the rotor and the buffered lidar estimate
S_RL_buffered           = S_RL.*exp(-1i*2*pi.*f'*T_buffer); 

% frequency domain response of FBFF control
% calculate analytic spectra of rotor speed with FB+FF control
G_OmegaR_FBFF          = squeeze(freqresp(CL_FBFF(2,1),   2*pi*f));                         % tf from Rotor to Omega
G_OmegaL_FBFF          = squeeze(freqresp(CL_FBFF(2,2),   2*pi*f));                         % tf from Lidar to Omega
S_Omega_r_FF           =  G_OmegaR_FBFF.*conj(G_OmegaR_FBFF).*S_RR                  +...    % equal to abs(G_OmegaR_FBLFF.^2).*S_RR;     
                          G_OmegaR_FBFF.*conj(G_OmegaL_FBFF).*S_RL_buffered         +...
                          G_OmegaL_FBFF.*conj(G_OmegaR_FBFF).*conj(S_RL_buffered)   +...
                          G_OmegaL_FBFF.*conj(G_OmegaL_FBFF).*S_LL;                         % equal to abs(G_OmegaL_FBLFF.^2).*S_LL;                         

% Package and send out
AnalyticalModel.S_Omega_r_FB            = squeeze(MAG(2,:,:)).^2.*S_RR;
AnalyticalModel.S_Omega_r_FF            = S_Omega_r_FF;
AnalyticalModel.f                       = f;

end

