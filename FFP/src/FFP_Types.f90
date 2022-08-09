! Name:   The variable types for baseline Feed-forward pitch (FFP) DLL for lidar assisted feedforward pitch control.
! Author: Feng Guo from Flensburg University of Applied Sciences, funded by LIKE -- Lidar Knowledge Europe, grant agreement No. 858358.   
! Target: This code aims to provide a reference Lidar-assisted control package for the community. Please cite the following paper if this code is helpful for your research:
! Guo, F., Schlipf, D., and Cheng, P. W.: Evaluation of lidar-assisted wind turbine control under various turbulence characteristics, Wind Energ. Sci. Discuss.
! [preprint], https://doi.org/10.5194/wes-2022-62, in review, 2022.    
    
    
! Function: The FFP module read in the rotor effective wind speed from the avrSWAP array, then filter the signal and shift it in time.
! It eventually returns the feed-forward pitch time derivative (rate), which is written into the avrSWAP array.     
! See https://doi.org/10.5194/wes-2022-62 for the definition of ¡°rotor effective wind speed".
    
! Reference: The subroutine relys on the legecy Bladed style data interface. See the Bladed manual for more detail.    
! The code is written based on the source code of ROSCO. Version 2.4.1, https://github.com/NREL/ROSCO, 2021. by NREL.

! License: MIT License
! Copyright (c) 2022 Flensburg University of Applied Sciences, WETI
    
MODULE FFP_Types
! Define Types
USE, INTRINSIC  :: ISO_C_Binding
IMPLICIT NONE

TYPE, PUBLIC :: LidarVariables
! ---------- -nacelle lidar related variables ----------
!    
    
    INTEGER(4)                          :: MaxBufferStep_REWS = 2000    ! the maximum buffer steps for store the REWS, 25 second for a 80Hz 
    REAL(8)                             :: GatesPerBeam_IN              ! number of gates in each lidar beam (pulsed lidar) received from the Discon.in  just need to cross check with GatesPerBeam from the avrSWAP
    
    INTEGER(4)                          :: NumPitchCurveFF              ! Number of points in the steady state pitch curve
    REAL(8)                             :: REWS                         ! Rotor effective wind speed,  estimated by lidar for feed-forward control at a time step [m/s]
    REAL(8), DIMENSION(:), ALLOCATABLE  :: REWS_f                       ! Low pass filtered rotor effective wind speed,  estimated by lidar for feed-forward control, with a buffer size [m/s]    
    REAL(8)                             :: REWS_b                       ! filterd and buffered Rotor effective wind speed [m/s]
    REAL(8), DIMENSION(:), ALLOCATABLE  :: REWS_f_Time                  ! Rotor effective wind speed,  buffer time[s]    
    REAL(8)                             :: Uref_Ini                     ! Initial reference mean wind speed
    REAL(8)                             :: LeadTime_1                   ! Thea leading time of the first measurement plane
    REAL(8)                             :: F_LPFCornerFreq              ! Corner frequency (-3dB point) in the low-pass filters, [rad/s]
    REAL(8)                             :: Pitch_ActTime                ! The pitch act time in second
    INTEGER(4)                          :: LPF_inst                     ! Instance of LPF filter

    REAL(8), DIMENSION(:), ALLOCATABLE  :: REWS_curve                   ! The second element of unit vector 
    REAL(8), DIMENSION(:), ALLOCATABLE  :: Pitch_curve                  ! The third element of unit vector 
    REAL(8)                             :: FF_Pitch                     !the pitch of feed forwad control
    REAL(8)                             :: FF_Pitch_old                 !the previous pitch of feed forwad control
    REAL(8)                             :: FF_PitchRate                 !the pitch rate of feed forwad control

    
    
    ! ---------- From avrSWAP ----------
    INTEGER(4)                          :: iStatus
    REAL(8)                             :: Time
    REAL(8)                             :: DT
    INTEGER(4)                          :: AvrIndex_REWS
    INTEGER(4)                          :: AvrIndex_REWS_b
    INTEGER(4)                          :: AvrIndex_REWS_f
    INTEGER(4)                          :: AvrIndex_FFrate
    INTEGER(4)                          :: AvrIndex_REWS_leadtime
    
END TYPE LidarVariables    

TYPE, PUBLIC :: LidarErrorVariables
    ! Error Catching
    INTEGER(4)                      :: size_avcMSG
    INTEGER(C_INT)                  :: aviFAIL             ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
    CHARACTER(:), ALLOCATABLE       :: ErrMsg              ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]
END TYPE LidarErrorVariables

END MODULE FFP_Types