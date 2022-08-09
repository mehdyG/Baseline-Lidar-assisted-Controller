
! Name:   The variable types for the baseline lidar data processing (LDP) DLL for lidar assisted feedforward pitch control.
! Author: Feng Guo from Flensburg University of Applied Sciences, funded by LIKE -- Lidar Knowledge Europe, grant agreement No. 858358.   
! Target: This code aims to provide a reference Lidar-assisted control package for the community. Please cite the following paper if this code is helpful for your research:
! Guo, F., Schlipf, D., and Cheng, P. W.: Evaluation of lidar-assisted wind turbine control under various turbulence characteristics, Wind Energ. Sci. Discuss.
! [preprint], https://doi.org/10.5194/wes-2022-62, in review, 2022.    
    
    
! Function: The LDP module read in LOS measurements from lidar and estimate the rotor effective wind speed which will eventually be written to the avrSWAP array.
! See https://doi.org/10.5194/wes-2022-62 for the definition of ¡°rotor effective wind speed".
    
! Reference: The subroutine relys on the legecy Bladed style data interface. See the Bladed manual for more detail.    
! The code is written based on the source code of ROSCO. Version 2.4.1, https://github.com/NREL/ROSCO, 2021. by NREL.

! License: MIT License
! Copyright (c) 2022 Flensburg University of Applied Sciences, WETI
  
    
MODULE LDP_Types
! Define Types
USE, INTRINSIC  :: ISO_C_Binding
IMPLICIT NONE

TYPE, PUBLIC :: LidarVariables

    ! ---------- -nacelle lidar related variables ----------
    INTEGER(4)                          :: MaxBufferStep = 20           ! the maximum buffer steps for store the u_est
    
    INTEGER(4)                          :: MaxBufferStep_REWS = 2000    ! the maximum buffer steps for store the REWS, 25 second for a 80Hz 
    !INTEGER(4)                          :: Lidar_End_index             ! avrSWAP Record 121: Record number for end of Lidar data
    INTEGER(4)                          :: GatesPerBeam                 ! number of gates in each lidar beam (pulsed lidar) received from avrSWAP by OpenFAST lidar module
    REAL(8)                             :: GatesPerBeam_IN              ! number of gates in each lidar beam (pulsed lidar) received from the Discon.in  just need to cross check with GatesPerBeam from the avrSWAP
    
    INTEGER(4)                          :: NumBeam                      ! number of beams measuring at different directions
    INTEGER(4)                          :: NewMeasurement               ! Flag whether it is a successful new measurement
    INTEGER(4)                          :: BeamID                       ! Beam ID of current lidar data
    !INTEGER(4)                          :: NumPitchCurveFF                       ! Beam ID of current lidar data
    
    REAL(8)                             :: Lidar_Roll                   ! Lidar roll angle [deg]. Euler angles
    REAL(8)                             :: Lidar_Pitch                  ! Lidar pitch anlge [deg].
    REAL(8)                             :: Lidar_Yaw                    ! Lidar yaw angle [deg].
    REAL(8)                             :: Lidar_Xd                     ! Lidar velicity in X direction.
    REAL(8)                             :: Lidar_Yd                     ! Lidar velicity in Y direction.
    REAL(8)                             :: Lidar_Zd                     ! Lidar velicity in Z direction.
    
    REAL(8), DIMENSION(:), ALLOCATABLE  :: Lidar_Azimuth                ! Lidar beam azimuth angles
    REAL(8), DIMENSION(:), ALLOCATABLE  :: Lidar_Elevation              ! Lidar beam elevation angles
    REAL(8)                             :: Lidar_RangeGates             ! Lidar beam range gate, single gate is considered in this baseline LDP
    
    REAL(8), DIMENSION(:,:), ALLOCATABLE :: MeaPositions                ! Measurement positions in Cartesian coordinate (Lidar coordinate system)
    REAL(8), DIMENSION(:,:), ALLOCATABLE :: u_est                       ! Estimated u component at each measurement position, the third dimention buffer steps
    REAL(8), DIMENSION(:,:), ALLOCATABLE :: u_est_Time                  ! The leading time of each u estimation, relative to the first measurement gate
    
    REAL(8), DIMENSION(:,:), ALLOCATABLE   :: UnitVector                ! the unit vector of lidar beams
    REAL(8)                             :: Vlos                         ! Line-of-sight wind speeds measurement [m/s]
    REAL(8)                             :: REWS                         ! Rotor effective wind speed,  estimated by lidar for feed-forward control at a time step [m/s]

    REAL(8)                             :: Uref_Ini                     ! Initial reference mean wind speed
    REAL(8)                             :: AveWin_T                     ! The time average window size in second
    REAL(8)                             :: LeadTime_1                   ! Thea leading time of the first measurement plane
    
    INTEGER(4)                          :: FlagMotionComp               ! Flag whether the motion due to nacelle motion needs to be compensated
    
    
    
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

END MODULE LDP_Types