! Name:   The subroutines for the baseline lidar data processing (LDP) DLL for lidar assisted feedforward pitch control.
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
  
! -------------------------------------------------------------------------------------------

! This module contains the subroutines to read in parameters, variables and to process lidar data



MODULE LDP_Subs
!...............................................................................................................................
    !USE Constants   
    USE :: LDP_Types
    
    IMPLICIT NONE

    ! Global Variables
    LOGICAL, PARAMETER          :: DEBUG_PARSING = .FALSE.      ! debug flag to output parsing information, set up Echo file later
    CHARACTER(*),  PARAMETER    :: NewLine     = ACHAR(10)      ! The delimiter for New Lines [ Windows is CHAR(13)//CHAR(10); MAC is CHAR(13); Unix is CHAR(10) {CHAR(13)=\r is a line feed, CHAR(10)=\n is a new line}]
    
    INTERFACE ParseInput                                        ! Parses a character variable name and value from a string.
        MODULE PROCEDURE ParseInput_Str                         ! Parses a character string from a string.
        MODULE PROCEDURE ParseInput_Dbl                         ! Parses a double-precision REAL from a string.
        MODULE PROCEDURE ParseInput_Int                         ! Parses an INTEGER from a string.

    END INTERFACE
    
    INTERFACE ParseAry                                          ! Parse an array of numbers from a string.
        MODULE PROCEDURE ParseDbAry                             ! Parse an array of double-precision REAL values.
        MODULE PROCEDURE ParseInAry                             ! Parse an array of whole numbers.
    END INTERFACE


CONTAINS
    
    ! Read avrSWAP array into the local Lidar Variables
    SUBROUTINE ReadAvrSWAP(avrSWAP, LidarVar)
        USE :: LDP_Types, ONLY : LidarVariables

        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)     ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(LidarVariables), INTENT(INOUT) :: LidarVar
        INTEGER(4)                      :: L           ! Index number in the avrSWAP array for the start of Lidar data
        INTEGER(4)                      :: I           ! index
        
        
        ! Load variables from calling program (See Appendix A of Bladed User's Guide):
        LidarVar%iStatus        = NINT(avrSWAP(1))   ! Initialization status  
        LidarVar%Time           = avrSWAP(2)         ! Current simulation time
        LidarVar%DT             = avrSWAP(3)         ! Time step
        
        
        ! --- read and set the lidar variables
        L                       = NINT(avrSWAP(63))  ! The index in the array where the lidar related data begins 
        
        !> NewData
        LidarVar%NewMeasurement = NINT(avrSWAP(L))   ! Flag whether the current measurment is a new one
       
       !> BeamID
       	LidarVar%BEAMID         = NINT(avrSWAP(L+1)) ! Lidar the beam number of the current lidar measurement
       
       !> Gates per beam
        LidarVar%GatesPerBeam_IN   = avrSWAP(L+ 2)   ! Number of range gate, for this reference version, only one range gate is supported
                                                             
       !> Lidar line-of-sight speed measurement
       LidarVar%VLOS = avrSWAP( L + 2 + 1 )  
       
       !> Lidar roll, pitch and yaw tilt angular (rotational) displacement (deg) and velocities
       LIDARVAR%Lidar_Roll  =  avrSWAP(L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 1)
       LIDARVAR%Lidar_Pitch =  avrSWAP(L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 2) 
       LIDARVAR%Lidar_Yaw   =  avrSWAP(L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 3)
       LIDARVAR%Lidar_xd    =  avrSWAP(L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 4)
       LIDARVAR%Lidar_yd    =  avrSWAP(L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 5)
       LIDARVAR%Lidar_zd    =  avrSWAP(L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 6)
       
       
       LIDARVAR%AvrIndex_REWS   = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 7           ! Lotor effective wind speed
       LIDARVAR%AvrIndex_REWS_b = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 8           ! Filtered and buffered (time shifted) rotor effective wind speed 
       LIDARVAR%AvrIndex_FFrate = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 9           ! Forward pitch rate
       LIDARVAR%AvrIndex_REWS_f = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 10          ! Filtered rotor effective wind speed
       LIDARVAR%AvrIndex_REWS_leadtime   = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 11 ! Leading time of the first measurement range gate

    END SUBROUTINE ReadAvrSWAP    
    
    !-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE WindFieldReconstruction(LidarVar,ErrVar)
    ! Rescontruct rotor effection wind speed based on lidar line-of-sight measurement
    
        USE LDP_Types, ONLY : LidarVariables,LidarErrorVariables
        
        TYPE(LidarVariables), INTENT(INOUT)          :: LidarVar
        TYPE(LidarErrorVariables), INTENT(INOUT)     :: ErrVar
        INTEGER(4)                                   :: iBeam                ! The index for lidar beam
        INTEGER(4)                                   :: iBuffer              ! The index for buffer
        INTEGER(4)                                   :: Counter = 0          ! Counter to determine how many LOS will be used for REWS estimation
        REAL(8)                                      :: REWS_temporal=0      ! Temporay REWS for summation
        INTEGER(4)                                   :: BuffReturnLevel      ! Flag whether we still need to loop over buffer to find suitable data for a measurement position
        REAL(8)                                      :: LeadTime_1           ! Leading time of the first measurement plane
        
        REAL(8)                                      :: UnitVectorinInertial(3)  ! 
        
        
        ! Get unitvector in inertial coordinate system
        UnitVectorinInertial  = LidarCoordinateToInertial(LidarVar%UnitVector(LidarVar%BeamID+1,1),LidarVar%UnitVector(LidarVar%BeamID+1,2), LidarVar%UnitVector(LidarVar%BeamID+1,3),LidarVar%Lidar_Roll,LidarVar%Lidar_Pitch,LidarVar%Lidar_Yaw)     
        
       
        !---------first in last out, shift the u_est and Time in the buffer 
        DO iBuffer = LidarVar%MaxBufferStep,2,-1
           LidarVar%u_est(:,iBuffer) = LidarVar%u_est(:,iBuffer-1)
           LidarVar%u_est_Time(:,iBuffer) = LidarVar%u_est_Time(:,iBuffer-1)
        END DO   
        

        !---------now calculate the estimated u component and assign the time to the first buffer
        IF  (LidarVar%Vlos>0 .AND. LidarVar%Vlos<50) THEN ! the the returned LOS is in reasonable range then we estimate the u 
        LidarVar%u_est(LidarVar%BeamID+1,1) = -LidarVar%Vlos/UnitVectorinInertial(1)
        ELSE
        LidarVar%u_est(LidarVar%BeamID+1,1) = 0  ! 0 means the measurement is not valid
        END IF 
               
        ! Important: the turbine and nacelle lidar are assumed looking to the negative x direction,
        ! So wind propogates by the positive x direction, the max is added to avoid dividing by zero
        ! The rotor is assumed to be at x = 0
        LidarVar%u_est_Time(LidarVar%BeamID+1,1) = LidarVar%Time+LidarVar%MeaPositions(LidarVar%BeamID+1,1)/MAX(LidarVar%Uref_Ini,0.1) 
        LeadTime_1                               = LidarVar%MeaPositions(1,1)/MAX(LidarVar%Uref_Ini,0.1) 
        
        !--------loop over the u_est buffer to calculate the rotor effective wind speed using the simplest method
        ! more advance method, e.g. with motion compensation should be developed further
        Counter            = 0
        REWS_temporal      = 0
        iBuffer            = 1
        BuffReturnLevel    = 0
        DO iBeam  = 1, LidarVar%NumBeam
                BuffReturnLevel = 0 ! for each beam and gate the return level is initilized to be 0
                DO iBuffer = 1,LidarVar%MaxBufferStep  ! we go from eariler stored data
                    IF (LidarVar%u_est(iBeam,iBuffer)>0.AND.&
                        LidarVar%u_est_Time(iBeam,iBuffer)<=(LeadTime_1+LidarVar%Time).AND.&
                        LidarVar%u_est_Time(iBeam,iBuffer)>(LeadTime_1+LidarVar%Time-LidarVar%AveWin_T).AND.BuffReturnLevel==0) THEN
                        ! first logical determine the current position is a valid measurement
                        ! second logical determine the time is within the averaging time window
                        ! third logical determine we have not find a value in the buffer
                         REWS_temporal = REWS_temporal+LidarVar%u_est(iBeam,iBuffer)
                         Counter      =  Counter+1
                         BuffReturnLevel = 1  ! one we have found a value in the buffer we should not enter the buffer loop anymore
                                              ! because we should not have more than one valid measurements from one position
                    END IF
                END DO
        END DO  
        
       
        ! now get the averaged u_est
        IF (Counter /=0) THEN
        LidarVar%REWS      = REWS_temporal/Counter
        ELSE
        LidarVar%REWS      = LidarVar%Uref_Ini  ! keep th mean wind if no successful measurement is made
        END IF
        
        LidarVar%LeadTime_1 = LeadTime_1

        
        
    END SUBROUTINE WindFieldReconstruction
    
   
    
 !---------------------------------------------------------------------   
    SUBROUTINE LidarSim_InitMeasuringPoints_Spherical(LidarVar, ErrVar)
	
    USE LDP_Types, ONLY : LidarErrorVariables, LidarVariables
        
    IMPLICIT                            NONE
    CHARACTER(*),                       PARAMETER       ::  RoutineName="LidarSim_InitMeasuringPoints_Spherical"

    !TYPE(LidarSim_ParameterType),       INTENT(INOUT)   ::  p                       !parameter data (destination of the InputFileData)
    TYPE(LidarErrorVariables),  INTENT(INOUT)   :: ErrVar
    TYPE(LidarVariables),       INTENT(INOUT)   :: LidarVar
    
    ! Local variables
    INTEGER(4)                                      ::  iBeam        !counter for looping through the coordinate data
    !INTEGER(4)                                      ::  iRange        !counter for looping through the multiple range gates

     IF (.not. allocated(LidarVar%MeaPositions)) THEN
        Allocate(LidarVar%MeaPositions(LIDARVAR%NUMBEAM,3))
     END IF
     
     IF (.not. allocated(LidarVar%UnitVector)) THEN
        Allocate(LidarVar%UnitVector(LIDARVAR%NUMBEAM,3))
     END IF
    
 
    !  loop over to calculate measurement positions and unit vectors  
    DO iBeam=1,LIDARVAR%NUMBEAM
        !DO iRange=1, LIDARVAR%GATESPERBEAM
                IF ( LIDARVAR%LIDAR_RANGEGATES /= 0 )THEN  !Range gates mustn't be 0. => Divide by 0 !
                    LidarVar%MeaPositions(iBeam,:) = &   !Transformation from the spherical to cartesian coordinates
                        LidarSim_Spherical2Cartesian(LIDARVAR%LIDAR_AZIMUTH(iBeam),LIDARVAR%LIDAR_ELEVATION(iBeam),	&	LIDARVAR%LIDAR_RANGEGATES)
                ELSE
                    		ERRVAR%ERRMSG = "Lidar Range gates must not be 0, check the .IN file!"
                            ERRVAR%AVIFAIL= -1
                RETURN
                ENDIF
            
        !END DO
        
        ! get the unit vector, not dependent on the range
        LidarVar%UnitVector(iBeam,1) = COSD(LIDARVAR%LIDAR_ELEVATION(iBeam))*COSD(LIDARVAR%LIDAR_AZIMUTH(iBeam))
        LidarVar%UnitVector(iBeam,2) = COSD(LIDARVAR%LIDAR_ELEVATION(iBeam))*SIND(LIDARVAR%LIDAR_AZIMUTH(iBeam))
        LidarVar%UnitVector(iBeam,3) = SIND(LIDARVAR%LIDAR_ELEVATION(iBeam))
            
    END DO
     
    END SUBROUTINE LidarSim_InitMeasuringPoints_Spherical
 
    FUNCTION LidarSim_Spherical2Cartesian(Azimuth, Elevation, Range)
	 
    IMPLICIT        NONE
	 CHARACTER(*),   PARAMETER       ::  RoutineName="LidarSim_Spherical2Cartesian"
	 
    REAL(8),     INTENT(IN   )   ::  Azimuth                             !Azimuth angle
    REAL(8),     INTENT(IN   )   ::  Elevation                           !Elevation angle
    REAL(8),     INTENT(IN   )   ::  Range                               !range gate
    REAL(8),     DIMENSION (3)   ::  LidarSim_Spherical2Cartesian        !Output : x,y,z coordinates
    
    LidarSim_Spherical2Cartesian(1)  =   Range*COSD(Elevation)*COSD(Azimuth)   !x
    LidarSim_Spherical2Cartesian(2)  =   Range*COSD(Elevation)*SIND(Azimuth)   !y
    LidarSim_Spherical2Cartesian(3)  =   Range*SIND(Elevation)                !z
	 
    END FUNCTION LidarSim_Spherical2Cartesian
    
        !-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION LidarCoordinateToInertial(x_n_L,y_n_L,z_n_L,Roll,Pitch,Yaw)
    ! convert the original unit vectors, e.g. the lidar in initial position, to that in the inertial system (if necelle motion is considered, the lidar coordinate is changing dynamically)  
    
        REAL(8), INTENT(IN)         :: Roll                     ! Euler roll in local lidar coordinate [deg]
        REAL(8), INTENT(IN)         :: Pitch                    ! Euler Pitch in local lidar coordinate [deg]
        REAL(8), INTENT(IN)         :: Yaw                      ! Euler Yaw in local lidar coordinate   [deg]
        REAL(8), INTENT(IN)         :: x_n_L                    ! The first element of unit vector in lidar coordinate [-]
        REAL(8), INTENT(IN)         :: y_n_L                    ! The second element of unit vector in lidar coordinate [-]
        REAL(8), INTENT(IN)         :: z_n_L                    ! The third element of unit vector in lidar coordinate [-]
        REAL(8)                     :: LidarCoordinateToInertial(3)                       ! The transformed unit vector
        REAL(8)                     :: T_Yaw(3,3)              ! The transformation matrix for yaw
        REAL(8)                     :: T_Pitch(3,3)             ! The transformation matrix for pitch
        REAL(8)                     :: T_Roll(3,3)              ! The transformation matrix for roll
        REAL(8)                     :: T(3,3)                   ! The overall transformation matrix, we stick to first yaw, then pitch and roll in the end, note the sequence matters!
        REAL(8), PARAMETER          :: PI = 3.14159265359       ! Mathematical constant pi
        REAL(8), PARAMETER          :: zero = 0.0       
        REAL(8), PARAMETER          :: one = 1.0       
        
        
        !! Yaw is a rotation around z-axis    
        T_Yaw(1,:)  = (/cos(Yaw),-sin(Yaw),zero/)
        T_Yaw(2,:)  = (/sin(Yaw),cos(Yaw),zero/)
        T_Yaw(3,:)  = (/zero,zero,one/)

        
        ! Pitch is a rotation around z-axis    
        T_Pitch(1,:)  = (/cos(Pitch),zero,sin(Pitch)/)
        T_Pitch(2,:)  = (/zero,one,zero/)
        T_Pitch(3,:)  = (/-sin(Pitch),zero,cos(Pitch)/)
        
        ! Roll is a rotation around x-axis
        T_Roll(1,:)  = (/one,zero,zero/)
        T_Roll(2,:)  = (/zero,cos(Roll),-sin(Roll)/)
        T_Roll(3,:)  = (/zero,sin(Roll),cos(Roll)/)
        
        T            = matmul(matmul(T_Yaw,T_Pitch),T_Roll)   !zyx
        
        LidarCoordinateToInertial(1)          = T(1,1)*x_n_L + T(1,2)*y_n_L + T(1,3)*z_n_L;
        LidarCoordinateToInertial(2)          = T(2,1)*x_n_L + T(2,2)*y_n_L + T(2,3)*z_n_L;
        LidarCoordinateToInertial(3)          = T(3,1)*x_n_L + T(3,2)*y_n_L + T(3,3)*z_n_L;
        

    END FUNCTION LidarCoordinateToInertial

    
    
    
 ! ----------------------------------------------------------------------------------- 
! -----------------------------------------------------------------------------------
    ! Get the sub DLL informations 
    SUBROUTINE SetLidarParameters(avrSWAP, accINFILE, size_avcMSG, LidarVar, ErrVar)
        USE LDP_Types, ONLY : LidarErrorVariables, LidarVariables

        
        REAL(C_FLOAT),              INTENT(INOUT)    :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        CHARACTER(C_CHAR),          INTENT(IN   )    :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file

        INTEGER(4),                 INTENT(IN   )   :: size_avcMSG
        TYPE(LidarErrorVariables),  INTENT(INOUT)   :: ErrVar
        TYPE(LidarVariables),       INTENT(INOUT)   :: LidarVar

        
        INTEGER(4)                              :: K    ! Index used for looping through blades.
        !INTEGER(4)                              :: iStatus ! the status of the DLL chain calling, 0 means first call
        
        CHARACTER(*),               PARAMETER       :: RoutineName = 'SetLidarParameters'

        
        !LidarVar%iStatus            = NINT(avrSWAP(1))

        ! Set ErrVar%aviFAIL to 0 in each iteration:
        ErrVar%aviFAIL = 0
        ! ALLOCATE(ErrVar%ErrMsg(size_avcMSG-1))
        ErrVar%size_avcMSG  = size_avcMSG
        !LidarVar%LPF_inst   = 1           ! instance to determine how many LPF we use
        
        ! Read the DLL Parameters specified in the User Interface
        !   and initialize variables:
        IF (LidarVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            ! Description:
            print *, '--------------------------------------------------------------------'
            print *, 'A baseline lidar data processing algorithm for pitch forward control'
            print *, 'Developed by Flensburg University of Applied Sciences, Germany'
            print *, 'Author: Feng Guo, David Schlipf'
            print *, '--------------------------------------------------------------------'
            
            
            CALL ReadLidarParameterFileSub(LidarVar,accINFILE, NINT(avrSWAP(50)),ErrVar)
            
            LidarVar%REWS      = LidarVar%Uref_Ini 
            
            
            IF (NINT(LIDARVAR%GATESPERBEAM_IN) /= LIDARVAR%GATESPERBEAM) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = 'The gates per beam obtained from the avrSWAP array does not match with the .IN parameter file.'
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
            END IF
            
            ! If there's been an file reading error, don't continue
            ! Add RoutineName to error message
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
                RETURN
            ENDIF



        ENDIF
    END SUBROUTINE SetLidarParameters
    
    
    
       ! Copied from NWTC_IO.f90
!> This function returns a character string encoded with today's date in the form dd-mmm-ccyy.
FUNCTION CurDate( )

    ! Function declaration.

    CHARACTER(11)                :: CurDate                                      !< 'dd-mmm-yyyy' string with the current date


    ! Local declarations.

    CHARACTER(8)                 :: CDate                                        ! String to hold the returned value from the DATE_AND_TIME subroutine call.



    !  Call the system date function.

    CALL DATE_AND_TIME ( CDate )


    !  Parse out the day.

    CurDate(1:3) = CDate(7:8)//'-'


    !  Parse out the month.

    SELECT CASE ( CDate(5:6) )
    CASE ( '01' )
        CurDate(4:6) = 'Jan'
    CASE ( '02' )
        CurDate(4:6) = 'Feb'
    CASE ( '03' )
        CurDate(4:6) = 'Mar'
    CASE ( '04' )
        CurDate(4:6) = 'Apr'
    CASE ( '05' )
        CurDate(4:6) = 'May'
    CASE ( '06' )
        CurDate(4:6) = 'Jun'
    CASE ( '07' )
        CurDate(4:6) = 'Jul'
    CASE ( '08' )
        CurDate(4:6) = 'Aug'
    CASE ( '09' )
        CurDate(4:6) = 'Sep'
    CASE ( '10' )
        CurDate(4:6) = 'Oct'
    CASE ( '11' )
        CurDate(4:6) = 'Nov'
    CASE ( '12' )
        CurDate(4:6) = 'Dec'
    END SELECT


    !  Parse out the year.

    CurDate(7:11) = '-'//CDate(1:4)


    RETURN
    END FUNCTION CurDate

!=======================================================================
!> This function returns a character string encoded with the time in the form "hh:mm:ss".
    FUNCTION CurTime( )

    ! Function declaration.

    CHARACTER(8)                 :: CurTime                                      !< The current time in the form "hh:mm:ss".


    ! Local declarations.

    CHARACTER(10)                :: CTime                                        ! String to hold the returned value from the DATE_AND_TIME subroutine call.



    CALL DATE_AND_TIME ( TIME=CTime )

    CurTime = CTime(1:2)//':'//CTime(3:4)//':'//CTime(5:6)


    RETURN
    END FUNCTION CurTime 
    
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadLidarParameterFileSub(LidarVar,accINFILE, accINFILE_size,ErrVar)!, accINFILE_size)
        
        USE, INTRINSIC :: ISO_C_Binding
        USE LDP_Types, ONLY : LidarErrorVariables,LidarVariables

        INTEGER(4)                                      :: accINFILE_size               ! size of DISCON input filename, INTENT(IN) here??
        CHARACTER(accINFILE_size),  INTENT(IN   )       :: accINFILE(accINFILE_size)    ! DISCON input filename
        !TYPE(ControlParameters),    INTENT(INOUT)       :: CntrPar                      ! Control parameter type
        TYPE(LidarErrorVariables),  INTENT(INOUT)       :: ErrVar
        TYPE(LidarVariables),       INTENT(INOUT)       :: LidarVar
        INTEGER(4),                 PARAMETER           :: UnControllerParameters = 89  ! Unit number to open file
        INTEGER(4)                                      :: CurLine 
        INTEGER(4)                                      :: ppos             ! index to remove file extension
        INTEGER(4)                                      :: stringlength     ! length of a string
        INTEGER(4)                                      :: i                   ! counter  
 
        CHARACTER(*),               PARAMETER           :: RoutineName = 'ReadLidarParameterFileSub'

        CurLine = 1
       

        OPEN(unit=UnControllerParameters, file=accINFILE(1), status='old', action='read')
        
        !----------------------- HEADER ------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !!------- LIDAR TRAJECTORY -----------------------------------------------------
        CALL ParseInput(UnControllerParameters,CurLine,'NumBeam',accINFILE(1),LidarVar%NumBeam,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'GatesPerBeam',accINFILE(1),LidarVar%GatesPerBeam,ErrVar)
        CALL ParseAry(UnControllerParameters,CurLine,'Lidar_Azimuth',LidarVar%Lidar_Azimuth,LidarVar%NumBeam,accINFILE(1),ErrVar)
        CALL ParseAry(UnControllerParameters,CurLine,'Lidar_Elevation',LidarVar%Lidar_Elevation,LidarVar%NumBeam,accINFILE(1),ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Lidar_RangeGates',accINFILE(1),LidarVar%Lidar_RangeGates,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        
       
        !------- ROTOR EFFECTIVE WIND SPEED ESTIMATION ---------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'AveWin_T',accINFILE(1),LidarVar%AveWin_T,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'FlagMotionComp',accINFILE(1),LidarVar%FlagMotionComp,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        
         !!------- INITIAL CONDITION----------------------------------------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'Uref_Ini',accINFILE(1),LidarVar%Uref_Ini,ErrVar)
       
        ! Close Input File
        CLOSE(UnControllerParameters)

        
        IF (.not. allocated(LidarVar%u_est)) THEN
        Allocate(LidarVar%u_est(LIDARVAR%NUMBEAM,LidarVar%MaxBufferStep))
        LidarVar%u_est(:,:) = 0
        END IF
        
        IF (.not. allocated(LidarVar%u_est_Time)) THEN
        Allocate(LidarVar%u_est_Time(LIDARVAR%NUMBEAM,LidarVar%MaxBufferStep))
        LidarVar%u_est_Time(:,:) = 0
        END IF
               
        
        IF ( LIDARVAR%Uref_Ini== 0 )THEN  !Range gates mustn't be 0. => Divide by 0 !
            ERRVAR%ERRMSG = "The initial reference mean wind speed (Uref_Ini) must not be 0, check the .IN file!"
            ERRVAR%AVIFAIL= -1 
        ENDIF
        
        
        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE ReadLidarParameterFileSub
    ! -----------------------------------------------------------------------------------
    
!--------------------------------Below the file I/O related routines copied from ROSCO    
    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    ! the ParseInput is copied from the ROSCO_Types
    subroutine ParseInput_Int(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE LDP_Types, ONLY : LidarErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(4),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(LidarErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(20)                           :: Words       (2)               ! The two "words" parsed from the line

        INTEGER(4),             INTENT(INOUT)   :: Variable   ! Variable
        INTEGER(4)                              :: ErrStatLcl                    ! Error status local to this routine.
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Int

    !=======================================================================
    ! Parse double input, this is a copy of ParseInput_Int and a change in the variable definitions
    subroutine ParseInput_Dbl(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE LDP_Types, ONLY : LidarErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(4),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(LidarErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(20)                           :: Words       (2)               ! The two "words" parsed from the line
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        REAL(8),             INTENT(INOUT)      :: Variable   ! Variable
        INTEGER(4)                              :: ErrStatLcl                    ! Error status local to this routine.

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            IF (DEBUG_PARSING) THEN
            print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),*,IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid INTEGER value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Dbl

    !=======================================================================
    ! Parse string input, this is a copy of ParseInput_Int and a change in the variable definitions
    subroutine ParseInput_Str(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE LDP_Types, ONLY : LidarErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(4),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(LidarErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
        CHARACTER(200)                          :: Words       (2)               ! The two "words" parsed from the line
        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

        CHARACTER(*),           INTENT(INOUT)   :: Variable   ! Variable
        INTEGER(4)                              :: ErrStatLcl                    ! Error status local to this routine.

        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN

            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Separate line string into 2 words
            CALL GetWords ( Line, Words, 2 )  

            ! Debugging: show what's being read, turn into Echo later
            if (DEBUG_PARSING) THEN
                print *, 'Read: '//TRIM(Words(1))//' and '//TRIM(Words(2)),' on line ', CurLine
            END IF

            ! Check that Variable Name is in Words
            IF (CheckName_) THEN
                CALL ChkParseData ( Words, VarName, FileName, CurLine, ErrVar )
            END IF

            ! IF We haven't failed already
            IF (ErrVar%aviFAIL >= 0) THEN        

                ! Read the variable
                READ (Words(1),'(A)',IOSTAT=ErrStatLcl)  Variable
                IF ( ErrStatLcl /= 0 )  THEN
                    ErrVar%aviFAIL  = -1
                    ErrVar%ErrMsg   =  NewLine//' >> A fatal error occurred when parsing data from "' &
                        //TRIM( FileName )//'".'//NewLine//  &
                        ' >> The variable "'//TRIM( Words(2) )//'" was not assigned valid STRING value on line #' &
                        //TRIM( Int2LStr( CurLine ) )//'.'//NewLine//&
                        ' >> The text being parsed was :'//NewLine//'    "'//TRIM( Line )//'"'
                ENDIF

            ENDIF   

            ! Increment line counter
            CurLine = CurLine + 1
        END IF

    END subroutine ParseInput_Str
    
!=======================================================================
!> This subroutine parses the specified line of text for AryLen REAL values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
    SUBROUTINE ParseDbAry ( Un, LineNum, AryName, Ary, AryLen, FileName, ErrVar, CheckName )

        USE LDP_Types, ONLY : LidarErrorVariables

        ! Arguments declarations.
        INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
        INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

        REAL(8), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

        INTEGER(4),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
        CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


        CHARACTER(*),           INTENT(IN   )   :: AryName                       !< The array name we are trying to fill.

        TYPE(LidarErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input

        LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName


        ! Local declarations.

        CHARACTER(1024)                         :: Line
        INTEGER(4)                              :: ErrStatLcl                    ! Error status local to this routine.
        INTEGER(4)                              :: i

        CHARACTER(200), ALLOCATABLE             :: Words_Ary       (:)               ! The array "words" parsed from the line.
        CHARACTER(1024)                         :: Debug_String 
        CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseDbAry'
        LOGICAL                                 :: CheckName_

        ! Figure out if we're checking the name, default to .TRUE.
        CheckName_ = .TRUE.
        if (PRESENT(CheckName)) CheckName_ = CheckName 

        ! If we've already failed, don't read anything
        IF (ErrVar%aviFAIL >= 0) THEN
            ! Read the whole line as a string
            READ(Un, '(A)') Line

            ! Allocate array and handle errors
            ALLOCATE ( Ary(AryLen) , STAT=ErrStatLcl )
            IF ( ErrStatLcl /= 0 ) THEN
                IF ( ALLOCATED(Ary) ) THEN
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( AryName )//' array; array was already allocated.'
                ELSE
                    ErrVar%aviFAIL = -1
                    ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( AryName )//' array.'
                END IF
            END IF
        
            ! Allocate words array
            ALLOCATE ( Words_Ary( AryLen + 1 ) , STAT=ErrStatLcl )
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Fatal error allocating memory for the Words array.'
                CALL Cleanup()
                RETURN
            ENDIF

            ! Separate line string into AryLen + 1 words, should include variable name
            CALL GetWords ( Line, Words_Ary, AryLen + 1 )  

            ! Debug Output
            IF (DEBUG_PARSING) THEN
                Debug_String = ''
                DO i = 1,AryLen+1
                    Debug_String = TRIM(Debug_String)//TRIM(Words_Ary(i))
                    IF (i < AryLen + 1) THEN
                        Debug_String = TRIM(Debug_String)//','
                    END IF
                END DO
                print *, 'Read: '//TRIM(Debug_String)//' on line ', LineNum
            END IF

            ! Check that Variable Name is at the end of Words, will also check length of array
            IF (CheckName_) THEN
                CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), AryName, FileName, LineNum, ErrVar )
            END IF
        
            ! Read array
            READ (Line,*,IOSTAT=ErrStatLcl)  Ary
            IF ( ErrStatLcl /= 0 )  THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                                //TRIM( FileName )//'".'//NewLine//  &
                                ' >> The "'//TRIM( AryName )//'" array was not assigned valid REAL values on line #' &
                                //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                                //'    "'//TRIM( Line )//'"' 
                RETURN
                CALL Cleanup()         
            ENDIF

        !  IF ( PRESENT(UnEc) )  THEN
        !     IF ( UnEc > 0 )  WRITE (UnEc,'(A)')  TRIM( FileInfo%Lines(LineNum) )
        !  END IF

            LineNum = LineNum + 1
            CALL Cleanup()
        ENDIF

        RETURN

        !=======================================================================
        CONTAINS
        !=======================================================================
            SUBROUTINE Cleanup ( )

                ! This subroutine cleans up the parent routine before exiting.

                ! Deallocate the Words array if it had been allocated.

                IF ( ALLOCATED( Words_Ary ) ) DEALLOCATE( Words_Ary )


                RETURN

            END SUBROUTINE Cleanup

  END SUBROUTINE ParseDbAry

  !=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   
  SUBROUTINE ParseInAry ( Un, LineNum, AryName, Ary, AryLen, FileName, ErrVar, CheckName )

    USE LDP_Types, ONLY : LidarErrorVariables

    ! Arguments declarations.
    INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
    INTEGER,                INTENT(IN   )   :: AryLen                        !< The length of the array to parse.

    INTEGER(4), ALLOCATABLE,   INTENT(INOUT)   :: Ary(:)            !< The array to receive the input values.

    INTEGER(4),             INTENT(INOUT)   :: LineNum                       !< The number of the line to parse.
    CHARACTER(*),           INTENT(IN)      :: FileName                      !< The name of the file being parsed.


    CHARACTER(*),           INTENT(IN   )   :: AryName                       !< The array name we are trying to fill.

    TYPE(LidarErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input

    LOGICAL, OPTIONAL,      INTENT(IN   )   :: CheckName

    ! Local declarations.

    CHARACTER(1024)                         :: Line
    INTEGER(4)                              :: ErrStatLcl                    ! Error status local to this routine.
    INTEGER(4)                              :: i

    CHARACTER(200), ALLOCATABLE             :: Words_Ary       (:)               ! The array "words" parsed from the line.
    CHARACTER(1024)                         :: Debug_String 
    CHARACTER(*), PARAMETER                 :: RoutineName = 'ParseInAry'

    LOGICAL                                 :: CheckName_

    ! Figure out if we're checking the name, default to .TRUE.
    CheckName_ = .TRUE.
    if (PRESENT(CheckName)) CheckName_ = CheckName    

    ! If we've already failed, don't read anything
    IF (ErrVar%aviFAIL >= 0) THEN
        ! Read the whole line as a string
        READ(Un, '(A)') Line

        ! Allocate array and handle errors
        ALLOCATE ( Ary(AryLen) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 ) THEN
            IF ( ALLOCATED(Ary) ) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for the '//TRIM( AryName )//' array; array was already allocated.'
            ELSE
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = RoutineName//':Error allocating memory for '//TRIM(Int2LStr( AryLen ))//' characters in the '//TRIM( AryName )//' array.'
            END IF
        END IF
    
        ! Allocate words array
        ALLOCATE ( Words_Ary( AryLen + 1 ) , STAT=ErrStatLcl )
        IF ( ErrStatLcl /= 0 )  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = RoutineName//':Fatal error allocating memory for the Words array.'
            CALL Cleanup()
            RETURN
        ENDIF

        ! Separate line string into AryLen + 1 words, should include variable name
        CALL GetWords ( Line, Words_Ary, AryLen + 1 )  

        ! Debug Output
        IF (DEBUG_PARSING) THEN
            Debug_String = ''
            DO i = 1,AryLen+1
                Debug_String = TRIM(Debug_String)//TRIM(Words_Ary(i))
                IF (i < AryLen + 1) THEN
                    Debug_String = TRIM(Debug_String)//','
                END IF
            END DO
            print *, 'Read: '//TRIM(Debug_String)//' on line ', LineNum
        END IF

        ! Check that Variable Name is at the end of Words, will also check length of array
        IF (CheckName_) THEN
            CALL ChkParseData ( Words_Ary(AryLen:AryLen+1), AryName, FileName, LineNum, ErrVar )
        END IF
    
        ! Read array
        READ (Line,*,IOSTAT=ErrStatLcl)  Ary
        IF ( ErrStatLcl /= 0 )  THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = RoutineName//':A fatal error occurred when parsing data from "' &
                            //TRIM( FileName )//'".'//NewLine//  &
                            ' >> The "'//TRIM( AryName )//'" array was not assigned valid REAL values on line #' &
                            //TRIM( Int2LStr( LineNum ) )//'.'//NewLine//' >> The text being parsed was :'//NewLine &
                            //'    "'//TRIM( Line )//'"' 
            RETURN
            CALL Cleanup()         
        ENDIF

    !  IF ( PRESENT(UnEc) )  THEN
    !     IF ( UnEc > 0 )  WRITE (UnEc,'(A)')  TRIM( FileInfo%Lines(LineNum) )
    !  END IF

        LineNum = LineNum + 1
        CALL Cleanup()
    ENDIF

    RETURN

    !=======================================================================
    CONTAINS
    !=======================================================================
        SUBROUTINE Cleanup ( )

            ! This subroutine cleans up the parent routine before exiting.

            ! Deallocate the Words array if it had been allocated.

            IF ( ALLOCATED( Words_Ary ) ) DEALLOCATE( Words_Ary )


            RETURN

        END SUBROUTINE Cleanup

    END SUBROUTINE ParseInAry
    
!=======================================================================
subroutine ReadEmptyLine(Un,CurLine)
    INTEGER(4),         INTENT(IN   )          :: Un   ! Input file unit
    INTEGER(4),         INTENT(INOUT)          :: CurLine   ! Current line of input

    CHARACTER(1024)                            :: Line

    READ(Un, '(A)') Line
    CurLine = CurLine + 1

END subroutine ReadEmptyLine

!=======================================================================
!> This subroutine is used to get the NumWords "words" from a line of text.
!! It uses spaces, tabs, commas, semicolons, single quotes, and double quotes ("whitespace")
!! as word separators. If there aren't NumWords in the line, the remaining array elements will remain empty.
!! Use CountWords (nwtc_io::countwords) to count the number of words in a line.
SUBROUTINE GetWords ( Line, Words, NumWords )

    ! Argument declarations.

    INTEGER, INTENT(IN)          :: NumWords                                     !< The number of words to look for.

    CHARACTER(*), INTENT(IN)     :: Line                                         !< The string to search.
    CHARACTER(*), INTENT(OUT)    :: Words(NumWords)                              !< The array of found words.


        ! Local declarations.

    INTEGER                      :: Ch                                           ! Character position within the string.
    INTEGER                      :: IW                                           ! Word index.
    INTEGER                      :: NextWhite                                    ! The location of the next whitespace in the string.
    CHARACTER(1), PARAMETER       :: Tab      = CHAR( 9 ) 



        ! Let's prefill the array with blanks.

    DO IW=1,NumWords
        Words(IW) = ' '
    END DO ! IW


        ! Let's make sure we have text on this line.

    IF ( LEN_TRIM( Line ) == 0 )  RETURN


        ! Parse words separated by any combination of spaces, tabs, commas,
        ! semicolons, single quotes, and double quotes ("whitespace").

    Ch = 0
    IW = 0

    DO

        NextWhite = SCAN( Line(Ch+1:) , ' ,!;''"'//Tab )

        IF ( NextWhite > 1 )  THEN

        IW        = IW + 1
        Words(IW) = Line(Ch+1:Ch+NextWhite-1)

        IF ( IW == NumWords )  EXIT

        Ch = Ch + NextWhite

        ELSE IF ( NextWhite == 1 )  THEN

        Ch = Ch + 1

        CYCLE

        ELSE

        EXIT

        END IF

    END DO


    RETURN
END SUBROUTINE GetWords

!=======================================================================
 !> This subroutine checks the data to be parsed to make sure it finds
    !! the expected variable name and an associated value.
SUBROUTINE ChkParseData ( Words, ExpVarName, FileName, FileLineNum, ErrVar )

    USE LDP_Types, ONLY : LidarErrorVariables

        ! Arguments declarations.
    TYPE(LidarErrorVariables),         INTENT(INOUT)          :: ErrVar   ! Current line of input

    INTEGER(4), INTENT(IN)             :: FileLineNum                   !< The number of the line in the file being parsed.
    INTEGER(4)                        :: NameIndx                      !< The index into the Words array that points to the variable name.

    CHARACTER(*),   INTENT(IN)             :: ExpVarName                    !< The expected variable name.
    CHARACTER(*),   INTENT(IN)             :: Words       (2)               !< The two words to be parsed from the line.

    CHARACTER(*),   INTENT(IN)             :: FileName                      !< The name of the file being parsed.


        ! Local declarations.

    CHARACTER(20)                          :: ExpUCVarName                  ! The uppercase version of ExpVarName.
    CHARACTER(20)                          :: FndUCVarName                  ! The uppercase version of the word being tested.




        ! Convert the found and expected names to uppercase.

    FndUCVarName = Words(1)
    ExpUCVarName = ExpVarName

    CALL Conv2UC ( FndUCVarName )
    CALL Conv2UC ( ExpUCVarName )

    ! See which word is the variable name.  Generate an error if it is the first
        
    IF ( TRIM( FndUCVarName ) == TRIM( ExpUCVarName ) )  THEN
        NameIndx = 1
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = ' >> A fatal error occurred when parsing data from "'//TRIM( FileName ) &
                            //'".'//NewLine//' >> The variable "'//TRIM( Words(1) )//'" was not assigned a valid value on line #' &
                            //TRIM( Int2LStr( FileLineNum ) )//'.' 
        RETURN
    ELSE
        FndUCVarName = Words(2)
        CALL Conv2UC ( FndUCVarName )
        IF ( TRIM( FndUCVarName ) == TRIM( ExpUCVarName ) )  THEN
        NameIndx = 2
        ELSE
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg = ' >> A fatal error occurred when parsing data from "'//TRIM( FileName ) &
                            //'".'//NewLine//' >> The variable "'//TRIM( ExpVarName )//'" was not assigned a valid value on line #' &
                            //TRIM( Int2LStr( FileLineNum ) )//'.' 
        RETURN
        ENDIF
    ENDIF


END SUBROUTINE ChkParseData 

!=======================================================================
!> This routine converts all the text in a string to upper case.
    SUBROUTINE Conv2UC ( Str )

        ! Argument declarations.
  
     CHARACTER(*), INTENT(INOUT)  :: Str                                          !< The string to be converted to UC (upper case).
  
  
        ! Local declarations.
  
     INTEGER                      :: IC                                           ! Character index
  
  
  
     DO IC=1,LEN_TRIM( Str )
  
        IF ( ( Str(IC:IC) >= 'a' ).AND.( Str(IC:IC) <= 'z' ) )  THEN
           Str(IC:IC) = CHAR( ICHAR( Str(IC:IC) ) - 32 )
        END IF
  
     END DO ! IC
  
  
     RETURN
    END SUBROUTINE Conv2UC

    
!=======================================================================
     !> This function returns a left-adjusted string representing the passed numeric value. 
    !! It eliminates trailing zeroes and even the decimal point if it is not a fraction. \n
    !! Use Num2LStr (nwtc_io::num2lstr) instead of directly calling a specific routine in the generic interface.   
    FUNCTION Int2LStr ( Num )

        CHARACTER(11)                :: Int2LStr                                     !< string representing input number.
    
    
        ! Argument declarations.
    
        INTEGER, INTENT(IN)          :: Num                                          !< The number to convert to a left-justified string.
    
    
    
        WRITE (Int2LStr,'(I11)')  Num
    
        Int2Lstr = ADJUSTL( Int2LStr )
    
    
        RETURN
    END FUNCTION Int2LStr
    
END MODULE LDP_Subs
