! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------

! The pitch controller is modified to accept the pitch forward pitch signal
! by Feng Guo  Flensburg University of Applied Sciences, funded by LIKE -- Lidar Knowledge Europe, grant agreement No. 858358.   
! Target: This code aims to provide a reference Lidar-assisted control package for the community. Please cite the following paper if this code is helpful for your research:
! Guo, F., Schlipf, D., and Cheng, P. W.: Evaluation of lidar-assisted wind turbine control under various turbulence characteristics, Wind Energ. Sci. Discuss.
! [preprint], https://doi.org/10.5194/wes-2022-62, in review, 2022.    


! High level run script

!=======================================================================
SUBROUTINE ROSCO(avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG)  BIND (C, NAME='ROSCO')
! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

USE, INTRINSIC  :: ISO_C_Binding
USE             :: ROSCO_Types
USE             :: ReadSetParameters
USE             :: ControllerBlocks
USE             :: Controllers
USE             :: Constants
USE             :: Filters
USE             :: Functions
!USE             :: LidarDataProcessing

IMPLICIT NONE
! Enable .dll export
#ifndef IMPLICIT_DLLEXPORT
!DEC$ ATTRIBUTES DLLEXPORT :: ROSCO
!GCC$ ATTRIBUTES DLLEXPORT :: ROSCO
#endif

!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------

! Passed Variables:
!REAL(C_FLOAT), INTENT(IN)      :: from_SC(*)       ! DATA from the super controller
!REAL(C_FLOAT), INTENT(INOUT)   :: to_SC(*)         ! DATA to the super controller

REAL(C_FLOAT),                  INTENT(INOUT)   :: avrSWAP(*)                       ! The swap array, used to pass data to, and receive data from, the DLL controller.
INTEGER(C_INT),                 INTENT(INOUT)   :: aviFAIL                          ! A flag used to indicate the success of this DLL call set as follows: 0 if the DLL call was successful, >0 if the DLL call was successful but cMessage should be issued as a warning messsage, <0 if the DLL call was unsuccessful or for any other reason the simulation is to be stopped at this point with cMessage as the error message.
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: avcOUTNAME(NINT(avrSWAP(51)))    ! OUTNAME (Simulation RootName)
CHARACTER(KIND=C_CHAR),         INTENT(INOUT)   :: avcMSG(NINT(avrSWAP(49)))        ! MESSAGE (Message from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
CHARACTER(SIZE(avcOUTNAME)-1)                   :: RootName                         ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)                       :: ErrMsg                           ! a Fortran version of the C string argument (not considered an array here) [subtract 1 for the C null-character]

TYPE(ControlParameters),        SAVE           :: CntrPar
TYPE(LocalVariables),           SAVE           :: LocalVar
TYPE(ObjectInstances),          SAVE           :: objInst
TYPE(PerformanceData),          SAVE           :: PerfData
TYPE(DebugVariables),           SAVE           :: DebugVar
TYPE(ErrorVariables),           SAVE           :: ErrVar

CHARACTER(*),                   PARAMETER      :: RoutineName = 'ROSCO'

RootName = TRANSFER(avcOUTNAME, RootName)
!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------
! Read avrSWAP array into derived types/variables
CALL ReadAvrSWAP(avrSWAP, LocalVar)

! Set Control Parameters
CALL SetParameters(avrSWAP, accINFILE, SIZE(avcMSG), CntrPar, LocalVar,objInst, PerfData, ErrVar)

! Filter signals
CALL PreFilterMeasuredSignals(CntrPar, LocalVar, objInst)

! Process Lidar data and get the lidar estimated rotor effective wind 
!CALL WindFieldReconstruction(CntrPar, LocalVar,objInst)



IF ((LocalVar%iStatus >= 0) .AND. (ErrVar%aviFAIL >= 0))  THEN  ! Only compute control calculations if no error has occurred and we are not on the last time step
    CALL WindSpeedEstimator(LocalVar, CntrPar, objInst, PerfData, DebugVar, ErrVar)
    CALL ComputeVariablesSetpoints(CntrPar, LocalVar, objInst)
    CALL StateMachine(CntrPar, LocalVar)
    CALL SetpointSmoother(LocalVar, CntrPar, objInst)
    CALL ComputeVariablesSetpoints(CntrPar, LocalVar, objInst)
    CALL VariableSpeedControl(avrSWAP, CntrPar, LocalVar, objInst)
    CALL PitchControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
    CALL YawRateControl(avrSWAP, CntrPar, LocalVar, objInst)
    CALL FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
    CALL Debug(LocalVar, CntrPar, DebugVar, avrSWAP, RootName, SIZE(avcOUTNAME))
END IF

! Add RoutineName to error message
IF (ErrVar%aviFAIL < 0) THEN
    ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
    print * , TRIM(ErrVar%ErrMsg)
ENDIF
ErrMsg = ErrVar%ErrMsg
avcMSG = TRANSFER(TRIM(ErrVar%ErrMsg)//C_NULL_CHAR, avcMSG, SIZE(avcMSG))
aviFAIL = ErrVar%aviFAIL

RETURN
END SUBROUTINE ROSCO
