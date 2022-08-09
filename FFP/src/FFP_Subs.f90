! Name:   The subroutines for baseline Feed-forward pitch (FFP) DLL for lidar assisted feedforward pitch control.
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

MODULE FFP_Subs
!...............................................................................................................................
    !USE Constants   ! the constants from the ROSCO will be used
    USE :: FFP_Types
    
    IMPLICIT NONE

    ! Global Variables
    LOGICAL, PARAMETER          :: DEBUG_PARSING = .FALSE.      ! debug flag to output parsing information, set up Echo file later
    CHARACTER(*),  PARAMETER    :: NewLine     = ACHAR(10)              ! The delimiter for New Lines [ Windows is CHAR(13)//CHAR(10); MAC is CHAR(13); Unix is CHAR(10) {CHAR(13)=\r is a line feed, CHAR(10)=\n is a new line}]
    
    INTERFACE ParseInput                                                         ! Parses a character variable name and value from a string.
        MODULE PROCEDURE ParseInput_Str                                             ! Parses a character string from a string.
        MODULE PROCEDURE ParseInput_Dbl                                             ! Parses a double-precision REAL from a string.
        MODULE PROCEDURE ParseInput_Int                                             ! Parses an INTEGER from a string.
        ! MODULE PROCEDURE ParseInput_Log                                             ! Parses an LOGICAL from a string.
    END INTERFACE
    
    INTERFACE ParseAry                                                         ! Parse an array of numbers from a string.
        MODULE PROCEDURE ParseDbAry                                             ! Parse an array of double-precision REAL values.
        MODULE PROCEDURE ParseInAry                                             ! Parse an array of whole numbers.
    END INTERFACE


CONTAINS
    
    ! Read avrSWAP array into the local Lidar Variables
    SUBROUTINE ReadAvrSWAP(avrSWAP, LidarVar)
        USE :: FFP_Types, ONLY : LidarVariables

        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        TYPE(LidarVariables), INTENT(INOUT) :: LidarVar
        INTEGER(4)                      :: L           ! Record number for start of Lidar data
        INTEGER(4)                      :: I           ! index
        
        
        ! Load variables from calling program (See Appendix A of Bladed User's Guide):
        LidarVar%iStatus            = NINT(avrSWAP(1))
        LidarVar%Time               = avrSWAP(2)
        LidarVar%DT                 = avrSWAP(3)
        
        
        ! --- read and set the lidar variables
        L                       = NINT(avrSWAP(63))
        
       !> Gates per beam
        LidarVar%GatesPerBeam_IN   = avrSWAP(L+ 2) 
       
       
       !> Rotor effective wind and pitch forward rate
       LIDARVAR%AvrIndex_REWS            = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 7
       LIDARVAR%AvrIndex_REWS_b          = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 8
       LIDARVAR%AvrIndex_FFrate          = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 9
       LIDARVAR%AvrIndex_REWS_f          = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 10
       LIDARVAR%AvrIndex_REWS_leadtime   = L + 2 + NINT(LidarVar%GatesPerBeam_IN) + 11
    
       LidarVar%REWS                     = avrSWAP(LIDARVAR%AvrIndex_REWS)
       LidarVar%LeadTime_1               = avrSWAP(LIDARVAR%AvrIndex_REWS_leadtime)


       

    END SUBROUTINE ReadAvrSWAP    
    
    
    
    !-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE FeedForwardPitchRate(avrSWAP,LidarVar,ErrVar)
    ! Rescontruct rotor effection wind speed based on lidar line-of-sight measurement
    
        USE FFP_Types, ONLY : LidarVariables,LidarErrorVariables
        
        TYPE(LidarVariables), INTENT(INOUT)          :: LidarVar
        TYPE(LidarErrorVariables), INTENT(INOUT)     :: ErrVar
        !INTEGER(4)                                   :: objInst =1
        INTEGER(4)                                   :: iBuffer
        REAL(8)                                      :: IND_Time(LidarVar%MaxBufferStep_REWS)
        INTEGER(4)                                   :: It
        REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)   ! The swap array, used to pass data to, and receive data from, the DLL controller.
        
        !---------first in last out, shift the filter and Time in the buffer 
        LidarVar%REWS_f(2:LidarVar%MaxBufferStep_REWS) = LidarVar%REWS_f(1:LidarVar%MaxBufferStep_REWS-1)
        LidarVar%REWS_f_Time(2:LidarVar%MaxBufferStep_REWS) = LidarVar%REWS_f_Time(1:LidarVar%MaxBufferStep_REWS-1)
           
        ! low pass filtering
        LidarVar%REWS_f(1)      = LPFilter(LidarVar%REWS, LidarVar%DT, LidarVar%F_LPFCornerFreq, LidarVar%iStatus, .FALSE., LidarVar%LPF_inst)
        
        LidarVar%REWS_f_Time(1) = LidarVar%LeadTime_1 + LidarVar%Time
        
        !---------timing  Get the buffer ID that has the time closest to the pitch action time

        IND_Time   =  ABS(LidarVar%REWS_f_Time - LidarVar%Time - LidarVar%Pitch_ActTime)    ! not need ParamData%InitXPosition
        !END DO
        It          = INT(MINLOC(IND_Time,1))
        
        IF (IND_Time(It)<=(LidarVar%DT)) THEN ! only if the time difference is small
        LidarVar%REWS_b  = LidarVar%REWS_f(It)
        LidarVar%FF_Pitch = interp1d(LidarVar%REWS_curve,LidarVar%Pitch_curve,LidarVar%REWS_b,ErrVar)
        ELSE
        LidarVar%REWS_b = LidarVar%Uref_Ini
        LidarVar%FF_Pitch = interp1d(LidarVar%REWS_curve,LidarVar%Pitch_curve,LidarVar%REWS_b,ErrVar)
        END IF
        
        ! Calculate the pitch forward rate
        IF (LidarVar%iStatus == 0) THEN ! not initialized yet
        LidarVar%FF_Pitch_old = LidarVar%FF_Pitch
        LidarVar%FF_PitchRate = 0
        ELSE 
        LidarVar%FF_PitchRate = (LidarVar%FF_Pitch-LidarVar%FF_Pitch_old)/LidarVar%DT ! currently first order Euler
        LidarVar%FF_Pitch_old = LidarVar%FF_Pitch
        END IF
        
        
        
        
    END SUBROUTINE FeedForwardPitchRate  
    
    
 

    
    ! Low pass filter taken from ROSCO
    REAL FUNCTION LPFilter(InputSignal, DT, CornerFreq, iStatus, reset, inst)
    ! Discrete time Low-Pass Filter of the form:
    !                               Continuous Time Form:   H(s) = CornerFreq/(1 + CornerFreq)
    !                               Discrete Time Form:     H(z) = (b1z + b0) / (a1*z + a0)
    !
        REAL(8), INTENT(IN)         :: InputSignal
        REAL(8), INTENT(IN)         :: DT                       ! time step [s]
        REAL(8), INTENT(IN)         :: CornerFreq               ! corner frequency [rad/s]
        INTEGER(4), INTENT(IN)      :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER(4), INTENT(INOUT)   :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)      :: reset                    ! Reset the filter to the input signal

            ! Local
        REAL(8), DIMENSION(99), SAVE    :: a1                   ! Denominator coefficient 1
        REAL(8), DIMENSION(99), SAVE    :: a0                   ! Denominator coefficient 0
        REAL(8), DIMENSION(99), SAVE    :: b1                    ! Numerator coefficient 1
        REAL(8), DIMENSION(99), SAVE    :: b0                    ! Numerator coefficient 0 

        REAL(8), DIMENSION(99), SAVE    :: InputSignalLast      ! Input signal the last time this filter was called. Supports 99 separate instances.
        REAL(8), DIMENSION(99), SAVE    :: OutputSignalLast ! Output signal the last time this filter was called. Supports 99 separate instances.

            ! Initialization
        IF ((iStatus == 0) .OR. reset) THEN   
            OutputSignalLast(inst) = InputSignal
            InputSignalLast(inst) = InputSignal
            a1(inst) = 2 + CornerFreq*DT
            a0(inst) = CornerFreq*DT - 2
            b1(inst) = CornerFreq*DT
            b0(inst) = CornerFreq*DT
        ENDIF

        ! Define coefficients

        ! Filter
        LPFilter = 1.0/a1(inst) * (-a0(inst)*OutputSignalLast(inst) + b1(inst)*InputSignal + b0(inst)*InputSignalLast(inst))

        ! Save signals for next time step
        InputSignalLast(inst)  = InputSignal
        OutputSignalLast(inst) = LPFilter
        inst = inst + 1

    END FUNCTION LPFilter    
 
    REAL FUNCTION interp1d(xData, yData, xq, ErrVar)
    ! interp1d 1-D interpolation (table lookup), xData should be strictly increasing
        
        USE FFP_Types, ONLY : LidarErrorVariables
        IMPLICIT NONE

        ! Inputs
        REAL(8), DIMENSION(:), INTENT(IN)       :: xData        ! Provided x data (vector), to be interpolated
        REAL(8), DIMENSION(:), INTENT(IN)       :: yData        ! Provided y data (vector), to be interpolated
        REAL(8), INTENT(IN)                     :: xq           ! x-value for which the y value has to be interpolated
        INTEGER(4)                              :: I            ! Iteration index

        ! Error Catching
        TYPE(LidarErrorVariables), INTENT(INOUT)     :: ErrVar
        INTEGER(4)                              :: I_DIFF

        CHARACTER(*), PARAMETER                 :: RoutineName = 'interp1d'

        
        ! Catch Errors
        ! Are xData and yData the same size?
        IF (SIZE(xData) .NE. SIZE(yData)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = ' xData and yData are not the same size'
            WRITE(ErrVar%ErrMsg,"(A,I2,A,I2,A)") " SIZE(xData) =", SIZE(xData), & 
            ' and SIZE(yData) =', SIZE(yData),' are not the same'
        END IF

        ! Is xData non decreasing
        DO I_DIFF = 1, size(xData) - 1
            IF (xData(I_DIFF + 1) - xData(I_DIFF) <= 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = ' xData is not strictly increasing'
                EXIT 
            END IF
        END DO
        
        ! Interpolate
        IF (xq <= MINVAL(xData)) THEN
            interp1d = yData(1)
        ELSEIF (xq >= MAXVAL(xData)) THEN
            interp1d = yData(SIZE(xData))
        ELSE
            DO I = 1, SIZE(xData)
                IF (xq <= xData(I)) THEN
                    interp1d = yData(I-1) + (yData(I) - yData(I-1))/(xData(I) - xData(I-1))*(xq - xData(I-1))
                    EXIT
                ELSE
                    CONTINUE
                END IF
            END DO
        END IF

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        
    END FUNCTION interp1d
    
 ! ----------------------------------------------------------------------------------- 
! -----------------------------------------------------------------------------------
    ! Get the sub DLL informations 
    SUBROUTINE SetLidarParameters(avrSWAP, accINFILE, size_avcMSG, LidarVar, ErrVar)
        USE FFP_Types, ONLY : LidarErrorVariables, LidarVariables

        
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
        LidarVar%LPF_inst   = 1           ! instance to determine how many LPF we use
        
        ! Read the DLL Parameters specified in the User Interface
        !   and initialize variables:
        IF (LidarVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            ! Description:
            print *, '--------------------------------------------------------------------'
            print *, 'A baseline pitch forward controller'
            print *, 'Developed by Flensburg University of Applied Sciences, Germany'
            print *, 'Author: Feng Guo, David Schlipf'
            print *, '--------------------------------------------------------------------'
            
            
            CALL ReadLidarParameterFileSub(LidarVar,accINFILE, NINT(avrSWAP(50)),ErrVar)
            
            
            
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
        USE FFP_Types, ONLY : LidarErrorVariables,LidarVariables

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

        
        !------- FILTERS AND TIMING----------------------------------------------------------
        CALL ParseInput(UnControllerParameters,CurLine,'F_LPFCornerFreq',accINFILE(1),LidarVar%F_LPFCornerFreq,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Pitch_ActTime',accINFILE(1),LidarVar%Pitch_ActTime,ErrVar)
        CALL ParseInput(UnControllerParameters,CurLine,'Uref_Ini',accINFILE(1),LidarVar%Uref_Ini,ErrVar)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        
        
        !------- PITCH FORWARD CURVE ---------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ParseInput(UnControllerParameters,CurLine,'NumPitchCurveFF',accINFILE(1),LidarVar%NumPitchCurveFF,ErrVar)
        CALL ParseAry(UnControllerParameters,CurLine,'REWS_curve',LidarVar%REWS_curve,LidarVar%NumPitchCurveFF,accINFILE(1),ErrVar)
        CALL ParseAry(UnControllerParameters,CurLine,'Pitch_curve',LidarVar%Pitch_curve,LidarVar%NumPitchCurveFF,accINFILE(1),ErrVar)
        !CALL ReadEmptyLine(UnControllerParameters,CurLine)
        
        
        
        ! Close Input File
        CLOSE(UnControllerParameters)

        
        
        IF (.not. allocated(LidarVar%REWS_f)) THEN
        Allocate(LidarVar%REWS_f(LidarVar%MaxBufferStep_REWS))
        LidarVar%REWS_f(:) = 0
        END IF
        
        IF (.not. allocated(LidarVar%REWS_f_Time)) THEN
        Allocate(LidarVar%REWS_f_Time(LidarVar%MaxBufferStep_REWS))
        LidarVar%REWS_f_Time(:) = 999
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
        USE FFP_Types, ONLY : LidarErrorVariables

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
        USE FFP_Types, ONLY : LidarErrorVariables

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
        USE FFP_Types, ONLY : LidarErrorVariables

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

        USE FFP_Types, ONLY : LidarErrorVariables

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

    USE FFP_Types, ONLY : LidarErrorVariables

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

    USE FFP_Types, ONLY : LidarErrorVariables

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
    
END MODULE FFP_Subs
