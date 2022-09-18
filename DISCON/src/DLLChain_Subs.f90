! Name:   		Master (wrapper) DLL.
! Authors: 		Feng Guo, David Schlipf from Flensburg University of Applied Sciences, funded by LIKE -- Lidar Knowledge Europe, grant agreement No. 858358.   
! Target: 		This code aims to provide a reference Lidar-assisted control package for the community. Please cite the following paper if this code is helpful for your research:
! 				Guo, F., Schlipf, D., and Cheng, P. W.: Evaluation of lidar-assisted wind turbine control under various turbulence characteristics, Wind Energ. Sci. Discuss.
! 				[preprint], https://doi.org/10.5194/wes-2022-62, in review, 2022.    
! Function: 	The DLL chain is designed to make the lidar data processing or other algorithms more independent from the feedback controller. 
! 				It allows a more flexible design of additional algorithms which meet the requirement of a "smart lidar" concept (https://zenodo.org/record/5004524#.Yevsp_7MKUk)
! Reference:	The subroutines rely on the legacy Bladed style data interface. See the Bladed manual for more detail.    
! 				The code is written based on the source code of ROSCO. Version 2.4.1, https://github.com/NREL/ROSCO, 2021. by NREL.
! License: 		MIT License
! Copyright (c) 2022 Flensburg University of Applied Sciences, WETI
! -------------------------------------------------------------------------------------------

MODULE DLLChain_Subs

    USE, INTRINSIC :: ISO_C_Binding
    USE            :: DLLChain_Types
    
    IMPLICIT NONE

    ! Global Variables
    LOGICAL, PARAMETER     		:: DEBUG_PARSING = .FALSE.      ! debug flag to output parsing information, set up Echo file later
    CHARACTER(*),  PARAMETER    :: NewLine     = ACHAR(10)  	! The delimiter for New Lines [ Windows is CHAR(13)//CHAR(10); MAC is CHAR(13); Unix is CHAR(10) {CHAR(13)=\r is a line feed, CHAR(10)=\n is a new line}]
    
    INTERFACE ParseInput                                     	! Parses a character variable name and value from a string.
        MODULE PROCEDURE ParseInput_Str                      	! Parses a character string from a string.
        MODULE PROCEDURE ParseInput_Dbl                       	! Parses a double-precision REAL from a string.
        MODULE PROCEDURE ParseInput_Int                    		! Parses an INTEGER from a string.
    END INTERFACE
    



CONTAINS
 ! ----------------------------------------------------------------------------------- 
! -----------------------------------------------------------------------------------
    ! Get the sub DLL information 
    SUBROUTINE SetDLLParameters(avrSWAP, accINFILE, size_avcMSG, DLL_Type, DLL_ErrVar)
        USE DLLChain_Types
        
        REAL(C_FLOAT),              INTENT(INOUT)    		:: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        CHARACTER(C_CHAR),          INTENT(IN   )    		:: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file

        INTEGER(4),                 INTENT(IN   )   		:: size_avcMSG
        TYPE(DLLErrorVariables),           INTENT(INOUT) 	:: DLL_ErrVar
        TYPE(DLLChainParameter_Types),     INTENT(INOUT)   	:: DLL_Type

        
        INTEGER(4)                              			:: iStatus ! the status of the DLL chain calling, 0 means first call
        
        CHARACTER(*),               PARAMETER       		:: RoutineName = 'SetDLLParameters'

        
        iStatus            = NINT(avrSWAP(1))

        ! Set ErrVar%aviFAIL to 0 in each iteration:
        DLL_ErrVar%aviFAIL = 0
        ! ALLOCATE(ErrVar%ErrMsg(size_avcMSG-1))
        DLL_ErrVar%size_avcMSG  = size_avcMSG
        
        
        ! Read any External DLL Parameters specified in the User Interface
        !   and initialize variables:
        IF (iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            
            ! Description:
            print *, '--------------------------------------------------------------------'
            print *, 'A DLL chain for developing lidar-assisted control - v1.0'
            print *, 'Developed by Flensburg University of Applied Sciences, Germany'
            print *, '--------------------------------------------------------------------'    

            CALL ReadDLLParameterFileSub(DLL_Type,accINFILE, NINT(avrSWAP(50)),DLL_ErrVar)
            
            ! If there's been an file reading error, don't continue
            ! Add RoutineName to error message
            IF (DLL_ErrVar%aviFAIL < 0) THEN
                DLL_ErrVar%ErrMsg = RoutineName//':'//TRIM(DLL_ErrVar%ErrMsg)
                RETURN
            ENDIF

            

        ENDIF
    END SUBROUTINE SetDLLParameters
    
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadDLLParameterFileSub(DLL_Type,accINFILE, accINFILE_size,DLL_ErrVar)!, accINFILE_size)
        USE, INTRINSIC :: ISO_C_Binding
        USE DLLChain_Types, ONLY : DLLErrorVariables
        
        INTEGER(4)                                      :: accINFILE_size               ! size of DISCON input filename
        CHARACTER(accINFILE_size),  INTENT(IN   )       :: accINFILE(accINFILE_size)    ! DISCON input filename
        TYPE(DLLChainParameter_Types),        INTENT(INOUT)       :: DLL_Type

        TYPE(DLLErrorVariables),       INTENT(INOUT)       :: DLL_ErrVar             	! Control parameter type

        INTEGER(4),                 PARAMETER           :: UnControllerParameters = 89  ! Unit number to open file
        INTEGER(4)                                      :: CurLine 
        INTEGER(4)                                      :: ppos             			! index to remove file extension
        INTEGER(4)                                      :: stringlength     			! length of a string
        INTEGER(4)                                      :: iDLL                 		! counter  
 
        CHARACTER(*),               PARAMETER           :: RoutineName = 'ReadDLLParameterFileSub'

        CurLine = 1
       

        OPEN(unit=UnControllerParameters, file=accINFILE(1), status='old', action='read')
        
        !----------------------- HEADER ------------------------
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)
        CALL ReadEmptyLine(UnControllerParameters,CurLine)

        !----------------------- NumberOfsubDLLs ------------------------
        CALL ParseInput(UnControllerParameters,CurLine,'NumberofSubDLLs',accINFILE(1),DLL_Type%NumberOfsubDLLs,DLL_ErrVar)
        
        
        IF (.not. allocated(DLL_TYPE%PROCADDR)) THEN 
            ALLOCATE(DLL_TYPE%PROCADDR(DLL_Type%NumberOfsubDLLs))
        END IF
        
        IF (.not. allocated(DLL_TYPE%DLLFILENAME)) THEN 
            ALLOCATE(DLL_TYPE%DLLFILENAME(DLL_Type%NumberOfsubDLLs))
        END IF
        
        IF (.not. allocated(DLL_TYPE%DLLINPUTFILENAME)) THEN 
            ALLOCATE(DLL_TYPE%DLLINPUTFILENAME(DLL_Type%NumberOfsubDLLs))
        END IF
        
        ! LOOP to get all DLL names and DLL input names
        DO iDLL=1, DLL_Type%NumberOfsubDLLs
			CALL ParseInput(UnControllerParameters,CurLine,'SubDLLName',     accINFILE(1),DLL_TYPE%DLLFILENAME(iDLL),     DLL_ErrVar)
			CALL ParseInput(UnControllerParameters,CurLine,'SubDLLInputName',accINFILE(1),DLL_TYPE%DLLINPUTFILENAME(iDLL),DLL_ErrVar)
	      
            IF (len_trim( DLL_TYPE%DLLINPUTFILENAME(iDLL))==0) THEN
                DLL_ErrVar%ErrMsg = RoutineName//':'//'Error reading the DISCON.IN, the number of sub DLLs does not match with the lines in the DISCON.IN file, check the DISCON.IN is correctly set up.'
                print * , TRIM(DLL_ErrVar%ErrMsg)
                DLL_ErrVar%aviFAIL = -1
            RETURN
            END IF

        END DO    
        
        
        ! Close Input File
        CLOSE(UnControllerParameters)

        

        ! Add RoutineName to error message
        IF (DLL_ErrVar%aviFAIL < 0) THEN
            DLL_ErrVar%ErrMsg = RoutineName//':'//TRIM(DLL_ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE ReadDLLParameterFileSub
    ! -----------------------------------------------------------------------------------
 	!============================================================================================================================================== 
	! Below this line, the file I/O related routines are copied from ROSCO    
    !==============================================================================================================================================        
    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors
    ! the ParseInput is copied from the ROSCO_Types
    subroutine ParseInput_Int(Un, CurLine, VarName, FileName, Variable, ErrVar, CheckName)
        USE DLLChain_Types, ONLY : DLLErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(4),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(DLLErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
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
        USE DLLChain_Types, ONLY : DLLErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(4),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(DLLErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
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
        USE DLLChain_Types, ONLY : DLLErrorVariables

        CHARACTER(1024)                         :: Line
        INTEGER(4),             INTENT(IN   )   :: Un   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: VarName   ! Input file unit
        CHARACTER(*),           INTENT(IN   )   :: FileName   ! Input file unit
        INTEGER(4),             INTENT(INOUT)   :: CurLine   ! Current line of input
        TYPE(DLLErrorVariables),   INTENT(INOUT)   :: ErrVar   ! Current line of input
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

    USE DLLChain_Types, ONLY : DLLErrorVariables


        ! Arguments declarations.
    TYPE(DLLErrorVariables),         INTENT(INOUT)          :: ErrVar   ! Current line of input

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
    
END MODULE DLLChain_Subs
