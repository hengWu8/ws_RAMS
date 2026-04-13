MODULE TRob1Main
! WS_RAMS VC-parity EGM joint streaming entrypoint.
!
! Goal:
! - Match the upstream/RobotStudio TRob1Main EGM path as closely as possible.
! - Keep the real-cell safety behavior of NOT forcing MoveAbsJ home on start.
!
! Compared to the upstream RobotStudio sample:
! - Same EGMSetupUC / EGMActJoint / EGMRunJoint shape
! - Same egm_condition = [-0.1, 0.1]
! - Same CondTime = 5, RampOutTime = 5
! - Same MaxSpeedDeviation = 20.0
! - MoveAbsJ home intentionally removed for real-cell safety

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    PROC main()
        TPWrite "entered main";
        TPWrite "WS_RAMS VC parity mode: staying at current pose, no MoveAbsJ home";

        EGMGetId egm_id;
        EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Joint;

        EGMActJoint egm_id
                    \J1:=egm_condition
                    \J2:=egm_condition
                    \J3:=egm_condition
                    \J4:=egm_condition
                    \J5:=egm_condition
                    \J6:=egm_condition
                    \MaxSpeedDeviation:=20.0;

        WHILE TRUE DO
            EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5;
        ENDWHILE

        EGMReset egm_id;

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timedout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
