MODULE TRob1Main
! WS_RAMS EGM diagnostic variant.
!
! Goal:
! - Keep the current VC-parity behavior and real-cell safety posture.
! - Change only one runtime variable: CondTime from 5 to 30.
!
! Why:
! - On the real controller, the low-level EGM state is oscillating between
!   EGM_RUNNING and EGM_STOPPED with a period strongly correlated to ~5 s.
! - This file is meant to test whether CondTime is the direct trigger.
!
! Compared to TRob1Main_WS_RAMS_EGM_Joint_VC_Parity.mod:
! - Same EGMSetupUC / EGMActJoint / EGMRunJoint shape
! - Same egm_condition = [-0.1, 0.1]
! - Same RampOutTime = 5
! - Same MaxSpeedDeviation = 20.0
! - Same "stay at current pose" behavior
! - Only CondTime is changed: 5 -> 30

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    PROC main()
        TPWrite "entered main";
        TPWrite "WS_RAMS EGM diagnostic mode: CondTime=30, no MoveAbsJ home";

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
            EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=30 \RampOutTime:=5;
        ENDWHILE

        EGMReset egm_id;

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timedout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
