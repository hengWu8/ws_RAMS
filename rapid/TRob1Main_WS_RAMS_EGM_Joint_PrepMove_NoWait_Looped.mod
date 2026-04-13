MODULE TRob1Main
! WS_RAMS long-running EGM diagnostic / candidate runtime variant.
!
! Goal:
! - Keep the preparatory ordinary Move that improved real-controller stability.
! - Keep the task alive continuously instead of letting a single EGM cycle end the
!   RAPID program.
! - Use ABB's asynchronous EGM pattern: EGMRunJoint \NoWaitCond + EGMWaitCond.
!
! Compared to the other diagnostic variants:
! - Same preparatory MoveAbsJ CJointT() with z50 to avoid hanging on exact-stop
! - Same egm_condition = [-0.1, 0.1]
! - Same CondTime = 5
! - Same RampOutTime = 5
! - Same MaxSpeedDeviation = 20.0
! - Adds WHILE TRUE around the async EGM cycle so the RAPID task stays resident

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    PROC main()
        TPWrite "entered main";
        TPWrite "WS_RAMS async looped EGM mode";

        ! ABB documents that the first movement after controller start must not be
        ! an EGM movement. Use a near-zero ordinary move as a safe prerequisite.
        ! A zoned stop avoids hanging on exact-stop at the current target.
        MoveAbsJ CJointT(), v10, z50, tool0;

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
            TPWrite "starting async EGM cycle";
            EGMRunJoint egm_id, EGM_STOP_HOLD \NoWaitCond, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5;
            EGMWaitCond egm_id;
            TPWrite "EGM cycle returned";
        ENDWHILE

        EGMReset egm_id;
        TPWrite "EGMReset done";

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timedout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
