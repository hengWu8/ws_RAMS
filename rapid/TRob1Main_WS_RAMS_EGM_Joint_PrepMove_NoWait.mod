MODULE TRob1Main
! WS_RAMS EGM diagnostic variant.
!
! Goal:
! - Keep the preparatory ordinary Move that improved EGM stability on the real cell.
! - Use the ABB-documented asynchronous EGM pattern: EGMRunJoint \NoWaitCond
!   followed by EGMWaitCond.
!
! Why:
! - ABB documents that \NoWaitCond releases the program pointer before the movement
!   is complete, and that EGMWaitCond is the mandatory completion point.
! - This variant tests whether the real controller behaves more predictably with
!   the documented asynchronous lifecycle than with repeated synchronous re-entry.
!
! Compared to TRob1Main_WS_RAMS_EGM_Joint_PrepMove.mod:
! - Same preparatory MoveAbsJ CJointT()
! - Same EGMSetupUC / EGMActJoint parameters
! - Same CondTime = 5, RampOutTime = 5, MaxSpeedDeviation = 20.0
! - Uses \NoWaitCond + EGMWaitCond instead of synchronous EGMRunJoint in a loop

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    PROC main()
        TPWrite "entered main";
        TPWrite "WS_RAMS async EGM diagnostic mode";

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

        TPWrite "starting async EGMRunJoint";
        EGMRunJoint egm_id, EGM_STOP_HOLD \NoWaitCond, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5;
        EGMWaitCond egm_id;
        TPWrite "EGMWaitCond returned";

        EGMReset egm_id;
        TPWrite "EGMReset done";

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timedout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
