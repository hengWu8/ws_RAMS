MODULE TRob1Main
! WS_RAMS EGM diagnostic variant.
!
! Goal:
! - Keep the current VC-parity behavior and real-cell safety posture.
! - Add one ordinary RAPID Move instruction before the first EGMRunJoint call.
!
! Why:
! - ABB's EGM manual states that, after a controller restart, the first movement
!   must not be an EGM movement.
! - This file uses a near-zero ordinary MoveAbsJ to the robot's current joint
!   target in order to satisfy that prerequisite with minimal physical motion.
! - A zoned stop is used instead of fine, because exact-stop at the current
!   target can hang on the real controller even when physical motion is tiny.
!
! Compared to TRob1Main_WS_RAMS_EGM_Joint_VC_Parity.mod:
! - Same EGMSetupUC / EGMActJoint / EGMRunJoint shape
! - Same egm_condition = [-0.1, 0.1]
! - Same CondTime = 5
! - Same RampOutTime = 5
! - Same MaxSpeedDeviation = 20.0
! - Adds one preparatory MoveAbsJ to CJointT() before EGMGetId

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    PROC main()
        TPWrite "entered main";
        TPWrite "WS_RAMS EGM diagnostic mode: preparatory MoveAbsJ to current joint target";

        ! Execute one ordinary RAPID move after controller restart without
        ! intentionally changing pose. This is a diagnostic test for the ABB
        ! documented prerequisite that the first movement may not be an EGM move.
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
