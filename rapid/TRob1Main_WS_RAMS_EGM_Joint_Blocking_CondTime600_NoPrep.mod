MODULE TRob1Main
! WS_RAMS real-controller EGM runtime without any preparatory RAPID motion.
!
! Why this variant exists:
! - On the real controller, MoveAbsJ CJointT() was observed to cause an
!   unexpected pre-EGM pose change before ROS/EGM was connected.
! - This module therefore performs no RAPID motion before EGMRunJoint.
! - It relies on the ROS-side test/bringup flow to seed the forward position
!   controller with the current robot joints before RAPID EGM is started.
!
! Safety posture:
! - No MoveAbsJ home.
! - No MoveAbsJ CJointT().
! - A long blocking EGMRunJoint window keeps the EGM session alive while the ROS
!   bridge provides all command gating, workspace guarding, and disarm behavior.

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    PROC main()
        TPWrite "entered main";
        TPWrite "WS_RAMS blocking EGM mode: CondTime=600, no RAPID prep move";

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
            TPWrite "starting blocking EGM window CondTime600";
            EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=600 \RampOutTime:=5;
            TPWrite "blocking EGM window returned";
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
