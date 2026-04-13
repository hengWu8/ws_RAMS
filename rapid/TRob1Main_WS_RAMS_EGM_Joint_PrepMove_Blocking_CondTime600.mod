MODULE TRob1Main
! WS_RAMS real-controller EGM candidate runtime with an operator-defined
! preparatory home/ready point.
!
! Goal:
! - Avoid the real-controller stall observed with the asynchronous
!   EGMRunJoint \NoWaitCond + EGMWaitCond pattern.
! - Keep the ordinary preparatory move that satisfies ABB's "first movement
!   after controller start must not be EGM" prerequisite.
! - Make that preparatory move an explicit operator-defined target, not CJointT().
! - Keep EGM in one long blocking EGMRunJoint window for visible slow tests.
!
! Why CondTime=600:
! - egm_condition is a convergence condition, not a workspace limit.
! - If the command remains inside [-0.1, 0.1] deg error, EGMRunJoint completes
!   after CondTime seconds even while ROS is still streaming.
! - A long CondTime keeps the EGM session alive for tests such as 100-200 s
!   slow Cartesian motion, while the ROS bridge still enforces workspace and
!   per-step safety limits.
!
! Operator setup:
! - Before running on the real robot, update egm_ready to a known-safe jointtarget
!   from the FlexPendant/RobotStudio.
! - This point should be close to the intended EGM start pose and clear of all
!   fixtures, cameras, cables, and humans.
!
! Safety posture:
! - No automatic MoveAbsJ home to a hidden/default target.
! - No MoveAbsJ CJointT(); that was observed to cause unexpected pre-EGM motion
!   on the real controller.
! - The only pre-EGM motion is the explicit operator-defined egm_ready target.

    LOCAL VAR egmident egm_id;
    LOCAL VAR egm_minmax egm_condition := [-0.1, 0.1];

    ! EDIT THIS POINT MANUALLY BEFORE RUNNING.
    ! Current default is the last observed safe-ish test pose from WS_RAMS logs,
    ! only to avoid a dangerous all-zero placeholder. Replace it with your own
    ! verified home/EGM-ready jointtarget.
    PERS jointtarget egm_ready := [[0.131528, 0.025436, 38.461166, -0.155532, -38.954636, 0.856903],
                                  [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

    PROC main()
        TPWrite "entered main";
        TPWrite "WS_RAMS blocking EGM mode: move to operator egm_ready first";

        MoveAbsJ egm_ready, v10, fine, tool0;

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
