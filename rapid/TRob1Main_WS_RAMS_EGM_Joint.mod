MODULE TRob1Main
! WS_RAMS safe EGM joint streaming entrypoint.
!
! IMPORTANT:
! - This version intentionally does NOT MoveAbsJ home on start.
! - Start it only when the robot is already in the desired handoff pose.
! - The ROS side still starts with publish_commands=false and must be explicitly armed.
! - Use AUTO + Motors On + local E-stop supervision for first tests.

    LOCAL VAR egmident egm_id;

    ! EGM convergence window in degrees. The ABB sample uses [-0.1, 0.1],
    ! which is too small for visible ROS-side smoke tests. This does not by
    ! itself move the robot; it lets EGM accept a larger joint correction window.
    LOCAL VAR egm_minmax egm_condition := [-2, 2];

    PROC main()
        TPWrite "WS_RAMS EGM joint streaming: starting at current pose, no MoveAbsJ home.";

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

        ! Keep these aligned with the previously validated RobotStudio EGM loop.
        ! The earlier 1000/1 variant was not behaviorally equivalent for visible motion tests.
        WHILE TRUE DO
            EGMRunJoint egm_id, EGM_STOP_HOLD, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5;
        ENDWHILE

        EGMReset egm_id;

    ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "WS_RAMS EGM communication timeout";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE
