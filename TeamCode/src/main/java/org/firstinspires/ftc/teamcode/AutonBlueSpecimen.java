package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Disabled
@Autonomous (name = "BlueSpecimen")
public class AutonBlueSpecimen extends LinearOpMode {
    /*
       PRELOAD: Path to submersible
       SCORE_PRELOAD: Score specimen
       PATH_TO_INTAKE: Turn robot, move forward a little
       INTAKE_1: Turn robot, throw first sample to human player
       INTAKE_2: Turn robot, throw second sample to human player
       INTAKE_3: Turn robot, throw third sample to human player
       PATH_TO_SCORE: Move leftwards a little, diagonal to submersible and observation zone
       SCORE_1: Intake, then score first specimen
       SCORE_2: Intake, then score second specimen
       SCORE_3: Intake, then score third specimen
       PARK: Park at observation zone
     */
    enum States {
        PRELOAD,
        SCORE_PRELOAD,
        PATH_TO_INTAKE,
        INTAKE_1,
        INTAKE_2,
        INTAKE_3,
        SCORE_1,
        SCORE_2,
        SCORE_3,
        PARK
    }
    boolean specimenScored = false, intake = false;
    boolean turretMove_Confirm = false;
    boolean intakeTransfer_Confirm = false, scoreTransfer_Confirm;
    boolean vert_ScoreReturn_Confirm = false, vert_SwitchState_Confirm = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        PartialRoadrunnerHelper roadrunner = new PartialRoadrunnerHelper(drive, robot.drivetrain::remote);

        States state = States.PRELOAD;
        ElapsedTime autonomous = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();

        roadrunner.setPIDCoefficients(Axis.X, 0, 0, 0);
        roadrunner.setPIDCoefficients(Axis.Y, 0, 0, 0);
        roadrunner.setPIDCoefficients(Axis.HEADING, 0.8, 0, 0);

        robot.scoring.clawClose();
        robot.intake.clawOpen();

        waitForStart();
        autonomous.reset();
        roadrunner.setPoseEstimate(-14.68, 63.63, 270.00);
        while (opModeIsActive()) {
            if (intake) specimenScored = false;
            if (!intake) specimenScored = true;

            intake = robot.clawIntake.getPosition() == 0 || robot.clawScoring.getPosition() == 0;
            specimenScored = robot.puncherLeft.getPosition() == 0.21 && robot.puncherRight.getPosition() == 0.19;
            turretMove_Confirm = robot.puncherLeft.getPosition() > 0.33 && robot.puncherRight.getPosition() > 0.31;
            intakeTransfer_Confirm = robot.clawIntake.getPosition() == 0 && robot.intake.atTransfer;
            scoreTransfer_Confirm = robot.clawScoring.getPosition() == 0 && robot.scoring.arm.atTransfer;
            vert_ScoreReturn_Confirm = robot.verticalSlider.getCurrentPosition() > 1000;
            vert_SwitchState_Confirm = robot.verticalSlider.getCurrentPosition() < 300;

            if (state == States.PRELOAD) {
                // TODO: Tune y value
                roadrunner.setTarget(0, 31.77, 270);
                if (roadrunner.getTargetY() >= 42.81) {
                    // Ready score
                    robot.verticalSlider.setHighChamber();
                    robot.scoring.arm.setScoring();
                    robot.scoring.alignTurret(robot.getHeadingDeg(), 270);
                    robot.scoring.clawClose();
                }
                if (autonomous.seconds() > 4 && roadrunner.getTargetY() >= 32.57) {
                    state = States.SCORE_PRELOAD;
                    timer1.reset();
                }
            }

            if (state == States.SCORE_PRELOAD) {
                // TODO: Tune y value
                roadrunner.setTarget(0, 31.77, 90);

                if (timer1.milliseconds() > 800) {
                    // Return
                    robot.verticalSlider.setPosition(0);
                    robot.scoring.arm.setSpecimenTransfer();
                    robot.scoring.clawOpen();
                    if (!turretMove_Confirm) robot.scoring.setTurret(0);

                    if (vert_ScoreReturn_Confirm) {
                        robot.scoring.puncher.setRetracted();
                    } else robot.scoring.puncher.setExtended();

                } else {
                    // Score
                    robot.verticalSlider.setHighChamber();
                    robot.scoring.arm.setScoring();
                    if (turretMove_Confirm) {
                        robot.scoring.alignTurret(robot.getHeadingDeg(), 270);
                    }
                    robot.scoring.clawClose();

                    if (vert_ScoreReturn_Confirm) {
                        robot.scoring.puncher.setExtended();
                    } else robot.scoring.puncher.setRetracted();
                }

                if (autonomous.seconds() > 7 || vert_SwitchState_Confirm) {
                    state = States.PATH_TO_INTAKE;
                    timer1.reset();
                }
            }

            if (state == States.PATH_TO_INTAKE) {
                if (timer1.milliseconds() > 1000) {
                    // TODO: Tune x value
                    roadrunner.setTarget(-23.94, 43.52, 180);
                    // TODO: Tune y value
                } else roadrunner.setTarget(0, 43.52, 0);

                if (autonomous.seconds() > 9 || roadrunner.isInPosition()) {
                    state = States.INTAKE_1;
                    timer1.reset();
                }
            }

            if (state == States.INTAKE_1) {
                robot.intake.setIntake();

                // TODO: Tune x value and heading
                if (intake) {
                    // Throw sample to human player
                    roadrunner.setTarget(-23.94, 43.52, 145);
                    if (timer1.milliseconds() > 500 && roadrunner.getHeadingDegrees() < 155) {
                        robot.intake.clawOpen();
                    } else robot.intake.clawClose();

                    if (!intake || autonomous.seconds() > 13) {
                        state = States.INTAKE_2;
                        timer1.reset();
                    }
                } else {
                    // Intake ground sample
                    roadrunner.setTarget(-23.94, 43.52, 215);
                    robot.linearSlider.setExtended();
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.INTAKE_2) {
                robot.intake.setIntake();

                // TODO: Tune x value and heading
                if (intake) {
                    // Throw sample to human player
                    roadrunner.setTarget(-38.18, 43.52, 145);
                    if (timer1.milliseconds() > 500 && roadrunner.getHeadingDegrees() < 155) {
                        robot.intake.clawOpen();
                    } else robot.intake.clawClose();

                    if (!intake || autonomous.seconds() > 16) {
                        state = States.INTAKE_3;
                        timer1.reset();
                    }
                } else {
                    // Intake ground sample
                    roadrunner.setTarget(-38.18, 43.52, 215);
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.INTAKE_3) {
                robot.intake.setIntake();

                // TODO: Tune x value and heading
                if (intake) {
                    // Throw sample to human player
                    roadrunner.setTarget(47.97, -43.52, 145);
                    if (timer1.milliseconds() > 500 && roadrunner.getHeadingDegrees() < 155) {
                        robot.intake.clawOpen();
                    } else robot.intake.clawClose();

                    if (!intake || autonomous.seconds() > 19) {
                        state = States.SCORE_1;
                        timer1.reset();
                    }
                } else {
                    // Intake ground sample
                    roadrunner.setTarget(-47.97, 43.52, 215);
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.SCORE_1) {
                // TODO: Tune x, y, heading values
                if (intake || timer1.milliseconds() > 1500) {
                    if (timer1.milliseconds() > 2500) {
                        // Return
                        robot.verticalSlider.setPosition(0);
                        robot.scoring.arm.setSpecimenTransfer();
                        robot.scoring.clawOpen();
                        if (!turretMove_Confirm) robot.scoring.setTurret(0);

                        if (vert_ScoreReturn_Confirm) {
                            robot.scoring.puncher.setRetracted();
                        } else robot.scoring.puncher.setExtended();

                        if (autonomous.seconds() > 23 || vert_SwitchState_Confirm) {
                            state = States.SCORE_2;
                            timer1.reset();
                        }

                    } else {
                        // Score specimen
                        roadrunner.setTarget(-9.17, 37.82, 135);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSpecimen();
                        robot.scoring.clawOpen();
                        if (intakeTransfer_Confirm) robot.scoring.clawClose();

                        if (intakeTransfer_Confirm && scoreTransfer_Confirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighChamber();
                            robot.scoring.arm.setScoring();
                            if (turretMove_Confirm) {
                                robot.scoring.alignTurret(robot.getHeadingDeg(), 210);
                            }
                            robot.scoring.clawClose();

                            if (vert_ScoreReturn_Confirm) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }

                } else {
                    // Intake specimen from ground
                    roadrunner.setTarget(-25.01, 50.82,135);
                    robot.intake.setIntake();
                    robot.intake.clawOpen();
                    // TODO: add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.SCORE_2) {
                // TODO: Tune x, y, heading values
                if (intake || timer1.milliseconds() > 1500) {
                    // Return
                    if (timer1.milliseconds() > 2500) {
                        robot.verticalSlider.setPosition(0);
                        robot.scoring.arm.setSpecimenTransfer();
                        robot.scoring.clawOpen();
                        if (!turretMove_Confirm) robot.scoring.setTurret(0);

                        if (vert_ScoreReturn_Confirm) {
                            robot.scoring.puncher.setRetracted();
                        } else robot.scoring.puncher.setExtended();

                        if (autonomous.seconds() > 26 || vert_SwitchState_Confirm) {
                            state = States.SCORE_3;
                            timer1.reset();
                        }

                    } else {
                        // Score specimen
                        roadrunner.setTarget(-9.17, 37.82, 315);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSpecimen();
                        robot.scoring.clawOpen();
                        if (intakeTransfer_Confirm) robot.scoring.clawClose();

                        if (intakeTransfer_Confirm && scoreTransfer_Confirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighChamber();
                            robot.scoring.arm.setScoring();
                            if (turretMove_Confirm) {
                                robot.scoring.alignTurret(robot.getHeadingDeg(), 215);
                            }
                            robot.scoring.clawClose();

                            if (vert_ScoreReturn_Confirm) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }

                } else {
                    // Intake specimen from ground
                    roadrunner.setTarget(-25.01, 50.82,135);
                    robot.intake.setIntake();
                    robot.intake.clawOpen();
                    // TODO: add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.SCORE_3) {
                // TODO: Tune x, y, heading values
                if (intake || timer1.milliseconds() > 1500) {
                    // Return
                    if (timer1.milliseconds() > 2500) {
                        robot.verticalSlider.setPosition(0);
                        robot.scoring.arm.setSpecimenTransfer();
                        robot.scoring.clawOpen();
                        if (!turretMove_Confirm) robot.scoring.setTurret(0);

                        if (vert_ScoreReturn_Confirm) {
                            robot.scoring.puncher.setRetracted();
                        } else robot.scoring.puncher.setExtended();

                        if (autonomous.seconds() > 28.5 || vert_SwitchState_Confirm) {
                            state = States.PARK;
                            timer1.reset();
                        }

                    } else {
                        // Score specimen
                        roadrunner.setTarget(-9.17, 37.82, 315);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSpecimen();
                        robot.scoring.clawOpen();
                        if (intakeTransfer_Confirm) robot.scoring.clawClose();

                        if (intakeTransfer_Confirm && scoreTransfer_Confirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighChamber();
                            robot.scoring.arm.setScoring();
                            if (turretMove_Confirm) {
                                robot.scoring.alignTurret(robot.getHeadingDeg(), 220);
                            }
                            robot.scoring.clawClose();

                            if (vert_ScoreReturn_Confirm) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }

                } else {
                    // Intake specimen
                    roadrunner.setTarget(-25.01, 50.82,135);
                    robot.intake.setIntake();
                    robot.intake.clawOpen();
                    // TODO: add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.PARK) {
                roadrunner.setTarget(-25.01, 50.82,135);
                robot.intake.setIntake();
                robot.linearSlider.setExtended();
                robot.intake.clawOpen();
            }

            roadrunner.update();
            telemetry.addLine(roadrunner.toTelemetry());
            telemetry.addData("State", state);
            telemetry.addData("vert", robot.verticalSlider.getCurrentPosition());
            telemetry.update();
        }

    }
}