package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "RedSample")
public class AutonRedSample extends LinearOpMode {
    enum States {
        PRELOAD,
        SCORE_PRELOAD,
        INTAKE_1,
        SCORE_1,
        INTAKE_2,
        SCORE_2,
        INTAKE_3,
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
        roadrunner.setPoseEstimate(-33.2, -63.1, 90.00);

        while (opModeIsActive()) {

            intake = robot.clawIntake.getPosition() == 0 || robot.clawScoring.getPosition() == 0;
            specimenScored = robot.puncherLeft.getPosition() == 0.21 && robot.puncherRight.getPosition() == 0.19;
            turretMove_Confirm = robot.puncherLeft.getPosition() > 0.33 && robot.puncherRight.getPosition() > 0.31;
            intakeTransfer_Confirm = robot.clawIntake.getPosition() == 0 && robot.intake.atTransfer;
            scoreTransfer_Confirm = robot.clawScoring.getPosition() == 0 && robot.scoring.arm.atTransfer;
            vert_ScoreReturn_Confirm = robot.verticalSlider.getCurrentPosition() > 1000;
            vert_SwitchState_Confirm = robot.verticalSlider.getCurrentPosition() < 300;

            if (state == States.PRELOAD) {
                // TODO: Tune x, y, heading values
                roadrunner.setTarget(-54.02, -52.95, 225);
                if (roadrunner.isInPosition() || timer1.milliseconds() > 750) {
                    robot.verticalSlider.setHighBasket();
                    robot.scoring.arm.setScoring();
                    robot.scoring.alignTurret(robot.getHeadingDeg(), 225);
                    robot.scoring.clawClose();
                }
                if (autonomous.seconds() > 3 || vert_ScoreReturn_Confirm) {
                    state = States.SCORE_PRELOAD;
                    timer1.reset();
                }
            }

            if (state == States.SCORE_PRELOAD) {
                // TODO: Tune y value
                roadrunner.setTarget(-54.02, -52.95, 225);

                if (timer1.milliseconds() > 800) {
                    // Return
                    robot.verticalSlider.setPosition(0);
                    robot.scoring.arm.setTransfer();
                    robot.scoring.clawOpen();
                    if (!turretMove_Confirm) robot.scoring.setTurret(0);

                    if (vert_ScoreReturn_Confirm) {
                        robot.scoring.puncher.setRetracted();
                    } else robot.scoring.puncher.setExtended();

                } else {
                    // Score
                    robot.verticalSlider.setHighBasket();
                    robot.scoring.arm.setScoring();
                    if (turretMove_Confirm) {
                        robot.scoring.alignTurret(robot.getHeadingDeg(), 225);
                    }
                    robot.scoring.clawClose();

                    if (vert_ScoreReturn_Confirm) {
                        robot.scoring.puncher.setExtended();
                    } else robot.scoring.puncher.setRetracted();
                }
            }

            if (autonomous.seconds() > 7 || vert_SwitchState_Confirm) {
                state = States.INTAKE_1;
                timer1.reset();
            }

            if (state == States.INTAKE_1) {
                // TODO: Tune x, y and heading value
                if (intake) {
                    // Ready score
                    roadrunner.setTarget(-54.02, -52.95, 225);
                    robot.intake.setTransferSample();
                    robot.intake.clawClose();
                    robot.scoring.clawOpen();

                    if (intake && timer1.milliseconds() > 500) {
                        state = States.SCORE_1;
                        timer1.reset();
                    } else if (!intake && autonomous.seconds() > 10) {
                        state = States.INTAKE_2;
                        timer1.reset();
                    }
                } else {
                    // Intake sample
                    roadrunner.setTarget(-54.38, -46.72, 270);
                    robot.linearSlider.setExtended();
                    robot.intake.setIntake();
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.SCORE_1) {
                if (timer1.milliseconds() > 2500) {
                    // Return
                    robot.verticalSlider.setPosition(0);
                    robot.scoring.arm.setTransfer();
                    robot.scoring.clawOpen();
                    if (!turretMove_Confirm) robot.scoring.setTurret(0);

                    if (vert_ScoreReturn_Confirm) {
                        robot.scoring.puncher.setRetracted();
                    } else robot.scoring.puncher.setExtended();

                    if (autonomous.seconds() > 14 || vert_SwitchState_Confirm) {
                        state = States.INTAKE_2;
                        timer1.reset();
                    } else {
                        // Score sample
                        roadrunner.setTarget(-54.02, -52.78, 225);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSpecimen();
                        robot.scoring.clawOpen();
                        if (intakeTransfer_Confirm) robot.scoring.clawClose();

                        if (intakeTransfer_Confirm && scoreTransfer_Confirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighBasket();
                            robot.scoring.arm.setScoring();
                            if (turretMove_Confirm) {
                                robot.scoring.alignTurret(robot.getHeadingDeg(), 225);
                            }
                            robot.scoring.clawClose();

                            if (vert_ScoreReturn_Confirm) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }
                }
            }

            if (state == States.INTAKE_2) {
                // TODO: Tune x, y and heading value
                if (intake) {
                    // Ready score
                    roadrunner.setTarget(-54.38, -46.72, 270);
                    robot.intake.setTransferSample();
                    robot.intake.clawClose();
                    robot.scoring.clawOpen();

                    if (intake && timer1.milliseconds() > 700) {
                        state = States.SCORE_2;
                        timer1.reset();
                    } else if (!intake && autonomous.seconds() > 17) {
                        state = States.INTAKE_3;
                        timer1.reset();
                    }
                } else {
                    // Intake sample
                    roadrunner.setTarget(-54.38, -46.72, 260);
                    robot.linearSlider.setExtended();
                    robot.intake.setIntake();
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.SCORE_2) {
                if (timer1.milliseconds() > 2500) {
                    // Return
                    robot.verticalSlider.setPosition(0);
                    robot.scoring.arm.setTransfer();
                    robot.scoring.clawOpen();
                    if (!turretMove_Confirm) robot.scoring.setTurret(0);

                    if (vert_ScoreReturn_Confirm) {
                        robot.scoring.puncher.setRetracted();
                    } else robot.scoring.puncher.setExtended();

                    if (autonomous.seconds() > 20 || vert_SwitchState_Confirm) {
                        state = States.INTAKE_3;
                        timer1.reset();
                    } else {
                        roadrunner.setTarget(-60.07, -48.68, 270);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSpecimen();
                        robot.scoring.clawOpen();
                        if (intakeTransfer_Confirm) robot.scoring.clawClose();

                        if (intakeTransfer_Confirm && scoreTransfer_Confirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighBasket();
                            robot.scoring.arm.setScoring();
                            if (turretMove_Confirm) {
                                robot.scoring.alignTurret(robot.getHeadingDeg(), 225);
                            }
                            robot.scoring.clawClose();

                            if (vert_ScoreReturn_Confirm) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }
                }
            }

            if (state == States.INTAKE_3) {
                // TODO: Tune x, y and heading value
                if (intake) {
                    // Ready score
                    roadrunner.setTarget(-54.38, -46.72, 270);
                    robot.intake.setTransferSample();
                    robot.intake.clawClose();
                    robot.scoring.clawOpen();

                    if (intake && timer1.milliseconds() > 700) {
                        state = States.SCORE_3;
                        timer1.reset();
                    } else if (!intake && autonomous.seconds() > 26) {
                        state = States.PARK;
                        timer1.reset();
                    }
                } else {
                    // Intake sample
                    roadrunner.setTarget(-54.38, -46.72, 235);
                    robot.linearSlider.setExtended();
                    robot.intake.setIntake();
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }
            }

            if (state == States.SCORE_3) {
                if (timer1.milliseconds() > 2500) {
                    // Return
                    robot.verticalSlider.setPosition(0);
                    robot.scoring.arm.setTransfer();
                    robot.scoring.clawOpen();
                    if (!turretMove_Confirm) robot.scoring.setTurret(0);

                    if (vert_ScoreReturn_Confirm) {
                        robot.scoring.puncher.setRetracted();
                    } else robot.scoring.puncher.setExtended();

                    if (autonomous.seconds() > 26 || vert_SwitchState_Confirm) {
                        state = States.PARK;
                        timer1.reset();
                    } else {
                        // Score
                        roadrunner.setTarget(-60.07, -48.68, 270);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSpecimen();
                        robot.scoring.clawOpen();
                        if (intakeTransfer_Confirm) robot.scoring.clawClose();

                        if (intakeTransfer_Confirm && scoreTransfer_Confirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighBasket();
                            robot.scoring.arm.setScoring();
                            if (turretMove_Confirm) {
                                robot.scoring.alignTurret(robot.getHeadingDeg(), 225);
                            }
                            robot.scoring.clawClose();

                            if (vert_ScoreReturn_Confirm) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }
                }
            }

            if (state == States.PARK) {
                roadrunner.setTarget(0, -31.59, 90);
                robot.intake.setIntakeRaised();

                if (roadrunner.isInPosition() || autonomous.seconds() > 27.5) {
                    robot.linearSlider.setExtended();
                    robot.intake.setIntake();
                    // TODO: Add limelight
                }
            }

            roadrunner.update();
            telemetry.addLine(roadrunner.toTelemetry());
            telemetry.addData("State", state);
            telemetry.addData("vert", robot.verticalSlider.getCurrentPosition());
            telemetry.update();
        }
    }
}
