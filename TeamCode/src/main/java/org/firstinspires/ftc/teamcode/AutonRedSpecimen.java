package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "RedSpecimen")
public class AutonRedSpecimen extends LinearOpMode {
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
    boolean puncherExtended = false;
    boolean intakeTransferConfirm = false, scoreTransferConfirm;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        PartialRoadrunnerHelper roadrunner = new PartialRoadrunnerHelper(drive, robot.drivetrain::remote);

        States state = States.PRELOAD;
        ElapsedTime autonomous = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(270)));

        roadrunner.setPIDCoefficients(Axis.X, 0, 0, 0);
        roadrunner.setPIDCoefficients(Axis.Y, 0, 0, 0);
        roadrunner.setPIDCoefficients(Axis.HEADING, 0.8, 0, 0);

        robot.scoring.clawClose();
        intake = true;
        waitForStart();
        autonomous.reset();
        roadrunner.setPoseEstimate(9.34, -62.57, 90.00);
        while (opModeIsActive()) {
            if (intake) specimenScored = false;
            if (!intake) specimenScored = true;

            intake = robot.clawIntake.getPosition() == 0 || robot.clawScoring.getPosition() == 0;
            specimenScored = robot.puncherLeft.getPosition() == 0.21 && robot.puncherRight.getPosition() == 0.19;
            puncherExtended = robot.puncherLeft.getPosition() == 0.61 && robot.puncherRight.getPosition() == 0.59;
            intakeTransferConfirm = robot.clawIntake.getPosition() == 0;
            scoreTransferConfirm = robot.clawScoring.getPosition() == 0;

            if (state == States.PRELOAD) {
                // TODO: Tune y value
                roadrunner.setTarget(0, -28.57, 90);
                robot.verticalSlider.setHighChamber();
                robot.scoring.arm.setScoring();
                robot.scoring.alignTurret(robot.getHeadingDeg(), 90);
                robot.scoring.clawClose();

                if (autonomous.seconds() > 4 && robot.verticalSlider.getCurrentPosition() > 1000) {
                    state = States.SCORE_PRELOAD;
                    timer1.reset();
                }
            }

            if (state == States.SCORE_PRELOAD) {
                // TODO: Tune y value
                roadrunner.setTarget(0, -28.57, 90);

                if (timer1.milliseconds() > 800) {
                    robot.verticalSlider.setPosition(0);
                    robot.scoring.arm.setTransfer();
                    robot.scoring.clawOpen();
                    if (!puncherExtended) robot.scoring.setTurret(0);

                    if (robot.verticalSlider.getCurrentPosition() > 900) {
                        robot.scoring.puncher.setRetracted();
                    } else robot.scoring.puncher.setExtended();

                } else {
                    robot.verticalSlider.setHighChamber();
                    robot.scoring.arm.setScoring();
                    robot.scoring.alignTurret(robot.getHeadingDeg(), 90);
                    robot.scoring.clawClose();

                    if (robot.verticalSlider.getCurrentPosition() > 900) {
                        robot.scoring.puncher.setExtended();
                    } else robot.scoring.puncher.setRetracted();
                }

                if (autonomous.seconds() > 7 || robot.verticalSlider.getCurrentPosition() < 300) {
                    state = States.PATH_TO_INTAKE;
                    timer1.reset();
                }
            }

            if (state == States.PATH_TO_INTAKE) {
                if (timer1.milliseconds() > 1000) {
                    // TODO: Tune x value
                    roadrunner.setTarget(23.94, -43.52, 180);
                    // TODO: Tune y value
                } else roadrunner.setTarget(0, -43.52, 180);

                if (autonomous.seconds() > 9) {
                    state = States.INTAKE_1;
                    timer1.reset();
                }
            }

            if (state == States.INTAKE_1) {
                robot.intake.setIntake();

                // TODO: Tune x value and heading
                if (intake) {
                    roadrunner.setTarget(23.94, -43.52,315);
                    if (timer1.milliseconds() > 500 && roadrunner.getHeadingDegrees() < 325) {
                        robot.intake.clawOpen();
                    } else robot.intake.clawClose();
                }
                else {
                    roadrunner.setTarget(23.94, -43.52,35);
                    robot.linearSlider.setExtended();
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }

                if (!intake || autonomous.seconds() > 13) {
                    state = States.INTAKE_2;
                    timer1.reset();
                }
            }

            if (state == States.INTAKE_2) {
                robot.intake.setIntake();

                // TODO: Tune x value and heading
                if (intake) {
                    roadrunner.setTarget(38.18, -43.52, 315);
                    if (timer1.milliseconds() > 500 && roadrunner.getHeadingDegrees() < 325) {
                        robot.intake.clawOpen();
                    } else robot.intake.clawClose();
                }
                else {
                    roadrunner.setTarget(38.18, -43.52, 35);
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }

                if (!intake || autonomous.seconds() > 16) {
                    state = States.INTAKE_3;
                    timer1.reset();
                }
            }

            if (state == States.INTAKE_3) {
                robot.intake.setIntake();

                // TODO: Tune x value and heading
                if (intake) {
                    roadrunner.setTarget(47.97, -43.52, 315);
                    if (timer1.milliseconds() > 500 && roadrunner.getHeadingDegrees() < 325) {
                        robot.intake.clawOpen();
                    } else robot.intake.clawClose();
                }
                else {
                    roadrunner.setTarget(47.97, -43.52, 35);
                    robot.intake.clawOpen();
                    // TODO: Add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }

                if (!intake || autonomous.seconds() > 19) {
                    state = States.SCORE_1;
                    timer1.reset();
                }
            }

            if (state == States.SCORE_1) {
                robot.intake.setIntake();

                // TODO: Tune x, y, heading values
                if (intake || timer1.milliseconds() > 1500) {
                    // Return
                    if (timer1.milliseconds() > 2500) {
                        robot.verticalSlider.setPosition(0);
                        robot.scoring.arm.setTransfer();
                        robot.scoring.clawOpen();
                        if (!puncherExtended) robot.scoring.setTurret(0);

                        if (robot.verticalSlider.getCurrentPosition() > 900) {
                            robot.scoring.puncher.setRetracted();
                        } else robot.scoring.puncher.setExtended();

                    } else {
                        // Score specimen
                        roadrunner.setTarget(9.17, -37.82, 315);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSample();
                        robot.scoring.clawOpen();
                        if (intakeTransferConfirm) robot.scoring.clawClose();

                        if (intakeTransferConfirm && scoreTransferConfirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighChamber();
                            robot.scoring.arm.setScoring();
                            robot.scoring.alignTurret(robot.getHeadingDeg(), 30);
                            robot.scoring.clawClose();

                            if (robot.verticalSlider.getCurrentPosition() > 900) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }

                } else {
                    // Intake specimen
                    roadrunner.setTarget(23.76, -52.06, 315);
                    robot.intake.clawOpen();
                    // TODO: add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }

                if (autonomous.seconds() > 23 || robot.verticalSlider.getCurrentPosition() < 300) {
                    state = States.INTAKE_2;
                    timer1.reset();
                }
            }

            if (state == States.INTAKE_2) {
                robot.intake.setIntake();

                // TODO: Tune x, y, heading values
                if (intake || timer1.milliseconds() > 1500) {
                    // Return
                    if (timer1.milliseconds() > 2500) {
                        robot.verticalSlider.setPosition(0);
                        robot.scoring.arm.setTransfer();
                        robot.scoring.clawOpen();
                        if (!puncherExtended) robot.scoring.setTurret(0);

                        if (robot.verticalSlider.getCurrentPosition() > 900) {
                            robot.scoring.puncher.setRetracted();
                        } else robot.scoring.puncher.setExtended();

                    } else {
                        // Score specimen
                        roadrunner.setTarget(9.17, -37.82, 315);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSample();
                        robot.scoring.clawOpen();
                        if (intakeTransferConfirm) robot.scoring.clawClose();

                        if (intakeTransferConfirm && scoreTransferConfirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighChamber();
                            robot.scoring.arm.setScoring();
                            robot.scoring.alignTurret(robot.getHeadingDeg(), 35);
                            robot.scoring.clawClose();

                            if (robot.verticalSlider.getCurrentPosition() > 900) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }

                } else {
                    // Intake specimen
                    roadrunner.setTarget(23.76, -52.06, 315);
                    robot.intake.clawOpen();
                    // TODO: add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }

                if (autonomous.seconds() > 26 || robot.verticalSlider.getCurrentPosition() < 300) {
                    state = States.INTAKE_3;
                    timer1.reset();
                }
            }

            if (state == States.INTAKE_3) {
                robot.intake.setIntake();

                // TODO: Tune x, y, heading values
                if (intake || timer1.milliseconds() > 1500) {
                    // Return
                    if (timer1.milliseconds() > 2500) {
                        robot.verticalSlider.setPosition(0);
                        robot.scoring.arm.setTransfer();
                        robot.scoring.clawOpen();
                        if (!puncherExtended) robot.scoring.setTurret(0);

                        if (robot.verticalSlider.getCurrentPosition() > 900) {
                            robot.scoring.puncher.setRetracted();
                        } else robot.scoring.puncher.setExtended();

                    } else {
                        // Score specimen
                        roadrunner.setTarget(9.17, -37.82, 315);
                        robot.intake.clawClose();
                        robot.linearSlider.retract();
                        robot.intake.setTransferSample();
                        robot.scoring.clawOpen();
                        if (intakeTransferConfirm) robot.scoring.clawClose();

                        if (intakeTransferConfirm && scoreTransferConfirm) {
                            robot.intake.clawOpen();
                            robot.verticalSlider.setHighChamber();
                            robot.scoring.arm.setScoring();
                            robot.scoring.alignTurret(robot.getHeadingDeg(), 40);
                            robot.scoring.clawClose();

                            if (robot.verticalSlider.getCurrentPosition() > 900) {
                                robot.scoring.puncher.setExtended();
                            } else robot.scoring.puncher.setRetracted();
                        }
                    }

                } else {
                    // Intake specimen
                    roadrunner.setTarget(23.76, -52.06, 315);
                    robot.intake.clawOpen();
                    // TODO: add limelight
                    // If can sense sample, claw closes, override slider and roadrunner
                }

                if (autonomous.seconds() > 28.5 || robot.verticalSlider.getCurrentPosition() < 300) {
                    state = States.PARK;
                    timer1.reset();
                }
            }

            if (state == States.PARK) {
                roadrunner.setTarget(23.76, -52.06, 315);
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
