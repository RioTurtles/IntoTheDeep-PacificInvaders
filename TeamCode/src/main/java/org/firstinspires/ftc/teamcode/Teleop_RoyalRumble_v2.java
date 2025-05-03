package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Teleop_RoyalRumble_v2 (intake + scoring)")
public class Teleop_RoyalRumble_v2 extends LinearOpMode {
    private enum State {
        INIT,
        GROUND_INTAKE,
        SUB_INTAKE,
        GRIP,
        TRANSFER,
        READY_SCORE,
        SCORING,
        RETURN_TO_INIT
    }
    double vertical, horizontal, pivot, heading;
    boolean sampleMode = false, specimenMode = true;
    boolean intake = false;
    boolean turretIsInPosition = false, puncherIsInPosition = false;
    double degrees = 0;
    int linearSliderPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        State state = State.INIT;
        ElapsedTime timer1 = new ElapsedTime();

        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        waitForStart();
        while (opModeIsActive()) {
            vertical = -gamepad.left_stick_y;
            horizontal = gamepad.left_stick_x;
            pivot = gamepad.right_stick_x;
            heading = robot.getHeadingRad();

            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);
            lastOperator.copy(operator);
            operator.copy(gamepad2);

            intake = robot.clawIntake.getPosition() == 0;
            turretIsInPosition = robot.scoring.turret.getPosition() >= 88 && robot.scoring.turret.getPosition() <= 92;
            puncherIsInPosition = robot.scoring.puncher.getPosition() >= 0.19 && robot.scoring.puncher.getPosition() <= 0.3;

            if (gamepad.y) robot.resetIMU();

            if (operator.dpad_left && !lastOperator.dpad_left) {
                sampleMode = true;
                specimenMode = false;
            }
            if (operator.dpad_right && !lastOperator.dpad_right) {
                sampleMode = false;
                specimenMode = true;
            }

            switch (state) {
                case INIT:
                    robot.scoring.puncher.setRetracted();
                    robot.scoring.arm.setSampleTransfer();
                    robot.linearSlider.retract();
                    robot.intake.setTransferSample();
                    robot.intake.clawOpen();

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.GROUND_INTAKE;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.INIT;
                        timer1.reset();
                    }
                    break;

                case GROUND_INTAKE:
                    if (sampleMode) {
                        robot.intake.clawOpen();
                        robot.intake.setSampleIntake();

                        if (gamepad.b && !lastGamepad.b) {
                            robot.intake.setOrientation(degrees += 45);
                        }
                        if (gamepad.x && !lastGamepad.x) {
                            robot.intake.setOrientation(degrees -= 45);
                        }

                        if (operator.b && !lastOperator.b) robot.linearSlider.set(1600);
                        if (operator.x && !lastOperator.x) robot.linearSlider.set(0);

                        if (operator.right_bumper && !lastOperator.right_bumper) {
                            robot.linearSlider.set(linearSliderPos += 533);
                        }
                        if (operator.left_bumper && !lastOperator.left_bumper) {
                            robot.linearSlider.set(linearSliderPos -= 533);
                        }

                        if (operator.right_trigger > 0 && !(lastOperator.right_trigger > 0)) {
                            robot.linearSlider.set(robot.linearSlider.getCurrentPosition() + 100);
                        }
                        if (operator.left_trigger > 0 && !(lastOperator.left_trigger > 0)) {
                            robot.linearSlider.set(robot.linearSlider.getCurrentPosition() - 100);
                        }
                    }

                    if (specimenMode) {
                        robot.intake.clawOpen();
                        robot.intake.setSpecimenIntake();

                        if (gamepad.b && !lastGamepad.b) {
                            robot.intake.setOrientation(degrees += 45);
                        }
                        if (gamepad.x && !lastGamepad.x) {
                            robot.intake.setOrientation(degrees -= 45);
                        }

                        if (operator.b && !lastOperator.b) robot.linearSlider.set(1600);
                        if (operator.x && !lastOperator.x) robot.linearSlider.set(0);

                        if (operator.right_bumper && !lastOperator.right_bumper) {
                            robot.linearSlider.set(linearSliderPos += 533);
                        }
                        if (operator.left_bumper && !lastOperator.left_bumper) {
                            robot.linearSlider.set(linearSliderPos -= 533);
                        }

                        if (operator.right_trigger > 0 && !(lastOperator.right_trigger > 0)) {
                            robot.linearSlider.set(robot.linearSlider.getCurrentPosition() + 100);
                        }
                        if (operator.left_trigger > 0 && !(lastOperator.left_trigger > 0)) {
                            robot.linearSlider.set(robot.linearSlider.getCurrentPosition() - 100);
                        }
                    }

                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        state = State.SUB_INTAKE;
                        timer1.reset();
                    }
                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.GRIP;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.INIT;
                        timer1.reset();
                    }
                    break;

                case SUB_INTAKE:
                    robot.intake.setIntakeRaised();
                    robot.intake.clawOpen();
                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        robot.intake.setSampleIntake();
                    }

                    if (gamepad.b && !lastGamepad.b) {
                        robot.intake.setOrientation(degrees += 45);
                    }
                    if (gamepad.x && !lastGamepad.x) {
                        robot.intake.setOrientation(degrees -= 45);
                    }

                    if (operator.b && !lastOperator.b) robot.linearSlider.set(1600);
                    if (operator.x && !lastOperator.x) robot.linearSlider.set(0);

                    if (operator.right_bumper && !lastOperator.right_bumper) {
                        robot.linearSlider.set(linearSliderPos += 533);
                    }
                    if (operator.left_bumper && !lastOperator.left_bumper) {
                        robot.linearSlider.set(linearSliderPos -= 533);
                    }

                    if (operator.right_trigger > 0 && !(lastOperator.right_trigger > 0)) {
                    robot.linearSlider.set(robot.linearSlider.getCurrentPosition() + 100);
                    }
                    if (operator.left_trigger > 0 && !(lastOperator.left_trigger > 0)) {
                    robot.linearSlider.set(robot.linearSlider.getCurrentPosition() - 100);
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.GRIP;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.INIT;
                        timer1.reset();
                    }
                    break;

                case GRIP:
                    if (sampleMode) robot.intake.setSampleIntake();
                    else if (specimenMode) robot.intake.setSpecimenIntake();
                    if (gamepad.right_trigger > 0) {
                        robot.intake.clawOpen();
                    } else robot.intake.clawClose();

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.TRANSFER;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.GROUND_INTAKE;
                        timer1.reset();
                    }
                    break;

                case TRANSFER:
                    robot.linearSlider.retract();
                    robot.scoring.puncher.setRetracted();
                    robot.intake.clawClose();

                        if (timer1.milliseconds() > 1000 && robot.intake.atTransfer) {
                            if (sampleMode) {
                                robot.intake.setTransferSample();
                                robot.scoring.arm.setSampleTransfer();
                            } else if (specimenMode) {
                                robot.intake.setTransferSpecimen();
                                robot.scoring.arm.setSpecimenTransfer();
                            }
                            robot.scoring.clawClose();
                        } else {
                            if (sampleMode) {
                                robot.intake.setTransferSample();
                                robot.scoring.arm.setSampleTransfer();
                            } else if (specimenMode) {
                                robot.intake.setTransferSpecimen();
                                robot.scoring.arm.setSpecimenTransfer();
                            }
                            robot.scoring.clawOpen();
                        }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.READY_SCORE;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.GRIP;
                        timer1.reset();
                    }
                    break;

                case READY_SCORE:
                    robot.linearSlider.retract();
                    robot.scoring.puncher.setRetracted();

                    if (timer1.milliseconds() > 500 && !robot.scoring.clawOpen) {
                        if (sampleMode) robot.intake.setTransferSample();
                        else if (specimenMode) robot.intake.setTransferSpecimen();
                        robot.intake.clawOpen();
                        robot.scoring.clawClose();

                        if (timer1.milliseconds() > 1000 && robot.intake.clawOpen) {
                            robot.scoring.arm.setSpecimenScoring();
                            robot.scoring.clawClose();
                        }
                    } else {
                        if (sampleMode) robot.intake.setTransferSample();
                        else if (specimenMode) robot.intake.setTransferSpecimen();
                        robot.intake.clawClose();
                        robot.scoring.clawClose();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.SCORING;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.TRANSFER;
                        timer1.reset();
                    }
                    break;

                case SCORING:
                    if (sampleMode) {
                        robot.scoring.puncher.setExtended();
                        robot.scoring.arm.setSpecimenScoring();
                        if (gamepad.right_trigger > 0) robot.scoring.clawOpen();
                        else robot.scoring.clawClose();
                    }

                    if (specimenMode) {
                        // TODO: Tune this
                        /*if (robot.scoring.puncher.getPosition() >= 0.19) {
                            robot.scoring.alignTurret(robot.getHeadingDeg(), 90);
                        }*/
                        robot.scoring.puncher.setExtended();
                        robot.scoring.arm.setSpecimenScoring();

                        if (timer1.milliseconds() > 1000 && timer1.milliseconds() < 1500 && robot.scoring.puncher.extendedPuncher) {
                            robot.scoring.arm.setSampleTransfer();
                        }

                        if (timer1.milliseconds() > 1500) {
                            robot.intake.clawOpen();
                        } else robot.intake.clawClose();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.RETURN_TO_INIT;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.READY_SCORE;
                        timer1.reset();
                    }
                    break;

                case RETURN_TO_INIT:
                    if (timer1.milliseconds() > 1000) {
                        robot.scoring.arm.setSampleTransfer();
                    } else {
                        robot.scoring.puncher.setRetracted();
                        robot.scoring.clawOpen();
                    }

                    if (puncherIsInPosition || timer1.milliseconds() > 1000) {
                        state = State.INIT;
                        timer1.reset();
                    }
            }

            telemetry.addData("state", state);
            telemetry.addData("Sample mode", sampleMode);
            telemetry.addData("Specimen mode", specimenMode);
            telemetry.addData("arm pos", robot.scoring.arm.getPosition());
            telemetry.addData("IMU", robot.getHeadingRad());

            telemetry.update();
            robot.drivetrain.remote(vertical, -horizontal, -pivot, heading);
        }
    }
}
