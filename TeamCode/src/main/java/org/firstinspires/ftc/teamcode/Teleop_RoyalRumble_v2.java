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
        SCORING
    }
    double vertical, horizontal, pivot, heading;
    boolean sampleMode = true, specimenMode = false;
    boolean intake = false, turretIsInPosition = false;
    double degrees = 0;

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
            turretIsInPosition = robot.scoring.turret.getPosition() >= 88 || robot.scoring.turret.getPosition() <= 92;

            if (gamepad.a) robot.imu.resetYaw();

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
                    robot.scoring.setTurret(0);
                    robot.scoring.arm.setSampleTransfer();
                    robot.intake.setTransferSample();
                    robot.intake.clawClose();

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
                        robot.intake.setIntake();

                        if (gamepad.b && !lastGamepad.b) {
                            robot.intake.setOrientation(degrees += 45);
                        }
                        if (gamepad.x && !lastGamepad.x) {
                            robot.intake.setOrientation(degrees -= 45);
                        }

                        if (operator.right_trigger > 0 && !(lastOperator.right_trigger > 0)) {
                            robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                        }
                        if (operator.left_trigger > 0 && !(lastOperator.left_trigger > 0)) {
                            robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
                        }
                    }

                    if (specimenMode) {
                        robot.intake.clawOpen();
                        robot.intake.setIntake();

                        if (operator.right_trigger > 0 && !(lastOperator.right_trigger > 0)) {
                            robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                        }
                        if (operator.left_trigger > 0 && !(lastOperator.left_trigger > 0)) {
                            robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
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
                        robot.intake.setIntake();
                    }

                    if (operator.right_trigger > 0 && !(lastOperator.right_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                    }
                    if (operator.left_trigger > 0 && !(lastOperator.left_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
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
                    if (sampleMode) robot.intake.setIntake();
                    if (specimenMode) robot.intake.setIntake();
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
                    robot.scoring.setTurret(0);
                    robot.intake.clawClose();

                    if (sampleMode) {
                        if (timer1.milliseconds() > 1000 && robot.intake.atTransfer) {
                            robot.intake.setTransferSample();
                            robot.scoring.arm.setSampleTransfer();
                            robot.scoring.clawClose();
                        } else {
                            robot.intake.setTransferSample();
                            robot.scoring.arm.setSampleTransfer();
                            robot.scoring.clawOpen();
                        }
                    }
                    if (specimenMode) {
                        if (timer1.milliseconds() > 1000 && robot.intake.atTransfer) {
                            robot.intake.setTransferSpecimen();
                            robot.scoring.arm.setSpecimenTransfer();
                            robot.scoring.clawClose();
                        } else {
                            robot.intake.setTransferSpecimen();
                            robot.scoring.arm.setSpecimenTransfer();
                            robot.scoring.clawOpen();
                        }
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
                    robot.scoring.setTurret(0);

                    if (timer1.milliseconds() > 500 && !robot.scoring.clawOpen) {
                        if (sampleMode) robot.intake.setTransferSample();
                        else robot.intake.setTransferSpecimen();
                        robot.intake.clawOpen();
                        robot.scoring.clawClose();

                        if (timer1.milliseconds() > 1000 && robot.intake.clawOpen) {
                            robot.scoring.arm.setScoring();
                            robot.scoring.clawClose();
                        }
                    } else {
                        if (sampleMode) robot.intake.setTransferSample();
                        else robot.intake.setTransferSpecimen();
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
                        robot.scoring.setTurret(0);
                        robot.scoring.arm.setScoring();
                        if (gamepad.right_trigger > 0) robot.scoring.clawOpen();
                        else robot.scoring.clawClose();
                    }

                    if (specimenMode) {
                        // TODO: Tune this
                        if (robot.scoring.puncher.getPosition() >= 0.19) {
                            robot.scoring.alignTurret(robot.getHeadingDeg(), 90);
                        }
                        if (turretIsInPosition) {
                            robot.scoring.puncher.setExtended();
                        }
                        robot.scoring.arm.setScoring();

                        if (timer1.milliseconds() > 800) {
                            robot.intake.clawOpen();
                        } else robot.intake.clawClose();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.INIT;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.READY_SCORE;
                        timer1.reset();
                    }
                    break;
            }

            telemetry.addData("state", state);
            telemetry.addData("Sample mode", sampleMode);
            telemetry.addData("Specimen mode", specimenMode);

            telemetry.update();
            robot.drivetrain.remote(vertical, -horizontal, pivot, heading);
        }
    }
}
