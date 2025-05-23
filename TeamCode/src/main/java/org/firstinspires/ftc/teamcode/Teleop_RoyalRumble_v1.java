package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Teleop_RoyalRumble_v1 (intake)")
public class Teleop_RoyalRumble_v1 extends LinearOpMode {
    private enum State {
        INIT,
        GROUND_INTAKE,
        SUB_INTAKE,
        GRIP,
        STAGING,
        READY_SCORE,
        SCORING,
        RETURN_TO_INIT
    }
    double vertical, horizontal, pivot, heading;
    boolean sampleMode = false, specimenMode = true;
    boolean intake = false;
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
            vertical = -gamepad.left_stick_y; horizontal = gamepad.left_stick_x;
            pivot = gamepad.right_stick_x; heading = robot.getHeadingRad();

            intake = robot.clawIntake.getPosition() == 0;
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);
            lastOperator.copy(operator);
            operator.copy(gamepad2);

            robot.scoring.puncher.setRetracted();
            robot.scoring.arm.setScoring();

            if (gamepad.y) robot.resetIMU();

            /*if (operator.dpad_left && !lastOperator.dpad_left) {
                sampleMode = true;
                specimenMode = false;
            }
            if (operator.dpad_right && !lastOperator.dpad_right) {
                sampleMode = false;
                specimenMode = true;
            }*/

            if (gamepad.dpad_up && !lastGamepad.dpad_up) robot.linearSlider.setExtended();
            if (gamepad.dpad_down && !lastGamepad.dpad_down) robot.linearSlider.retract();

            switch (state) {
                case INIT:
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
                        state = State.STAGING;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.GROUND_INTAKE;
                        timer1.reset();
                    }
                    break;

                 case STAGING:
                     robot.linearSlider.retract();

                    /*if (sampleMode) {
                        robot.intake.setPitch(0.7);
                        robot.intake.clawClose();

                        if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                            state = State.SCORING;
                            timer1.reset();
                        }
                    }*///

                    if (specimenMode) {
                        robot.intake.setTransferSample();
                        robot.intake.clawClose();

                        if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                            state = State.SCORING;
                            timer1.reset();
                        }
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.GRIP;
                        timer1.reset();
                    }
                    break;

                /*case READY_SCORE:
                    robot.intake.setPitch(0.85);
                    robot.intake.clawClose();

                    /*if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                }
                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
                }*/

                    /*if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.SCORING;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.STAGING;
                        specimenMode = true;
                        timer1.reset();
                    }
                    break;*/

                case SCORING:
                    if (sampleMode) {
                        robot.intake.setPitch(0.6);
                        if (gamepad.right_trigger > 0) {
                            robot.intake.clawOpen();
                        } else robot.intake.clawClose();
                    }

                    if (specimenMode) {
                        robot.linearSlider.setExtended();
                        if (timer1.milliseconds() > 1200) {
                            robot.intake.clawOpen();
                        } else robot.intake.clawClose();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.RETURN_TO_INIT;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.STAGING;
                        timer1.reset();
                    }
                    break;

                case RETURN_TO_INIT:
                    robot.linearSlider.retract();
                    if (timer1.milliseconds() > 800) {
                        state = State.INIT;
                        timer1.reset();
                    }
            }

            telemetry.addData("state", state);
            telemetry.addData("Sample mode", sampleMode);
            telemetry.addData("Specimen mode", specimenMode);

            telemetry.update();
            robot.drivetrain.remote(vertical, -horizontal, -pivot, heading);
        }
    }
}
