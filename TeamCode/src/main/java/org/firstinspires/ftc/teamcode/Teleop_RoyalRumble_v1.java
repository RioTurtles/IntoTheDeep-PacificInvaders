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
        SCORING
    }
    double vertical, horizontal, pivot, heading;
    boolean sampleMode = true, specimenMode = false;
    boolean intake = false;
    double degrees = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        State state = State.INIT;
        ElapsedTime timer1 = new ElapsedTime();

        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();

        waitForStart();
        while (opModeIsActive()) {
            vertical = -gamepad.left_stick_y; horizontal = gamepad.left_stick_x;
            pivot = gamepad.right_stick_x; heading = robot.getHeadingRad();

            intake = robot.clawIntake.getPosition() == 0;
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            robot.scoring.puncher.setRetracted();
            robot.scoring.arm.setScoring();

            if (gamepad.a) robot.imu.resetYaw();

            if (gamepad.dpad_left && !lastGamepad.dpad_left) {
                sampleMode = true;
                specimenMode = false;
            }
            if (gamepad.dpad_right && !lastGamepad.dpad_right) {
                sampleMode = false;
                specimenMode = true;

            }

            switch (state) {
                case INIT:
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

                /*if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                }
                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
                }*/
                    }

                    if (specimenMode) {
                        robot.intake.clawOpen();
                        robot.intake.setIntake();

                        /*if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                }
                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
                }*/
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
                    if (gamepad.b && !lastGamepad.b) robot.intake.setIntake();

                    /*if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                }
                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
                }*/

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

                    if (sampleMode) {
                        robot.intake.setPitch(0.7);
                        robot.intake.clawClose();

                        if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                            state = State.SCORING;
                            timer1.reset();
                        }
                    }

                    if (specimenMode) {
                        robot.intake.setTransferSample();
                        robot.intake.clawClose();

                        if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                            state = State.READY_SCORE;
                            timer1.reset();
                        }
                    }

                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.GRIP;
                        timer1.reset();
                    }
                    break;

                case READY_SCORE:
                    robot.intake.setPitch(0.85);
                    robot.intake.clawClose();

                    /*if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() + 100);
                }
                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    robot.linearSlider.setPosition(robot.linearSlider.getCurrentPosition() - 100);
                }*/

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.SCORING;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.STAGING;
                        specimenMode = true;
                        timer1.reset();
                    }
                    break;

                case SCORING:
                    if (sampleMode) {
                        robot.intake.setPitch(0.6);
                        if (gamepad.right_trigger > 0) {
                            robot.intake.clawOpen();
                        } else robot.intake.clawClose();
                    }

                    if (specimenMode) {
                        robot.intake.setIntakeRaised(); // Score low chamber
                        if (timer1.milliseconds() > 800) {
                            robot.intake.clawOpen();
                        } else robot.intake.clawClose();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.INIT;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.STAGING;
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
