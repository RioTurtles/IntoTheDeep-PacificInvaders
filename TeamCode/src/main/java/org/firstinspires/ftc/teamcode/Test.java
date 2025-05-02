package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "test")
public class Test extends LinearOpMode {
    private enum State {
        INIT,
        INTAKE,
        GRIP,
        TRANSFER,
        READY_SCORE,
        SCORING
    }

    boolean sampleMode = true, specimenMode = false;
    boolean intakeTransfer_Confirm = false, scoreTransfer_Confirm = false;
    boolean intake = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        State state = State.INIT;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        waitForStart();
        while (opModeIsActive()) {
            intake = robot.clawIntake.getPosition() == 0;
            intakeTransfer_Confirm = robot.clawIntake.getPosition() <= 0.05 && robot.intake.atTransfer;
            scoreTransfer_Confirm = robot.clawScoring.getPosition() <= 0.05 && robot.scoring.arm.atTransfer;
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

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
                        state = State.INTAKE;
                        timer1.reset();
                    }
                    break;

                case INTAKE:
                    //robot.linearSlider.setExtended();
                    robot.intake.clawOpen();
                    robot.intake.setIntake();

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
                    robot.intake.clawClose();

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.TRANSFER;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = State.INTAKE;
                        timer1.reset();
                    }
                    break;

                 case TRANSFER:
                    if (sampleMode) {
                        if (timer1.milliseconds() > 1000 && intake) {
                            robot.intake.clawClose();
                            //robot.linearSlider.retract();
                            robot.intake.setTransferSample();
                            robot.scoring.arm.setTransfer();
                        } else {
                            robot.intake.clawClose();
                        }
                    }

                    if (specimenMode) {
                        if (timer1.milliseconds() > 1000 && intake) {
                            robot.intake.clawClose();
                            //robot.linearSlider.retract();
                            robot.intake.setTransferSpecimen();
                            robot.scoring.arm.setTransfer();
                        } else {
                            robot.intake.clawClose();
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
                    if (sampleMode) {
                        if (timer1.milliseconds() > 1000) {
                            robot.scoring.clawClose();
                            robot.intake.clawOpen();
                            if (timer1.milliseconds() > 1500) robot.scoring.arm.setScoring();
                        } else {
                            robot.intake.setTransferSample();
                            robot.intake.clawClose();
                            robot.scoring.clawOpen();
                            robot.scoring.arm.setTransfer();
                        }
                    }

                    if (specimenMode) {
                        if (timer1.milliseconds() > 1000) {
                            robot.scoring.clawClose();
                            robot.intake.clawOpen();
                            if (timer1.milliseconds() > 1500) robot.scoring.arm.setScoring();
                        } else {
                            robot.intake.setTransferSpecimen();
                            robot.intake.clawClose();
                            robot.scoring.clawOpen();
                            robot.scoring.arm.setTransfer();
                        }
                    }
                    break;
            }

            telemetry.addData("state", state);
            telemetry.addData("Sample mode", sampleMode);
            telemetry.addData("Specimen mode", specimenMode);
            telemetry.update();
        }
    }
}
