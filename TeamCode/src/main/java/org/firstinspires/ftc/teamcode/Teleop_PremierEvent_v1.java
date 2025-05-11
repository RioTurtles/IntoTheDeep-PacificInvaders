package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

@TeleOp (name = "Teleop_EPE_v1")
public class Teleop_PremierEvent_v1 extends LinearOpMode {
    public enum States {
        INIT,
        INTAKE_ALIGN,
        GROUND_INTAKE,
        GRIP,
        TRANSFER,
        READY_SCORE,
        SCORING,
        RETURN_TO_INIT
    }
    double vertical, horizontal, pivot, heading;
    boolean sampleMode = false, specimenMode = true;
    boolean highMode = true, lowMode = false;
    boolean intakeClawClose = false, turretIsInPosition = false, puncherIsInPosition = false;
    boolean returning = false, chambered = false;
    double vertPos, horPos;
    double degrees = 0, turretTarget = 0;
    int linearSliderPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        States state = States.INIT;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();

        PIDController pidX = new PIDController(0.5, 0.001, 0);
        PIDController pidY = new PIDController(0.5, 0.001, 0);
        PIDController pidR = new PIDController(0.55, 0.001, 0);

        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        waitForStart();
        loopTime.reset();
        while (opModeIsActive()) {
            vertical = -gamepad.left_stick_y; horizontal = gamepad.left_stick_x;
            pivot = gamepad.right_stick_x; heading = robot.getHeadingRad();

            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);

            vertPos = robot.verticalSlider.getCurrentPosition();
            horPos = robot.linearSlider.getCurrentPosition();
            intakeClawClose = robot.clawIntake.getPosition() == 0;
            turretIsInPosition = robot.scoring.turret.getPosition() >= 88
                    && robot.scoring.turret.getPosition() <= 92;
            puncherIsInPosition = robot.scoring.puncher.getPosition() >= 0.19
                    && robot.scoring.puncher.getPosition() <= 0.3;
            turretTarget = robot.scoring.turret.getPosition();

            if (gamepad.touchpad) robot.resetIMU();

            if (operator.dpad_left && !lastOperator.dpad_left) {
                sampleMode = true; specimenMode = false;
            }
            if (operator.dpad_right && !lastOperator.dpad_right) {
                specimenMode = true; sampleMode = false;
            }
            if (operator.dpad_up && !lastOperator.dpad_up) {
                highMode = true; lowMode = false;
            }
            if (operator.dpad_down && !lastOperator.dpad_down) {
                lowMode = true; highMode = false;
            }

            switch (state) {
                case INIT:
                    robot.verticalSlider.retract();
                    robot.linearSlider.retract();

                    robot.scoring.setTurret(0);
                    robot.scoring.puncher.setRetracted();
                    robot.scoring.arm.setSampleTransfer();
                    robot.scoring.clawOpen();

                    robot.intake.setSampleIntake();
                    robot.intake.clawClose();

                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        state = States.GROUND_INTAKE;
                        timer1.reset();
                    }
                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = States.INTAKE_ALIGN;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = States.INIT;
                        timer1.reset();
                    }
                    break;

                case GROUND_INTAKE:
                    robot.intake.clawOpen();
                    if (gamepad.cross && !lastGamepad.cross && sampleMode) {
                        robot.intake.setSampleIntake();
                    } else if (gamepad.triangle && !lastGamepad.triangle && sampleMode) {
                        robot.intake.setIntakeRaised();
                    }
                    if (specimenMode) robot.intake.setSpecimenIntake();

                    if (gamepad.circle && !lastGamepad.circle) {
                        robot.intake.setOrientation(degrees += 45);
                    }
                    if (gamepad.square && !lastGamepad.square) {
                        robot.intake.setOrientation(degrees -= 45);
                    }

                    if (operator.triangle && !lastOperator.triangle) robot.linearSlider.set(1600);
                    if (operator.cross && !lastOperator.cross) robot.linearSlider.set(0);

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


                    if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                        state = States.INTAKE_ALIGN;
                        timer1.reset();
                    }
                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = States.GRIP;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = States.INIT;
                        timer1.reset();
                    }
                    break;

                case INTAKE_ALIGN:
                    robot.intake.clawOpen();
                    if (gamepad.cross && !lastGamepad.cross && sampleMode) {
                        robot.intake.setSampleIntake();
                    } else if (gamepad.triangle && !lastGamepad.triangle && sampleMode) {
                        robot.intake.setIntakeRaised();
                    }
                    if (specimenMode) robot.intake.setSpecimenIntake();

                    // Alignment using limelight.
                    @Nullable Double[] data = robot.limelight.getData();
                    if (!Objects.isNull(data)) {
                        assert data != null;
                        double tx = data[0]; double ty = data[1]; double orientation = data[2];
                        horizontal = pidX.calculate(0, tx, loopTime.seconds());
                        robot.linearSlider.setPosition((int) Math.round(pidY.calculate(
                                0, ty, loopTime.seconds()
                        )));
                        robot.intake.setOrientation(orientation);
                    } else {
                        pidX.reset();
                        pidY.reset();
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper || intakeClawClose) {
                        state = States.GRIP;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = States.INIT;
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
                        state = States.TRANSFER;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = States.INTAKE_ALIGN;
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
                        state = States.READY_SCORE;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = States.GRIP;
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
                        state = States.SCORING;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = States.TRANSFER;
                        timer1.reset();
                    }
                    break;

                case SCORING:
                    if (sampleMode) {
                        robot.scoring.puncher.setExtended();
                        // robot.scoring.alignTurret(robot.getHeadingDeg(), 225);
                        robot.scoring.arm.setSpecimenScoring();

                        if (highMode) {
                            robot.verticalSlider.setHighBasket();
                        } else if (lowMode) robot.verticalSlider.setLowBasket();

                        if (gamepad.right_trigger > 0) robot.scoring.clawOpen();
                        else robot.scoring.clawClose();
                    }

                    if (specimenMode) {
                        // TODO: Tune this
                        /*if (robot.scoring.puncher.getPosition() >= 0.19) {
                            robot.scoring.alignTurret(robot.getHeadingDeg(), 90);
                        }*/
                        if (turretIsInPosition) {
                            robot.scoring.puncher.setExtended();
                        }
                        robot.scoring.arm.setSpecimenScoring();

                        if (timer1.milliseconds() > 800 && robot.scoring.puncher.extendedPuncher) {
                            robot.intake.clawOpen();
                        } else robot.intake.clawClose();
                    }

                    if (gamepad.circle && !lastGamepad.circle) {
                        robot.scoring.setTurret(turretTarget + 5);
                    }
                    if (gamepad.square && !lastGamepad.square) {
                        robot.scoring.setTurret(turretTarget - 5);
                    }

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = States.RETURN_TO_INIT;
                        timer1.reset();
                    }
                    if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                        state = States.READY_SCORE;
                        timer1.reset();
                    }
                    break;

                case RETURN_TO_INIT:
                    robot.verticalSlider.retract();
                    robot.scoring.setTurret(0);
                    if (timer1.milliseconds() > 1000) {
                        robot.scoring.arm.setSampleTransfer();
                    } else {
                        if (turretIsInPosition) robot.scoring.puncher.setRetracted();
                        robot.scoring.clawOpen();
                    }

                    if (vertPos < 500 || timer1.milliseconds() > 1000) {
                        state = States.INIT;
                        timer1.reset();
                    }
            }
            robot.drivetrain.remote(vertical, -horizontal, -pivot, heading);
            loopTime.reset();
        }
    }
}
