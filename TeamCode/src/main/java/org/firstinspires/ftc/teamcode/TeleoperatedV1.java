package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        State state = State.INIT;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();

        PIDController pidX = new PIDController(0.5, 0.001, 0);
        PIDController pidY = new PIDController(0.5, 0.001, 0);
        PIDController pidR = new PIDController(0.55, 0.001, 0);

        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        Double autoAlignTarget;
        double vertical, horizontal, pivot, heading;
        boolean returning = false, chambered = false;

        waitForStart();
        loopTime.reset();
        robot.limelight.start(4);
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            vertical = -gamepad.left_stick_y; horizontal = gamepad.left_stick_x;
            pivot = gamepad.right_stick_x; heading = robot.getHeadingRad();

            if (state == State.INIT) {
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.intake.setIntakeRaised();
                    robot.intake.setOrientation(0);
                    state = State.INTAKE_SPECIMEN;
                    robot.intake.clawOpen();
                }
            }

            else if (state == State.INTAKE_SAMPLE) {
                returning = false;
                robot.scoring.arm.setTransfer();

                if (timer1.milliseconds() > 150) robot.intake.clawOpen();

                if (gamepad.right_bumper) {
                    if (robot.intake.clawOpen) robot.intake.clawClose();
                    else robot.intake.clawOpen();
                }

                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    state = State.INTAKE_ALIGN;
                    pidX.reset();
                    pidY.reset();
                }

                if (gamepad.right_trigger > 0) robot.linearSlider.extend(6);
                if (gamepad.cross) robot.linearSlider.extend(-6);

                if (gamepad.triangle) {
                    robot.intake.clawClose();
                    state = State.TRANSFER;
                    timer1.reset();
                }

                if (operator.share) {
                    robot.intake.setOrientation(0);
                    robot.intake.setIntake();
                    state = State.INTAKE_SPECIMEN;
                    robot.mode = Project1Hardware.Mode.SPECIMEN;
                }
            }

            else if (state == State.INTAKE_ALIGN) {
                if (!(gamepad.left_trigger > 0)) {
                    robot.intake.clawClose();
                    state = State.TRANSFER;
                    timer1.reset();
                }

                if (gamepad.left_bumper) state = State.INTAKE_SAMPLE;

                // Alignment using limelight.
                @Nullable Double[] data = robot.limelight.getData();
                if (!Objects.isNull(data)) {
                    assert data != null;
                    double tx = data[0]; double ty = data[1]; double orientation = data[2];
                    horizontal = pidX.calculate(0, tx, loopTime.seconds());
                    vertical = pidY.calculate(0, ty, loopTime.seconds());
                    robot.intake.setOrientation(orientation);
                } else {
                    pidX.reset();
                    pidY.reset();
                }
            }

            else if (state == State.INTAKE_SPECIMEN) {
                returning = false;
                robot.intake.setIntake();
                robot.scoring.arm.setTransfer();

                if (timer1.milliseconds() > 150) robot.intake.clawOpen();

                if (gamepad.right_bumper) {
                    if (robot.intake.clawOpen) robot.intake.clawClose();
                    else robot.intake.clawOpen();
                }

                if (gamepad.right_trigger > 0) robot.linearSlider.extend(6);
                if (gamepad.left_trigger > 0) robot.linearSlider.extend(-6);

                if (gamepad.triangle) {
                    robot.intake.clawClose();
                    state = State.TRANSFER;
                    timer1.reset();
                }

                if (operator.share) {
                    robot.intake.setOrientation(0);
                    robot.intake.setIntakeRaised();
                    state = State.INTAKE_SAMPLE;
                    robot.mode = Project1Hardware.Mode.SAMPLE;
                }
            }

            else if (state == State.TRANSFER) {
                robot.scoring.arm.setTransfer();

                if (timer1.milliseconds() > 450) {
                    robot.scoring.arm.setScoring();
                    if (gamepad.right_bumper && !lastGamepad.right_bumper) state = State.TRANSITION;
                }
                else if (timer1.milliseconds() > 400) robot.intake.clawOpen();
                else if (timer1.milliseconds() > 350) robot.scoring.clawClose();
                else if (timer1.milliseconds() > 150) robot.intake.setTransfer();

                if (gamepad.left_bumper && !lastGamepad.left_bumper || returning) {
                    robot.scoring.arm.setTransfer();
                    robot.scoring.clawOpen();
                    robot.intake.setIntake();
                    if (robot.mode == Project1Hardware.Mode.SAMPLE) state = State.INTAKE_SAMPLE;
                    else state = State.INTAKE_SPECIMEN;
                    timer1.reset();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
            }

            else if (state == State.TRANSITION) {
                if (!returning) {
                    if (robot.mode == Project1Hardware.Mode.SAMPLE) {
                        if (robot.height == Project1Hardware.Height.HIGH)
                            robot.verticalSlider.setHighBasket();
                        else robot.verticalSlider.setLowBasket();

                        if (robot.verticalSlider.inPosition()
                                || (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)))
                            state = State.SCORING_BASKET;

                        robot.scoring.puncher.setExtended();
                    } else {
                        if (robot.height == Project1Hardware.Height.HIGH)
                            robot.verticalSlider.setHighChamber();
                        else robot.verticalSlider.setLowChamber();

                        if (robot.verticalSlider.inPosition()
                                || (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)))
                            state = State.SCORING_CHAMBER;

                        robot.scoring.puncher.setAim();
                    }

                } else {
                    robot.verticalSlider.retract();

                    if (robot.verticalSlider.inPosition()
                            && gamepad.left_bumper && !(lastGamepad.left_bumper)
                            || (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)))
                        state = State.TRANSFER;
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
            }

            else if (state == State.SCORING_BASKET) {
                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.scoring.clawOpen) robot.scoring.clawClose();
                    else robot.scoring.clawOpen();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    returning = true;
                    state = State.TRANSITION;
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    returning = false;
                    chambered = false;
                    state = State.RETURN;
                    timer1.reset();
                }

                if (operator.share && !lastOperator.share) {
                    robot.mode = Project1Hardware.Mode.SPECIMEN;
                    returning = false;
                    state = State.TRANSITION;
                }

                robot.scoring.alignTurret(robot.getHeadingDeg(), 135);
            }

            else if (state == State.SCORING_CHAMBER) {
                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) chambered = true;
                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) chambered = false;

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    returning = true;
                    state = State.TRANSITION;
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    returning = false;
                    chambered = false;
                    state = State.RETURN;
                    timer1.reset();
                }

                if (chambered) {
                    robot.scoring.puncher.setExtended();
                    if (timer1.milliseconds() > 300) robot.scoring.clawOpen();
                } else {
                    robot.scoring.puncher.setAim();
                    timer1.reset();
                }

                if (operator.options && !lastOperator.options) {
                    robot.mode = Project1Hardware.Mode.SAMPLE;
                    returning = false;
                    state = State.TRANSITION;
                }

                robot.scoring.alignTurret(robot.getHeadingDeg(), 0);
            }

            else if (state == State.RETURN) {
                robot.scoring.setTurret(0);
                robot.scoring.clawOpen();

                if (timer1.milliseconds() > 800) {
                    robot.verticalSlider.retract();
                    if (robot.verticalSlider.inPosition()) {
                        robot.verticalSlider.powerOff();
                        if (robot.mode == Project1Hardware.Mode.SAMPLE) state = State.INTAKE_SAMPLE;
                        else state = State.INTAKE_SPECIMEN;
                        timer1.reset();
                    }
                }
                else if (timer1.milliseconds() > 300) robot.scoring.puncher.setRetracted();
            }

//            if (operator.triangle) autoAlignTarget = 180.0;  // Forward
//            else if (operator.square) autoAlignTarget = -45.0;  // Basket alignment
//            else if (operator.circle) autoAlignTarget = 90.0;  // Submersible alignment
            if (operator.cross) autoAlignTarget = 45.0;  // Specimen intake from observation
            else autoAlignTarget = null;

            if (Objects.nonNull(autoAlignTarget)) {
                assert autoAlignTarget != null;

                double current = Math.toDegrees(heading);
                double smallerAngle = Math.min(
                        Math.abs(current - autoAlignTarget),
                        360 - Math.abs(current - autoAlignTarget)
                );

                double resultant1 = current - smallerAngle;
                if (resultant1 <= -180) resultant1 += 360;
                double resultant2 = current + smallerAngle;
                if (resultant2 > 180) resultant2 -= 360;

                if (resultant1 == autoAlignTarget) pivot = Math.toRadians(smallerAngle);
                else if (resultant2 == autoAlignTarget) pivot = Math.toRadians(-smallerAngle);

                pivot = pidR.calculate(0, pivot, loopTime.seconds());

                heading = 0;
                vertical *= 0.8;
                horizontal *= 0.8;
            } else pidR.reset();

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);
            telemetry.addData("State", state);
            telemetry.addData("Mode", robot.mode + "|" + robot.height);
            telemetry.update();
            loopTime.reset();
        }
    }

    enum State {
        INIT,
        INTAKE_SAMPLE,
        INTAKE_ALIGN,
        INTAKE_SPECIMEN,
        TRANSFER,
        TRANSITION,
        SCORING_BASKET,
        SCORING_CHAMBER,
        RETURN,
    }
}
