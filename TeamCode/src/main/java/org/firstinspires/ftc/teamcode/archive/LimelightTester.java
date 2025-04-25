package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;

import org.firstinspires.ftc.teamcode.archive.LimelightTestHardware.LimelightPipeline;

@Disabled
@TeleOp
public class LimelightTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LimelightTestHardware robot = new LimelightTestHardware(hardwareMap);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad(), lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad(), lastOperator = new Gamepad();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();


        PIDController headingController = new PIDController(0.55, 0.001, 0);
        PIDController limelightX = new PIDController(0.55, 0.001, 0);
        PIDController limelightY = new PIDController(0.55, 0.001, 0);

        Double autoAlignTarget;
        double vertical, horizontal, pivot, heading;
        double armTargetOffset = 0;
        int sliderTargetOffset = 0;
        boolean returning = false, chambered = false;

        waitForStart();
        robot.startLimelight(LimelightPipeline.NEURAL_DETECTOR);
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            vertical = gamepad.left_stick_y; horizontal = -gamepad.left_stick_x;
            pivot = -gamepad.right_stick_x; heading = robot.getIMUYaw();

            if (state == State.INIT) {
                robot.arm.setPower(0);

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.clawOpen();
                    state = State.INTAKE;
                    timer1.reset();
                }
            }

            else if (state == State.INTAKE) {
                returning = false;
                robot.arm.setPower(0);
                robot.slider.setPower(1);

                if (timer1.milliseconds() > 150) robot.retractSlider();

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    robot.clawOpen();
                    state = State.INTAKE_EXTEND;
                    robot.extendSlider();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.switchLimelightPipeline(LimelightPipeline.NEURAL_DETECTOR);
                    state = State.INTAKE_LIMELIGHT_NEURAL;
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
                }

                if (gamepad.circle) robot.clawCloseRare();

                if (gamepad.triangle && !lastGamepad.triangle) {
                    robot.clawClose();
                    robot.retractSlider();
                    state = State.TRANSFER_ARM;
                }
            }

            else if (state == State.INTAKE_EXTEND) {
                returning = false;
                robot.arm.setPower(0);

                final int LIMIT = 975;
                final int CONTROL = 100;
                final double SPEED = 0.8;

                // While holding -> keep going
                if (gamepad.right_trigger > 0 && robot.getSlider() < (LIMIT - 50)) {
                    robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (robot.getSlider() > (LIMIT - CONTROL)) robot.slider.setPower(
                            SPEED * Math.pow(
                                    (double) Math.abs(robot.getSlider() - LIMIT) / LIMIT, 4)
                    );
                    else robot.slider.setPower(SPEED);
                } else if (gamepad.left_trigger > 0) {
                    if (robot.getSlider() < 300) {  // Go back to INTAKE state
                        robot.slider.setPower(-1);
                        robot.setSlider(0);
                        state = State.INTAKE;
                        timer1.reset();
                    } else if (robot.getSlider() > 0) {
                        robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.slider.setPower(-SPEED);
                    }
                } else {
                    robot.slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    robot.slider.setTargetPosition(robot.slider.getCurrentPosition());
                    robot.slider.setPower(1);
                    robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                // Left bumper -> go back
                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    robot.clawClose();
                    timer1.reset();
                    state = State.INTAKE;
                    // Slider retracts in the INTAKE state.
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
                }

                if (gamepad.circle) robot.clawCloseRare();

                if (gamepad.triangle && !lastGamepad.triangle) {
                    robot.clawClose();
                    state = State.TRANSFER_EXTEND;
                    timer1.reset();
                    // Arm lifts in the next state.
                }
            }

            else if (state == State.INTAKE_LIMELIGHT_NEURAL) {
                LLResult result = robot.getLimelightResults();
                if (result.isValid()) {
                    double x = result.getTx() / 640;
                    double y = result.getTy() / 320;
                    horizontal = limelightX.calculate(0, x, loopTime.seconds());
                    vertical = limelightY.calculate(0, y, loopTime.seconds());

                    if (horizontal <= 0.005 && vertical <= 0.005) {
                        horizontal = 0;
                        vertical = 0;

                        switch (result.getDetectorResults().get(0).getClassName()) {
                            case "yellow": robot.switchLimelightPipeline(1); break;
                            case "red": robot.switchLimelightPipeline(2); break;
                            case "blue": robot.switchLimelightPipeline(3); break;
                        }
                        state = State.INTAKE_LIMELIGHT_COLOUR;
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    state = State.INTAKE;
                    timer1.reset();
                }
            }

            else if (state == State.INTAKE_LIMELIGHT_COLOUR) {
                double orientation = robot.getLimelightSampleOrientation();
                pivot = headingController.calculate(orientation, Math.toDegrees(robot.getIMUYaw()),
                        loopTime.seconds());
                if (headingController.lastError <= 4) {
                    robot.clawClose();
                    state = State.TRANSFER_ARM;
                }
            }

            else if (state == State.TRANSFER_EXTEND) {
                if (timer1.milliseconds() > 150) {
                    robot.setArm(20);
                    if (robot.getArmError() <= 3) state = State.TRANSFER_ARM;
                }
            }

            else if (state == State.TRANSFER_ARM) {
                robot.arm.setPower(1);
                robot.setSlider(0);

                if (!returning) {  // Forward
                    if (robot.sliderInPosition(15)) {
                        if (robot.scoringMode == ScoringMode.BASKET)
                            robot.setArm(LimelightTestHardware.BASKET_ANGLE + armTargetOffset);
                        else robot.setArm(LimelightTestHardware.CHAMBER_ANGLE + armTargetOffset);

                        if (gamepad.right_bumper && !lastGamepad.right_bumper)
                            state = State.TRANSFER_SLIDER;
                    }

                    if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0))
                        state = State.TRANSFER_SLIDER;
                } else {  // Reverse
                    robot.setArm(0);
                    if (robot.getArmError() <= 2
                            || (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0))) {
                        if (robot.scoringMode == ScoringMode.BASKET) robot.clawOpen();
                        state = State.INTAKE;
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
                if (gamepad.right_bumper && !lastGamepad.right_bumper) returning = false;
            }

            else if (state == State.TRANSFER_SLIDER) {
                robot.arm.setPower(1);
                if (!returning) {  // Forward
                    if (robot.scoringMode == ScoringMode.BASKET) {
                        if (robot.scoringHeight == ScoringHeight.HIGH)
                            robot.setSlider(LimelightTestHardware.SLIDER_HIGH + sliderTargetOffset);
                        else if (robot.scoringHeight == ScoringHeight.LOW)
                            robot.setSlider(LimelightTestHardware.SLIDER_LOW + sliderTargetOffset);
                    } else
                        robot.setSlider(LimelightTestHardware.SLIDER_CHAMBER + sliderTargetOffset);


                    if (robot.sliderInPosition(5)) {
                        switch (robot.scoringMode) {
                            case BASKET: state = State.SCORING_BASKET; break;
                            case CHAMBER: state = State.SCORING_CHAMBER; break;
                        }
                    }
                } else {  // Reverse
                    robot.setSlider(0);
                    if (robot.sliderInPosition(5)) state = State.TRANSFER_ARM;
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    switch (robot.scoringMode) {
                        case BASKET: state = State.SCORING_BASKET; break;
                        case CHAMBER: state = State.SCORING_CHAMBER; break;
                    }
                }

                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0))
                    state = State.TRANSFER_ARM;

                if (gamepad.left_bumper && !lastGamepad.left_bumper) returning = true;
                if (gamepad.right_bumper && !lastGamepad.right_bumper) returning = false;

                chambered = false;
            }

            else if (state == State.SCORING_BASKET) {
                robot.setArm(LimelightTestHardware.BASKET_ANGLE + armTargetOffset);

                switch (robot.scoringHeight) {
                    case HIGH:
                        robot.setSlider(LimelightTestHardware.SLIDER_HIGH + sliderTargetOffset);
                        break;
                    case LOW:
                        robot.setSlider(LimelightTestHardware.SLIDER_LOW + sliderTargetOffset);
                        break;
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.clawClosed) robot.clawOpen(); else robot.clawClose();
                }

                if ((gamepad.left_bumper && !lastGamepad.left_bumper
                        || (gamepad.right_bumper && !lastGamepad.right_bumper))) {
                    returning = true;
                    state = State.TRANSFER_SLIDER;
                }

                switch (robot.scoringHeight) {
                    case HIGH:
                        robot.setSlider(LimelightTestHardware.SLIDER_HIGH + sliderTargetOffset);
                        break;
                    case LOW:
                        robot.setSlider(LimelightTestHardware.SLIDER_LOW + sliderTargetOffset);
                        break;
                }

                if ((gamepad.options && !lastGamepad.options)
                        || (operator.options && !lastOperator.options)) {
                    state = State.SCORING_CHAMBER;
                    timer1.reset();
                }
            }

            else if (state == State.SCORING_CHAMBER) {
                if (!chambered) {
                    robot.setArm(LimelightTestHardware.CHAMBER_ANGLE + armTargetOffset);
                    robot.setSlider(LimelightTestHardware.SLIDER_CHAMBER + sliderTargetOffset);
                    robot.clawClose();
                } else {
                    robot.setArm(LimelightTestHardware.CHAMBERED_ANGLE + armTargetOffset);
                    robot.setSlider(LimelightTestHardware.SLIDER_CHAMBERED + sliderTargetOffset);

                    if (robot.sliderInPosition(10) || gamepad.left_trigger > 0)
                        robot.clawOpen();
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0))
                    chambered = !chambered;

                if ((gamepad.left_bumper && !lastGamepad.left_bumper
                        || (gamepad.right_bumper && !lastGamepad.right_bumper))) {
                    returning = true;
                    state = State.TRANSFER_SLIDER;
                }

                if ((gamepad.share && !lastGamepad.share)
                        || (operator.share && !lastOperator.share)) {
                    state = State.SCORING_BASKET;
                    timer1.reset();
                }
            }

            else if (state == State.ASCENT) {
                robot.setArm(100);
                if (robot.getArmError() <= 10 || operator.right_bumper) robot.setSlider(200);
            }

            if (operator.triangle) autoAlignTarget = 180.0;  // Forward
            else if (operator.square) autoAlignTarget = -45.0;  // Basket alignment
            else if (operator.circle) autoAlignTarget = 90.0;  // Submersible alignment
            else if (operator.cross) autoAlignTarget = 0.0;  // Backwards
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

                pivot = headingController.calculate(0, pivot, loopTime.seconds());

                heading = 0;
                vertical *= 0.8;
                horizontal *= 0.8;
            } else headingController.reset();

            // Mode & height switching
            if (gamepad.share) robot.scoringMode = ScoringMode.BASKET;
            if (gamepad.options) robot.scoringMode = ScoringMode.CHAMBER;
            if (operator.options) robot.scoringHeight = ScoringHeight.HIGH;
            if (operator.share) robot.scoringHeight = ScoringHeight.LOW;

            // Emergency resets
            if (operator.dpad_right) robot.powerResetSlider();
            if (operator.dpad_left) robot.powerResetSliderR();
            if (!operator.dpad_right && lastOperator.dpad_right) robot.resetSlider();
            if (!operator.dpad_left && lastOperator.dpad_left) robot.resetSlider();

            if (operator.dpad_down) {robot.powerResetArm(); robot.clawOpen();}
            if (operator.dpad_up) robot.powerResetArmR();
            if ((!operator.dpad_down && lastOperator.dpad_down)
                    || (!operator.dpad_up && lastOperator.dpad_up)
                    || operator.touchpad) {
                robot.resetArm();
                robot.resetSlider();
                armTargetOffset = 0;
                sliderTargetOffset = 0;
            }

            if (operator.right_bumper && !lastOperator.right_bumper) armTargetOffset += 2;
            if (operator.left_bumper && !lastOperator.left_bumper) armTargetOffset -= 2;

            if (operator.right_trigger > 0 && !(lastOperator.right_trigger > 0))
                sliderTargetOffset += 50;
            if (operator.left_trigger > 0 && !(lastOperator.left_trigger > 0))
                sliderTargetOffset -= 50;

            if (operator.left_stick_button) {
                state = State.INIT;
                robot.arm.setPower(0);
                robot.slider.setPower(0);
                robot.clawOpen();
            }

            if (operator.right_stick_button) {
                state = State.ASCENT;
                robot.setSlider(0);
            }

            if (gamepad.touchpad) robot.imu.resetYaw();
            robot.drivetrain.remote(vertical, horizontal, pivot, heading);

            if (returning) telemetry.addData("State", state + " | *");
            else telemetry.addData("State", state);
            telemetry.addData("Scoring", robot.getScoringState());
            telemetry.addData("Arm offset", armTargetOffset);
            telemetry.addData("Slider offset", sliderTargetOffset);
            telemetry.addLine();
            telemetry.addData("Claw", robot.getClawString());
            telemetry.addData("Slider", robot.getSlider());
            telemetry.addData("Arm target", robot.armTargetAngle);
            telemetry.addData("Arm current angle", robot.getArmAngle());
            telemetry.addData("Arm error", robot.getArmError());
            telemetry.addData("Slider in pos?", robot.sliderInPosition(5));
            telemetry.addData("Slider target encoder", robot.slider.getTargetPosition());
            telemetry.addData("Slider current encoder", robot.slider.getCurrentPosition());
            telemetry.update();
            loopTime.reset();
        }
    }

    enum State {
        INIT,
        INTAKE,
        INTAKE_EXTEND,
        INTAKE_LIMELIGHT_NEURAL,
        INTAKE_LIMELIGHT_COLOUR,
        TRANSFER_EXTEND,
        TRANSFER_ARM,
        TRANSFER_SLIDER,
        SCORING_BASKET,
        SCORING_CHAMBER,
        ASCENT
    }
}
