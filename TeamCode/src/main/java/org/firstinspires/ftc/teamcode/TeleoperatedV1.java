package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        double vertical, horizontal, pivot, heading;

        waitForStart();
        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            vertical = -gamepad.left_stick_y; horizontal = gamepad.left_stick_x;
            pivot = gamepad.right_stick_x; heading = robot.getHeadingRad();

            if (state == State.INIT) {
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    robot.intake.setIntake();
                    robot.intake.setOrientation(0);
                    state = State.INTAKE_SAMPLE;
                }
            }

            else if (state == State.INTAKE_SAMPLE) {

            }

            else if (state == State.INTAKE_SPECIMEN) {
                robot.intake.setIntake();
                if (gamepad.right_trigger > 0) robot.linearSlider.extend(6);
                if (gamepad.left_trigger > 0) robot.linearSlider.extend(-6);
            }

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);
            telemetry.addData("State", state);
            telemetry.update();
        }
    }

    enum State {
        INIT,
        INTAKE_SAMPLE,
        INTAKE_SPECIMEN,
    }
}
