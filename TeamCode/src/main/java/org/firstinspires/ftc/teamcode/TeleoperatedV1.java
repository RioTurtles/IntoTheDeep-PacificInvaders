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

        waitForStart();
        while (opModeIsActive()) {
        }
    }

    enum State {
        INIT,
        INTAKE,
    }
}
