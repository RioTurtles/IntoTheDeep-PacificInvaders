package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    double max, sin, cos, theta, power, vertical, horizontal, pivot, heading;
    double powerFL, powerFR, powerBL, powerBR;

    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    /**
     * Classic drivetrain movement method - self explanatory.
     *
     * @param vertical   Gamepad's vertical axis (y).
     * @param horizontal Gamepad's horizontal axis (x).
     * @param pivot      Gamepad's rotational axis (<code>right_stick_x</code>).
     * @param heading    Robot's heading.
     */
    public void remote(double vertical, double horizontal, double pivot, double heading) {
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.pivot = pivot;
        this.heading = heading;

        theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
        power = Math.hypot(horizontal, vertical);

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        powerFL = power * (cos / max) + pivot;
        powerFR = power * sin / max - pivot;
        powerBL = power * -(sin / max) - pivot;
        powerBR = power * -(cos / max) + pivot;

        frontLeft.setPower(-powerFL);
        frontRight.setPower(-powerFR);
        backLeft.setPower(powerBL);
        backRight.setPower(powerBR);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}