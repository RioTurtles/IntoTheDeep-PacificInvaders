package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MultipleMotorSystem {
    public DcMotorEx[] motors;

    public MultipleMotorSystem(DcMotorEx... motors) {
        this.motors = motors;
    }

    public void reset() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setPosition(int position) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(position);
            motor.setPower(1);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public int getCurrentPosition() {
        return motors[0].getCurrentPosition();
    }

    public void setPower(double power) {
        for (DcMotorEx motor : motors) motor.setPower(power);
    }

    public double getPower(double power) {
        return motors[0].getPower();
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior k) {
        for (DcMotorEx motor : motors) motor.setZeroPowerBehavior(k);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehaviour() {
        return motors[0].getZeroPowerBehavior();
    }

    public void setPositionPIDCoefficients(double kP) {
        for (DcMotorEx motor : motors) motor.setPositionPIDFCoefficients(kP);
    }

    public void extend(int rate) {setPosition(motors[0].getTargetPosition() + rate);}
    public void extend() {extend(5);}
}
