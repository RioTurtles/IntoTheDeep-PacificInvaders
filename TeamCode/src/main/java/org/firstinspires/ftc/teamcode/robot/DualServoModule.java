package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class DualServoModule {
    ServoImplEx left, right;
    private double offsetLeft, offsetRight;

    public DualServoModule(ServoImplEx left, ServoImplEx right) {
        this.left = left;
        this.right = right;
    }

    public DualServoModule(ServoImplEx left, ServoImplEx right, double offsetL, double offsetR) {
        this.left = left;
        this.right = right;
        offsetLeft = offsetL;
        offsetRight = offsetR;
    }

    public void setPosition(double position) {
        left.setPosition(position + offsetLeft);
        right.setPosition(position + offsetRight);
    }

    public double getPosition() {return left.getPosition() - offsetLeft;}
}
