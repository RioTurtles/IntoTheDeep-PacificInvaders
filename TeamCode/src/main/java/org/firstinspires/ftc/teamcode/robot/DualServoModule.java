package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class DualServoModule {
    ServoImplEx left, right;
    private double offsetLeft, offsetRight;

    public DualServoModule(ServoImplEx left, ServoImplEx right) {
        this.left = left;
        this.right = right;
    }

    public DualServoModule(ServoImplEx left, ServoImplEx right, double dL, double dR) {
        this.left = left;
        this.right = right;
        offsetLeft = dL;
        offsetRight = dR;
    }

    public void setPosition(double position) {
        left.setPosition(position + offsetLeft);
        right.setPosition(position + offsetRight);
    }
}
