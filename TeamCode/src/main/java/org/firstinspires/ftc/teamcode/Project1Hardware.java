package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.DifferentialModule;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.DualServoModule;
import org.firstinspires.ftc.teamcode.robot.MultipleMotorSystem;

public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx linearLeft, linearRight, verticalLeft, verticalRight;
    ServoImplEx intakeLeft, intakeRight, clawIntake;
    ServoImplEx puncherLeft, puncherRight, armLeft, armRight, turret, clawScoring;
    IMU imu;

    Drivetrain drivetrain;
    LinearSlider linearSlider;
    VerticalSlider verticalSlider;
    Intake intake;
    Scoring scoring;

    public Project1Hardware(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        linearLeft = hardwareMap.get(DcMotorEx.class, "linearLeft");
        linearRight = hardwareMap.get(DcMotorEx.class, "linearRight");
        verticalLeft = hardwareMap.get(DcMotorEx.class, "verticalLeft");
        verticalRight = hardwareMap.get(DcMotorEx.class, "verticalRight");
        intakeLeft = hardwareMap.get(ServoImplEx.class, "intakeLeft");
        intakeRight = hardwareMap.get(ServoImplEx.class, "intakeRight");
        clawIntake = hardwareMap.get(ServoImplEx.class, "clawIntake");
        puncherLeft = hardwareMap.get(ServoImplEx.class, "puncherLeft");
        puncherRight = hardwareMap.get(ServoImplEx.class, "puncherRight");
        armLeft = hardwareMap.get(ServoImplEx.class, "armLeft");
        armRight = hardwareMap.get(ServoImplEx.class, "armRight");
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        clawScoring = hardwareMap.get(ServoImplEx.class, "clawScoring");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        drivetrain = new Drivetrain(frontLeft, frontRight, backLeft, backRight);
        linearSlider = new LinearSlider(linearLeft, linearRight);
        verticalSlider = new VerticalSlider(verticalLeft, verticalRight);
        intake = new Intake(intakeLeft, intakeRight, clawIntake);
        scoring = new Scoring(armLeft, armRight, puncherLeft, puncherRight, turret, clawScoring);
    }

    public static class LinearSlider extends MultipleMotorSystem {
        public LinearSlider(DcMotorEx... motors) {super(motors);}
        public void setExtended() {setPosition(500);}
        public void retract() {setPosition(0);}
    }

    public static class VerticalSlider extends MultipleMotorSystem {
        public VerticalSlider(DcMotorEx... motors) {super(motors);}
        public void setHighBasket() {setPosition(3000);}
        public void setLowBasket() {setPosition(1500);}
        public void setHighChamber() {setPosition(1800);}
        public void setLowChamber() {setPosition(900);}
        public void retract() {setPosition(0);}
        public void powerOff() {setPower(0);}
    }

    public static class Intake extends DifferentialModule {
        ServoImplEx claw;

        public Intake(ServoImplEx left, ServoImplEx right, ServoImplEx claw) {
            super(left, right);
            this.claw = claw;
        }

        public void setIntake() {setPitch(HALF);}
        public void setTransfer() {setPosition(0.5, 0);}

        public void clawOpen() {claw.setPosition(0);}
        public void clawClose() {claw.setPosition(1);}
    }

    public static class Scoring {
        Arm arm;
        Puncher puncher;
        ServoImplEx turret, claw;
        private final static double TURRET_RANGE = 255;

        public Scoring(
                ServoImplEx armLeft, ServoImplEx armRight,
                ServoImplEx puncherLeft, ServoImplEx puncherRight,
                ServoImplEx turret, ServoImplEx claw
        ) {
            arm = new Arm(armLeft, armRight);
            puncher = new Puncher(puncherLeft, puncherRight);
            this.turret = turret;
            this.claw = claw;
        }

        public static class Arm extends DualServoModule {
            public Arm(ServoImplEx left, ServoImplEx right) {super(left, right);}
            public void setTransfer() {setPosition(0.5);}
            public void setScoring() {setPosition(1);}
        }

        public static class Puncher extends DualServoModule {
            public Puncher(ServoImplEx left, ServoImplEx right) {super(left, right);}
            public void setRetracted() {setPosition(0);}
            public void setExtended() {setPosition(1);}
        }

        public void setTurret(double angle) {
            turret.setPosition((angle + TURRET_RANGE / 2) / TURRET_RANGE);
        }

        public void clawOpen() {claw.setPosition(0);}
        public void clawClose() {claw.setPosition(1);}
    }
}
