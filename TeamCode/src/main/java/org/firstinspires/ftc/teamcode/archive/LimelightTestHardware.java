package org.firstinspires.ftc.teamcode.archive;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * This class represents the robot object. This is a modified version in order to avoid conflicts
 * with TeleOp hardware classes.
 */
public class LimelightTestHardware {
    MecanumDrive drivetrain;
    DcMotor frontLeft, frontRight;
    DcMotor backLeft, backRight;
    DcMotorEx slider, arm;
    ServoImplEx claw;
    IMU imu;
    Limelight3A limelight;

    ScoringMode scoringMode;
    ScoringHeight scoringHeight;
    double armTargetAngle;
    boolean clawClosed;

    public static final double BASKET_ANGLE = 77.5;
    public static final double CHAMBER_ANGLE = 66.8;
    public static final double CHAMBERED_ANGLE = 62;
    public static final int SLIDER_HIGH = 1400;
    public static final int SLIDER_LOW = 675;
    public static final int SLIDER_CHAMBER = 540;
    public static final int SLIDER_CHAMBERED = 230;

    public LimelightTestHardware(@NonNull HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        slider = hardwareMap.get(DcMotorEx.class, "slider");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        reset();

        drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.armTargetAngle = getArmAngle();
        this.scoringMode = ScoringMode.BASKET;
        this.scoringHeight = ScoringHeight.HIGH;
        this.clawClosed = false;
    }

    private void reset() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        slider.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // Regular methods.
    public void setSlider(int pos) {
        if (pos > 1375) pos = 1375;

        slider.setTargetPosition(pos + (arm.getCurrentPosition() / 19));
        slider.setPositionPIDFCoefficients(20);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setPower(1);
    }

    public int getSlider() {
        return (int) (slider.getCurrentPosition() - (arm.getCurrentPosition() / 19));
    }

    public boolean sliderInPosition(int tolerance) {
        return Math.abs(lengthToEncoderValueSlider(getSliderLength()) - slider.getTargetPosition())
                < tolerance;
    }

    public void setScoringArm(double angle) {
        arm.setTargetPosition(angleToEncoderValueArm(angle));
    }

    // Extensions
    public void extendSlider() {
        slider.setPower(1);
        setSlider(1000);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    // Retractions
    public void retractSlider() {
        slider.setPower(1);
        slider.setTargetPosition(0);
        slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private int angleToEncoderValueArm(double angle) {
        double CPR = 3895.9;
        double revolutions = angle / 360;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setArm(double angle) {
        armTargetAngle = angle;
        arm.setPower(1);
        arm.setPositionPIDFCoefficients(5);
        arm.setTargetPosition(angleToEncoderValueArm(angle));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getArmAngle() {
        double CPR = 3895.9;
        int position = arm.getCurrentPosition();
        double revolutions = position / CPR;
        return revolutions * 360;
    }

    public double getArmError() {return Math.abs(getArmAngle() - armTargetAngle);}

    public int lengthToEncoderValueSlider(double length) {
        double CPR = 145.1 * 1.4;
        double revolutions = length / 35.65 / Math.PI;
        double tmp = revolutions * CPR;
        return (int) tmp;
    }

    public void setSliderLength(double length) {
        slider.setTargetPosition(lengthToEncoderValueSlider(length));
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(1);
    }

    public double getSliderLength() {
        double CPR = 145.1 * 1.4;
        int position = slider.getCurrentPosition();
        double revolutions = position / CPR;
        return revolutions * 35.65 * Math.PI;
    }

    public void powerResetArm() {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(-0.8);
    }

    public void powerResetArmR() {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setPower(0.8);
    }

    public void powerResetSlider() {
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setPower(-0.8);
    }

    public void powerResetSliderR() {
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setPower(0.8);
    }

    public void resetArm() {
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(arm.getCurrentPosition());
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetSlider() {
        slider.setPower(0);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setTargetPosition(slider.getCurrentPosition());
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void clawOpen() {claw.setPosition(0.55); clawClosed = false;}
    public void clawCloseRare() {claw.setPosition(0.3); clawClosed = true;}
    public void clawClose() {claw.setPosition(0.16); clawClosed = true;}
    public String getClawString() {if (clawClosed) return "CLOSED"; else return "OPEN";}
    public String getScoringState() {return scoringMode + " | " + scoringHeight;}
    public double getIMUYaw() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    public void startLimelight(int pipeline) {
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public void startLimelight(LimelightPipeline pipeline) {startLimelight(pipeline.getIndex());}

    public void switchLimelightPipeline(LimelightPipeline pipeline) {
        limelight.pipelineSwitch(pipeline.getIndex());
    }

    public double getLimelightSampleOrientation() {
        double angle = 0;
        LLResultTypes.ColorResult r = getLimelightResults().getColorResults().get(0);
        ArrayList<Util.Coordinate> coordinates = new ArrayList<>();
        ArrayList<Double> lengths = new ArrayList<>();
        for (List<Double> corner : r.getTargetCorners())
            coordinates.add(new Util.Coordinate(corner.get(0), corner.get(1)));
        for (Util.Coordinate coordinate : coordinates)
            lengths.add(coordinates.get(0).distanceTo(coordinate));
        for (Util.Coordinate coordinate : coordinates)
            if (coordinates.get(0).distanceTo(coordinate) == Util.max(lengths))
                angle = Math.toDegrees(Math.atan(coordinates.get(0).slopeWith(coordinate)));

        return angle;
    }

    public enum LimelightPipeline {
        NEURAL_DETECTOR(4),
        COLOUR_YELLOW(1),
        COLOUR_RED(2),
        COLOUR_BLUE(3);

        private final int index;
        LimelightPipeline(int index) {this.index = index;}
        public int getIndex() {return index;}
    }

    public void switchLimelightPipeline(int index) {limelight.pipelineSwitch(index);}
    public LLResult getLimelightResults() {return limelight.getLatestResult();}
}

class MecanumDrive {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;
    }

    public void remote(double vertical, double horizontal, double pivot, double heading) {
        double theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
        double power = Math.hypot(horizontal, vertical);
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double powerFL = power * (cos/max) + pivot;
        double powerFR = power * (sin/max) - pivot;
        double powerBL = power * -(sin/max) - pivot;
        double powerBR = power * -(cos/max) + pivot;

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

enum ScoringMode {BASKET, CHAMBER}
enum ScoringHeight {HIGH, LOW}
