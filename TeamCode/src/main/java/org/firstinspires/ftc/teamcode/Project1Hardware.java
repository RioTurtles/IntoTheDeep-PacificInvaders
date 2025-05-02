package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.DifferentialModule;
import org.firstinspires.ftc.teamcode.robot.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.DualServoModule;
import org.firstinspires.ftc.teamcode.robot.MultipleMotorSystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

public class Project1Hardware {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx linearLeft, linearRight, verticalLeft, verticalRight;
    ServoImplEx intakeLeft, intakeRight, clawIntake;
    ServoImplEx puncherLeft, puncherRight, armLeft, armRight, turret, clawScoring;
    IMU imu;
    Limelight3A limelight3A;

    Drivetrain drivetrain;
    LinearSlider linearSlider;
    VerticalSlider verticalSlider;
    Intake intake;
    Scoring scoring;
    Limelight limelight;

    Mode mode;
    Height height;

    public Project1Hardware(@NonNull HardwareMap hardwareMap) {
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
        // limelight3A = hardwareMap.get(Limelight3A.class, "Limelight 3A");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        linearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLeft.setDirection(Servo.Direction.FORWARD);
        intakeRight.setDirection(Servo.Direction.REVERSE);
        clawIntake.setDirection(Servo.Direction.FORWARD);
        puncherLeft.setDirection(Servo.Direction.REVERSE);
        puncherRight.setDirection(Servo.Direction.FORWARD);
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight.setDirection(Servo.Direction.REVERSE);
        turret.setDirection(Servo.Direction.FORWARD);
        clawScoring.setDirection(Servo.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drivetrain = new Drivetrain(frontLeft, frontRight, backLeft, backRight);
        linearSlider = new LinearSlider(linearLeft, linearRight);
        verticalSlider = new VerticalSlider(verticalLeft, verticalRight);
        intake = new Intake(intakeLeft, intakeRight, clawIntake);
        scoring = new Scoring(armLeft, armRight, puncherLeft, puncherRight, turret, clawScoring);
        limelight = new Limelight(limelight3A);

        linearSlider.reset();
        verticalSlider.reset();

        mode = Mode.SAMPLE;
        height = Height.HIGH;
    }

    public double getHeadingRad() {return imu.getRobotYawPitchRollAngles().getYaw();}
    public double getHeadingDeg() {return Math.toDegrees(getHeadingRad());}
    public void resetIMU() {imu.resetYaw();}

    public static class LinearSlider extends MultipleMotorSystem {
        public LinearSlider(DcMotorEx... motors) {
            super(motors);
            setMinimum(0);
            setMaximum(1600);  // tune this
        }

        public void setExtended() {setPosition(1600);}
        public void retract() {setPosition(0);}
    }

    public static class VerticalSlider extends MultipleMotorSystem {
        public VerticalSlider(DcMotorEx... motors) {
            super(motors);
            setMinimum(0);
            setMaximum(3800);  // tune this
        }

        public void setHighBasket() {setPosition(3000);}
        public void setLowBasket() {setPosition(1500);}
        public void setHighChamber() {setPosition(1800);}
        public void setLowChamber() {setPosition(900);}
        public void retract() {setPosition(0);}
        public void powerOff() {setPower(0);}
    }

    public static class Intake extends DifferentialModule {
        ServoImplEx claw;
        boolean clawOpen = false;
        boolean atTransfer = false;

        public Intake(ServoImplEx left, ServoImplEx right, ServoImplEx claw) {
            super(left, right);
            this.claw = claw;
        }

        public void setIntake() {setPitch(HALF); atTransfer = false;}
        public void setIntakeRaised() {setPitch(0.5); atTransfer = false;}
        public void setTransferSample() {setPosition(1, 0); atTransfer = true;}
        public void setTransferSpecimen() {setPosition(0.7, 0); atTransfer = true;}

        public void clawOpen() {claw.setPosition(0.2); clawOpen = true;}
        public void clawClose() {claw.setPosition(0); clawOpen = false;}
    }

    public static class Scoring {
        Arm arm;
        Puncher puncher;
        ServoImplEx turret, claw;
        private final static double TURRET_RANGE = 255;
        boolean clawOpen = false;

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
            boolean atTransfer = false;
            public Arm(ServoImplEx left, ServoImplEx right) {super(left, right);}
            public void setSampleTransfer() {setPosition(0.02); atTransfer = true;}
            public void setSpecimenTransfer() {setPosition(0.07); atTransfer = true;} // TODO: Tune this
            public void setScoring() {setPosition(0.68); atTransfer = false;}
        }

        public static class Puncher extends DualServoModule {
            public static final double RETRACTED = 0.19;
            public static final double EXTENDED = 0.59;
            public Puncher(ServoImplEx left, ServoImplEx right) {super(left, right, +0.02, 0);}

            public void setRetracted() {setPosition(RETRACTED);}
            public void setAim() {setPosition((RETRACTED - EXTENDED) / 2);}
            public void setExtended() {setPosition(EXTENDED);}

            public void setLength(double length) {
                setPosition(RETRACTED + (EXTENDED - RETRACTED) * length);
            }
        }

        public void setTurret(double angle) {
            if (puncher.getPosition() >= 0.19)
                turret.setPosition(Range.clip(
                        (angle + TURRET_RANGE / 2) / TURRET_RANGE,
                        0.15, 0.85
                ));
        }

        public void alignTurret(double imu, double target) {
            double difference = target - imu;
            if (difference > 180) difference -= 360;
            if (difference < -180) difference += 360;
            setTurret(-difference);
        }

        public void clawOpen() {claw.setPosition(0.3); clawOpen = true;}
        public void clawClose() {claw.setPosition(0); clawOpen = false;}
    }

    public static class Limelight {
        private final Limelight3A limelight;
        public Limelight(Limelight3A limelight) {this.limelight = limelight;}

        public void start(int pipeline) {
            switchPipeline(pipeline);
            limelight.start();
        }

        public void start(Colour colour) {start(colour.index());}
        public void start() {start(1);}
        public void stop() {limelight.stop();}

        public void switchPipeline(int pipeline) {limelight.pipelineSwitch(pipeline);}
        public void switchPipeline(Colour colour) {limelight.pipelineSwitch(colour.index());}

        public @Nullable LLResult getValidResults() {
            LLResult result = limelight.getLatestResult();
            if (Objects.isNull(result)) return null; else {
                // Nested-ifs as isValid() could produce NullPointerException if result is null
                if (result.isValid()) return result; else return null;
            }
        }

        public @Nullable List<LLResultTypes.DetectorResult> getValidDetections() {
            LLResult result = getValidResults();
            if (Objects.isNull(result)) return null; else {
                assert result != null;
                return result.getDetectorResults();
            }
        }

        public @Nullable Double[] getData() {
            List<LLResultTypes.DetectorResult> results = getValidDetections();
            if (Objects.isNull(results)) return null;
            assert results != null;

            double maxTA = 0;
            @Nullable Double[] bestData = null;
            for (LLResultTypes.DetectorResult result : results) {
                if (result.getTargetArea() >= 0.05 && result.getTargetArea() > maxTA) {
                    List<Coordinate> list = Coordinate.fromTargetCorners(result.getTargetCorners());
                    double k = Coordinate.getMaxXDist(list) / Coordinate.getMaxYDist(list);
                    double bestOrientation = Math.toDegrees(Math.atan((7 - 3 * k) / (7 * k - 3)));

                    bestData = new Double[]{
                            result.getTargetXPixels(),
                            result.getTargetYPixels(),
                            90 - bestOrientation
                    };
                }
            }

            return bestData;
        }

        private static class Coordinate {
            public double x;
            public double y;
            public Coordinate(double x, double y) {this.x = x; this.y = y;}
            public double getX() {return x;}
            public double getY() {return y;}

            public static List<Coordinate> fromTargetCorners(List<List<Double>> corners) {
                List<Coordinate> result = new ArrayList<>();
                for (List<Double> corner : corners)
                    result.add(new Coordinate(corner.get(0), corner.get(1)));
                return result;
            }

            public static double getMaxXDist(List<Coordinate> coordinates) {
                if (coordinates.size() < 2) {
                    throw new IllegalArgumentException("At least two coordinates are required.");
                }

                double maxDistance = 0;

                for (int i = 0; i < coordinates.size(); i++) {
                    for (int j = i + 1; j < coordinates.size(); j++) {
                        double distance = Math.abs(coordinates.get(i).x - coordinates.get(j).x);
                        if (distance > maxDistance) maxDistance = distance;
                    }
                }

                return maxDistance;
            }

            public static double getMaxYDist(List<Coordinate> coordinates) {
                if (coordinates.size() < 2) {
                    throw new IllegalArgumentException("At least two coordinates are required.");
                }

                double maxDistance = 0;

                for (int i = 0; i < coordinates.size(); i++) {
                    for (int j = i + 1; j < coordinates.size(); j++) {
                        double distance = Math.abs(coordinates.get(i).y - coordinates.get(j).y);
                        if (distance > maxDistance) maxDistance = distance;
                    }
                }

                return maxDistance;
            }
        }
    }

    public enum Colour {
        YELLOW(1),
        RED(2),
        BLUE(3),
        NEURAL(4);

        private final int index;
        public int index() {return this.index;}
        Colour(int index) {this.index = index;}
    }

    public enum Mode {SAMPLE, SPECIMEN}
    public enum Height {HIGH, LOW}
}
