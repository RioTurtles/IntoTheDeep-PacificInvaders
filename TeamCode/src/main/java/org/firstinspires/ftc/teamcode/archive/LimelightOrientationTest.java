package org.firstinspires.ftc.teamcode.archive;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Project1Hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

@TeleOp
public class LimelightOrientationTest extends LinearOpMode {
    Limelight3A limelight3A;
    
    @Override
    public void runOpMode() {
        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight 3A");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        Limelight limelight = new Limelight(limelight3A);

        waitForStart();
        while (opModeIsActive()) {
            Double[] data = limelight.getData();
            if (data != null) {
                telemetry.addData("Valid tx", data[0]);
                telemetry.addData("Valid ty", data[1]);
                telemetry.addData("Valid tr", data[2]);
            } else {
                telemetry.addLine("Nothing found.");
            }
            telemetry.update();
        }
    }

    public static class Limelight {
        private final Limelight3A limelight;
        public Limelight(Limelight3A limelight) {this.limelight = limelight;}

        public void start(int pipeline) {
            switchPipeline(pipeline);
            limelight.start();
        }

        public void start(Project1Hardware.Colour colour) {start(colour.index());}
        public void start() {start(1);}
        public void stop() {limelight.stop();}

        public void switchPipeline(int pipeline) {limelight.pipelineSwitch(pipeline);}
        public void switchPipeline(Project1Hardware.Colour colour) {limelight.pipelineSwitch(colour.index());}

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
}
