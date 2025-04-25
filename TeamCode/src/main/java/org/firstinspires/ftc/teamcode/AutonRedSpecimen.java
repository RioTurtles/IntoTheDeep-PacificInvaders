package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous (name = "RedSpecimen")
public class AutonRedSpecimen extends LinearOpMode {

    enum States {
        PRELOAD,
        SCORE_PRELOAD,
        PATH_TO_FIRST,
        SCORE_FIRST,
        PATH_TO_SECOND,
        SCORE_SECOND,
        PATH_TO_THIRD,
        SCORE_THIRD,
        PARK
    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Project1Hardware robot = new Project1Hardware(hardwareMap);
        PartialRoadrunnerHelper roadrunner = new PartialRoadrunnerHelper(drive, robot.drivetrain::remote);

        States state = States.PRELOAD;
        ElapsedTime autonomous = new ElapsedTime();
        ElapsedTime timer1 = new ElapsedTime();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(new Pose2d(12, -62, Math.toRadians(270)));

        roadrunner.setPIDCoefficients(Axis.X, 0, 0, 0);
        roadrunner.setPIDCoefficients(Axis.Y, 0, 0, 0);
        roadrunner.setPIDCoefficients(Axis.HEADING, 0.8, 0, 0);

        waitForStart();
        autonomous.reset();
        roadrunner.setPoseEstimate(-33.14, -62.99, 270.00);
        while (opModeIsActive()) {
            if (state == States.PRELOAD) {
                roadrunner.setTarget(0, -28.57, 90);
                if (autonomous.seconds() > 4) {
                    state = States.SCORE_PRELOAD;
                    timer1.reset();
                }
            }

            if (state == States.SCORE_PRELOAD) {
                roadrunner.setTarget(0, -28.57, 90);
            }

        }

    }
}
