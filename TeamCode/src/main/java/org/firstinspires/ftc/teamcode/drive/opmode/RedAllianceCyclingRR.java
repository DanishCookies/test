package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Common.Newton2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@Autonomous
public class RedAllianceCyclingRR extends LinearOpMode {
Newton2 newton;
    public void runOpMode() {
        Pose2d startPose = new Pose2d(10, -65, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, -66, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(35)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(35)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();

        waitForStart();
        newton = new Newton2();
        newton.initialize(hardwareMap, telemetry, "Left");

        telemetry.addData("note", "there has been an update");
        telemetry.update();
        waitForStart();

        String position;
        position = newton.barcodeDetection("Left");

        switch (position) {
            case "L":
                linearSlideLowRung();
                telemetry.addData("Block Location", "Low");
                telemetry.update();
                break;
            case "M":
                linearSlideMiddleRung();
                telemetry.addData("Block Location", "Middle");
                telemetry.update();
                break;
            case "R":
                linearSlideHighRung();
                telemetry.addData("Block Location", "High");
                telemetry.update();
                break;
        }


        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        for (int count = 0; count < 2; count++) {
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
        }
    }
    public void linearSlideLowRung(){
        Pose2d startPose = new Pose2d(10, -65, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, -66, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(35)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(35)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        for (int count = 0; count < 2; count++) {
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
        }

    }
    public void linearSlideMiddleRung(){
        Pose2d startPose = new Pose2d(10, -65, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, -66, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(35)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(35)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        for (int count = 0; count < 2; count++) {
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
        }

    }
    public void linearSlideHighRung(){
        Pose2d startPose = new Pose2d(10, -65, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, -66, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(35)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(35)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, -33, Math.toRadians(-45)))
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        for (int count = 0; count < 2; count++) {
            drive.followTrajectory(traj4);
            drive.followTrajectory(traj5);
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
        }

    }
}