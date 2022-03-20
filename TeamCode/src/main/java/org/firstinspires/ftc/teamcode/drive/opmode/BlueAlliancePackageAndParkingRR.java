package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Common.Newton2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@Autonomous
public class BlueAlliancePackageAndParkingRR extends LinearOpMode {
 Newton2 newton;
    public void runOpMode() {

        Pose2d startPose = new Pose2d(10, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, 33, Math.toRadians(45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, 69, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(30)
                .build();
        waitForStart();
        newton = new Newton2();
        newton.initialize(hardwareMap, telemetry, "Left");

        telemetry.addData("note", "there has been an update");
        telemetry.update();

        String position;
        position = newton.barcodeDetection("Left");

        switch (position) {
            case "L":
                clawLowRung();
                telemetry.addData("Block Location", "Low");
                telemetry.update();
                break;
            case "M":
                clawMiddleRung();
                telemetry.addData("Block Location", "Middle");
                telemetry.update();
                break;
            case "R":
                clawHighRung();
                telemetry.addData("Block Location", "High");
                telemetry.update();
                break;
        }






        if(isStopRequested()) return;


    }
    public void clawLowRung(){
        Pose2d startPose = new Pose2d(10, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, 33, Math.toRadians(45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, 69, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(30)
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

    }
    public void clawMiddleRung(){
        Pose2d startPose = new Pose2d(10, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, 33, Math.toRadians(45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, 69, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(30)
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

    }
    public void clawHighRung(){
        Pose2d startPose = new Pose2d(10, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, 33, Math.toRadians(45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, 69, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(30)
                .build();
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

    }
}