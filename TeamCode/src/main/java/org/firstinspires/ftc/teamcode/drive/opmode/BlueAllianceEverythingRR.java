package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Common.Newton2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@Autonomous
public class BlueAllianceEverythingRR extends LinearOpMode {

    Newton2 newton;

    public void runOpMode() {
        Pose2d startPose = new Pose2d(-28, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-28, 33, Math.toRadians(140)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-59, 58, Math.toRadians(180)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(24)
                .build();
        newton = new Newton2();
        newton.initialize(hardwareMap, telemetry, "Right");

        telemetry.addData("note", "there has been an update");
        telemetry.update();


        waitForStart();

        String position;
        position = newton.barcodeDetection("Right");

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


        waitForStart();


        if(isStopRequested()) return;






    }
    public void clawLowRung(){
        Pose2d startPose = new Pose2d(-28, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-28, 33, Math.toRadians(140)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-59, 58, Math.toRadians(180)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(24)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }
    public void clawMiddleRung(){
        Pose2d startPose = new Pose2d(-28, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-28, 33, Math.toRadians(140)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-59, 58, Math.toRadians(180)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(24)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }
    public void clawHighRung(){
        Pose2d startPose = new Pose2d(-28, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-28, 33, Math.toRadians(140)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-59, 58, Math.toRadians(180)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeLeft(24)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
    }
}