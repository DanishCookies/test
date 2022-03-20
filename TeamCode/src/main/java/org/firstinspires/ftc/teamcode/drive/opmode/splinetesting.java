package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@Autonomous
public class splinetesting extends LinearOpMode {
    @Override

    public void runOpMode() {
        Pose2d startPose = new Pose2d(10, 65, Math.toRadians(-90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(5, 33, Math.toRadians(45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(10, 67, Math.toRadians(0)))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(35)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(35)
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, 33, Math.toRadians(45)))
                .build();
Trajectory cyclingtest = drive.trajectoryBuilder(traj3.end(), true)
        .splineTo(new Vector2d(10, 67), Math.toRadians(0))
        .splineTo(new Vector2d(5, 33), Math.toRadians(45))
        .build();
Trajectory cyclingtest2 = drive.trajectoryBuilder(cyclingtest.end())
        .splineTo(new Vector2d(10, 67), Math.toRadians(0))
        .splineTo(new Vector2d(10, 102), Math.toRadians(0))
        .build();

        waitForStart();


        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        for (int count = 0; count < 3; count++) {
            drive.followTrajectory(cyclingtest);
            drive.followTrajectory(cyclingtest2);
        }
    }
}