package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@Autonomous
public class FunctionsTesting extends LinearOpMode {
    @Override

    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory strafe = drive.trajectoryBuilder(startPose)
                .strafeLeft(24)
                .build();

    Trajectory forward = drive.trajectoryBuilder(strafe.end())
            .forward(24)
            .build();

    Trajectory right = drive.trajectoryBuilder(new Pose2d(24, 24,  Math.toRadians(-90)))
            .forward(24)
            .build();

    Trajectory back = drive.trajectoryBuilder(new Pose2d(), true)
            .back(24)
            .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(strafe);
        sleep(1000);
        drive.followTrajectory(forward);
        sleep(1000);
        drive.turn(Math.toRadians(-90));
        sleep(1000);
        drive.followTrajectory(right);
        sleep(1000);
        drive.turn(Math.toRadians(90));
        sleep(1000);
        drive.followTrajectory(back);
    }
}