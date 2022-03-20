package org.firstinspires.ftc.teamcode.TeleOp;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MecanumTeleOp")
public class MechanumTeleOp extends LinearOpMode {
    //private DcMotor ArmY;
    private DcMotor LeftLinear;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor intake;
    private DcMotor RC;
    private Servo   output;
    private DistanceSensor sensorRange;

    @Override
    public void runOpMode() throws InterruptedException {
        LeftLinear = hardwareMap.get(DcMotor.class, "LeftLinearSlide");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        RC = hardwareMap.get(DcMotor.class, "RC");
        output = hardwareMap.get(Servo.class, "OutputServo");
        sensorRange = hardwareMap.get(DistanceSensor.class, "distance");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        int MovementToggle;
        int IntakeTickBackwards;
        int IntakeTickForward;
        int CarouselTicksR;
        float Vertical;
        float Horizontal;
        float Pivot;
        MovementToggle = 0;
        IntakeTickBackwards = 0;
        IntakeTickForward = 0;
        CarouselTicksR = 0;

        telemetry.addData("Wait:", "Waiting for Start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive())
        {
            while (opModeIsActive())
            {
                Vertical = -gamepad1.left_stick_y;
                Horizontal = gamepad1.left_stick_x;
                Pivot = gamepad1.right_stick_x;

                if (gamepad1.left_stick_button && MovementToggle == 0) {
                    MovementToggle = 1;
                } else if (gamepad1.left_stick_button && MovementToggle == 1) {
                    MovementToggle = 0;
                }
                if (MovementToggle == 1) {
                    FR.setPower((-Pivot + (Vertical - Horizontal)) * 0.4);
                    BR.setPower((-Pivot + Vertical + Horizontal) * 0.4);
                    FL.setPower((Pivot + Vertical + Horizontal) * 0.4);
                    BL.setPower((Pivot + (Vertical - Horizontal)) * 0.4);
                } else if (MovementToggle == 0) {
                    FR.setPower(-Pivot + (Vertical - Horizontal));
                    BR.setPower(-Pivot + Vertical + Horizontal);
                    FL.setPower(Pivot + Vertical + Horizontal);
                    BL.setPower(Pivot + (Vertical - Horizontal));
                }

                //Linear Slide:
                if (gamepad2.a)
                {
                    output.setPosition(0.5);
                    sleep(500);
                    LeftLinear.setTargetPosition(0);
                    LeftLinear.setPower(1);
                    LeftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                    output.setPosition(1);
                }

                if (gamepad2.y)
                {
                    output.setPosition(0.5);
                    sleep(500);
                    LeftLinear.setTargetPosition(1605);
                    LeftLinear.setPower(1);
                    LeftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sleep(500);
                }

                if (gamepad2.dpad_left)
                {
                    output.setPosition(0);
                }

                if (gamepad2.dpad_right)
                {
                    output.setPosition(1);
                }

                if (gamepad2.dpad_up)
                {
                    output.setPosition(0.5);
                }

                if (gamepad2.left_bumper == true && IntakeTickBackwards == 0)
                {
                    intake.setPower(1);
                    IntakeTickBackwards = 1;
                } else if (gamepad2.left_bumper == false && IntakeTickBackwards == 1)
                {
                    intake.setPower(0);
                    IntakeTickBackwards = 0;
                }

                if (gamepad2.right_bumper == true && IntakeTickForward == 0)
                {
                    intake.setPower(-1);
                    IntakeTickForward = 1;
                } else if (gamepad2.right_bumper == false && IntakeTickForward == 1)
                {
                    intake.setPower(0);
                    IntakeTickForward = 0;
                }

                if (gamepad1.right_bumper == true && CarouselTicksR == 0)
                {
                    RC.setPower(0.8);
                    CarouselTicksR = 1;
                } else if (gamepad1.right_bumper == false && CarouselTicksR == 1)
                {
                    RC.setPower(0);
                    CarouselTicksR = 0;
                }

                if (gamepad1.left_bumper == true && CarouselTicksR == 0)
                {
                    RC.setPower(-0.8);
                    CarouselTicksR = 1;
                } else if (gamepad1.left_bumper == false && CarouselTicksR == 1)
                {
                    RC.setPower(0);
                    CarouselTicksR = 0;
                }

                telemetry.addData("FL", FL.getPower());
                telemetry.addData("BL", BL.getPower());
                telemetry.addData("FR", FR.getPower());
                telemetry.addData("BR", BR.getPower());
                telemetry.addData("Slide", LeftLinear.getCurrentPosition());
                telemetry.addData("Distance", sensorRange.getDistance(DistanceUnit.CM));
                telemetry.update();

                //Right Linear Slide:

                /*if (gamepad2.a) ;
                {
                    RightLinear.setTargetPosition(0);
                    RightLinear.setPower(1);
                    RightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad2.b) ;
                {
                    RightLinear.setTargetPosition(100);
                    RightLinear.setPower(1);
                    RightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad2.y) ;
                {
                    RightLinear.setTargetPosition(200);
                    RightLinear.setPower(1);
                    RightLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }*/
                if (gamepad2.left_stick_y < 0) {
                    LeftLinear.setTargetPosition(LeftLinear.getCurrentPosition() + 100);
                    LeftLinear.setPower(1);
                    LeftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                } else if (gamepad2.left_stick_y == 0) {
                    LeftLinear.setPower(0);
                } else if (gamepad2.left_stick_y > 0) {
                    LeftLinear.setTargetPosition(LeftLinear.getCurrentPosition() - 100);
                    LeftLinear.setPower(1);
                    LeftLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                /*if (gamepad2.dpad_left) {
                    box.setPosition(0);
                } else if (gamepad2.dpad_up) {
                    box.setPosition(0.5);
                } else if (gamepad2.dpad_right) {
                    box.setPosition(1);
                }*/
            }
        }
    }
}