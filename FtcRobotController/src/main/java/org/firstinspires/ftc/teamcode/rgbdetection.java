package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "colorsensor")
public class rgbdetection extends LinearOpMode {

    private ColorSensor colorSensor_REV_ColorRangeSensor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        colorSensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("red", colorSensor_REV_ColorRangeSensor.red());
                telemetry.addData("green", colorSensor_REV_ColorRangeSensor.green());
                telemetry.addData("blue", colorSensor_REV_ColorRangeSensor.blue());
                telemetry.update();
            }
        }
    }
}