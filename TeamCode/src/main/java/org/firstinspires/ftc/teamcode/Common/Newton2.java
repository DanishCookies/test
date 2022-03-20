package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Newton2 {
    public OpenCvWebcam Webcam1;
    DcMotor Right_Carousel;
    //DcMotor Left_Carousel;
    public CRServo IntakeServo;
    public Servo BoxServo;
    BNO055IMU imu;
    Orientation angles;
    Orientation lastAngles;
    DcMotor LinearSlide;

    CustomObjectDetectionRightAllignment rightdetector;
    CustomObjectDetectionLeftAlignment leftdetector;

    public void initialize(HardwareMap myHardwareMap, Telemetry telemetry, String position) {
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BoxServo = myHardwareMap.servo.get("box");
        LinearSlide = myHardwareMap.dcMotor.get("LeftLinearSlide");
        LinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        BoxServo.setDirection(Servo.Direction.FORWARD);
        Right_Carousel.setDirection(DcMotor.Direction.FORWARD);
        //Left_Carousel.setDirection(DcMotor.Direction.FORWARD);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = myHardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        int cameraMonitorViewId = myHardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", myHardwareMap.appContext.getPackageName());

        Webcam1 = OpenCvCameraFactory.getInstance().createWebcam(myHardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);

        if (position.equals("Left"))
        {
            leftdetector = new CustomObjectDetectionLeftAlignment(telemetry);
            Webcam1.setPipeline(leftdetector);
        }
        else
        {
            rightdetector = new CustomObjectDetectionRightAllignment(telemetry);
            Webcam1.setPipeline(rightdetector);
        }

        Webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        lastAngles = new Orientation();

        initServo();
        initCarousel();


    }

    public String barcodeDetection(String robotpos) {
        String pos;
        pos = "";
        if (robotpos.equals("Left")) {
            switch (leftdetector.getLocation()) {
                case LEFT:
                    pos = "L";
                    Webcam1.stopStreaming();
                    break;
                case MIDDLE:
                    pos = "M";
                    Webcam1.stopStreaming();
                    break;
                case RIGHT:
                    pos = "R";
                    Webcam1.stopStreaming();
                    break;
            }
        } else
        {
            switch (rightdetector.getLocation()) {
                case LEFT:
                    pos = "L";
                    Webcam1.stopStreaming();
                    break;
                case MIDDLE:
                    pos = "M";
                    Webcam1.stopStreaming();
                    break;
                case RIGHT:
                    pos = "R";
                    Webcam1.stopStreaming();
                    break;
            }
        }
        return pos;
    }



    public void CarouselOn(double movepower) {
        Right_Carousel.setPower(movepower);
        //Left_Carousel.setPower(movepower);
    }

    public void CarouselOn() {
        Right_Carousel.setPower(0.5);
        //Left_Carousel.setPower(0.5);
    }

    public void CarouselOff() {
        Right_Carousel.setPower(0);
        //Left_Carousel.setPower(0);
    }



    private void initCarousel (){
        Right_Carousel.setPower(0);
        //Left_Carousel.setPower(0);
    }

    private void initServo () {
        BoxServo.setPosition(0.5);
        IntakeServo.setPower(0);
    }
    public void LinearSlide (double movedistance, double movepower){
        double movedistance2;
        //movedistance2 =movedistance*45.5;
        LinearSlide.setTargetPosition((int) movedistance);
        LinearSlide.setPower(movepower);
        LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void linearSlideHigh(){

        LinearSlide(1605, .5);
    }
    public void linearSlideMiddle(){
        LinearSlide(1428, .5);
    }
    public void linearSlideLow(){
        LinearSlide(0, .5);
    }





}