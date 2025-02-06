package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.features2d.SimpleBlobDetector;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

@Autonomous(name = "autoUsingApriltags")
public class autoUsingApriltags extends LinearOpMode {
    TouchSensor button;
    public DcMotor FL;
    public DcMotor BL;
    public DcMotor FR;
    public DcMotor BR;
    public DcMotor elevation;
    public DcMotor slide;
    public CRServo roller;
    public Servo tilt;
    public Servo dump;
    // public Servo launch;


    float normalPower = 0.7f;
    final float lowerPower = 0.4f;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private SimpleBlobDetector blobDetector;
    private VisionPortal visionPortal;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() {
        float turn_FL_X = 0;
        float turn_BR_X = 0;
        float turn_FR_X = 0;
        float turn_BL_X = 0;
        float strafe_FR_X = 0;
        float strafe_BL_X = 0;
        float strafe_BR_X = 0;
        float strafe_FL_X = 0;
        float strafe_FL_Y = 0;
        float strafe_FR_Y = 0;
        float strafe_BL_Y = 0;
        float strafe_BR_Y = 0;
        double driveSpeed = 1.0;
        float strafe_power = 0;
        int evelation_hold_pos;
        boolean elevation_locked = false;
        long lock_start_time = System.currentTimeMillis();
        boolean buttonHitRecently = false;
        long buttonHitTime = 0;
        int HOLD_DURATION = 700;
        double HOLDING_POWER = 0.2;
        boolean isPositionSet = false;
        long positionHoldStartTime = 0;
        boolean unload_on_button_lock = false;
        long unroll_start_time = 0;
        boolean sequenceStarted = false;
        boolean slide_set = false;
        boolean turn180 = false;
        long turn180_timer = 0;

        FL = hardwareMap.get(DcMotor.class, "leftfront");
        BL = hardwareMap.get(DcMotor.class, "leftback");
        FR = hardwareMap.get(DcMotor.class, "rightfront");
        BR = hardwareMap.get(DcMotor.class, "rightback");
        elevation = hardwareMap.get(DcMotor.class, "elevationMotor");
        slide = hardwareMap.get(DcMotor.class, "slideMotor");
        tilt = hardwareMap.get(Servo.class, "tilt");
        roller = hardwareMap.get(CRServo.class, "roller");
        dump = hardwareMap.get(Servo.class, "dump");
        // launch = hardwareMap.get(Servo.class, "launch");


        button = hardwareMap.get(TouchSensor.class, "button");

        initAprilTag();
        waitForStart();
        if (opModeIsActive()) {

            elevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            evelation_hold_pos = elevation.getCurrentPosition(); // bad code lol



            move(1, 0.5, 0.5);
            move(2, 0.5, -0.5);
            visionPortal.resumeStreaming();


            telemetry.update();

        }
    }

    public void move(double distance, double left_power, double right_power) {
        //532 is equivalent to 1ft
        distance = distance * 532;
        int edistance = Math.toIntExact(Math.round(distance));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (left_power > 0) {
            FL.setTargetPosition(-edistance);
            BL.setTargetPosition(-edistance);
        } else {
            FL.setTargetPosition(edistance);
            BL.setTargetPosition(edistance);
        }
        if (right_power > 0) {
            FR.setTargetPosition(-edistance);
            BR.setTargetPosition(-edistance);
        } else {
            FR.setTargetPosition(edistance);
            BR.setTargetPosition(edistance);
        }
        FL.setPower(left_power);
        BL.setPower(left_power);
        FR.setPower(right_power);
        BR.setPower(right_power);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.isBusy()) {
            telemetry.addData("busy", distance);
            telemetry.update();
        }
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        sleep(150);
    }

    private void strafe(double distance, double fL, double fR, double bL, double bR) {
        //532 is equivalent to 1ft
        distance = distance * 532;
        int edistance = Math.toIntExact(Math.round(distance));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (fL > 0) {
            FL.setTargetPosition(edistance);
        } else {
            FL.setTargetPosition(-edistance);
        }
        if (fR > 0) {
            FR.setTargetPosition(edistance);
        } else {
            FR.setTargetPosition(-edistance);
        }
        if (bL > 0) {
            BL.setTargetPosition(edistance);
        } else {
            BL.setTargetPosition(-edistance);
        }
        if (bR > 0) {
            BL.setTargetPosition(edistance);
        } else {
            BL.setTargetPosition(-edistance);
        }
        FL.setPower(fL);
        FR.setPower(fR);
        BL.setPower(bL);
        BR.setPower(bR);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.isBusy()) {
            telemetry.addData("busy", distance);
            telemetry.update();
        }
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        sleep(250);
    }
    public void initAprilTag() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal using the builder
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "webcam1"))
                    .addProcessor(aprilTag)
                    .enableLiveView(true)
                    .setAutoStartStreamOnBuild(true)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .build();
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }   // end method initAprilTag()

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()



} // end class
