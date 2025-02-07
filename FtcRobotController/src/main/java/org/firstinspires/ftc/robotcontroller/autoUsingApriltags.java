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

import java.util.List;

@Autonomous(name = "autoUsingApriltags")
public class autoUsingApriltags extends LinearOpMode {
    private TouchSensor button;
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;
    private DcMotor elevation;
    private DcMotor slide;
    private CRServo roller;
    private Servo tilt;
    private Servo dump;

    private float normalPower = 0.7f;
    private final float lowerPower = 0.4f;

    private boolean run_apriltag_assist = true;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> currentDetections;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();
        initAprilTag();

        waitForStart();
        visionPortal.resumeStreaming();
        if (opModeIsActive()) {
            // Set motor directions and behaviors
            setMotorConfigurations();

            int elevationHoldPos = elevation.getCurrentPosition();

            // Ensure vision portal is streaming




            // First movement sequence
            move(0.8, 0.5, 0.5);
            strafe(1.75, 0.5, -0.5, -0.5, 0.5);
            move(0.4, 0.5, -0.5);
            if (tilt.getPosition() <= 0.4) {
                tilt.setPosition(0.4);
                sleep(50);
            }
            sleep(400);

            // Now start the AprilTag detection sequence
            boolean alignmentComplete = false;

            while (opModeIsActive() && !alignmentComplete && run_apriltag_assist) {
                // Get fresh AprilTag detections
                currentDetections = aprilTag.getDetections();

                // Debug output
                telemetry.addData("Number of detections", currentDetections.size());

                if (currentDetections != null && !currentDetections.isEmpty()) {
                    boolean tagFound = false;

                    for (AprilTagDetection detection : currentDetections) {
                        if (detection != null && detection.metadata != null &&
                                (detection.id == 16 || detection.id == 13)) {

                            tagFound = true;
                            double bearing = detection.ftcPose.bearing;

                            telemetry.addData("Tag ID", detection.id);
                            telemetry.addData("Bearing", bearing);

                            // If we're close to aligned
                            if (Math.abs(bearing) < 1.5) {
                                stopMotors();
                                alignmentComplete = true;
                                break;
                            }

                            // Calculate turn power
                            float turnPower = (float) Math.min(0.15f, Math.abs(bearing) * 0.05f);
                            if (turnPower < 0.08f) turnPower = 0.08f;
                            turnPower *= (bearing > 0) ? 1 : -1;

                            // Apply turn power
                            setTurnPowers(turnPower);
                            telemetry.addData("Turn Power", turnPower);
                        }
                    }

                    if (!tagFound) {
                        telemetry.addLine("No target tags found");
                        stopMotors();
                    }
                } else {
                    telemetry.addLine("No detections");
                    stopMotors();
                }

                telemetryAprilTag();
                telemetry.update();

                // Small delay to prevent CPU overload
                sleep(10);
            }

            // Stop all motors after alignment
            stopMotors();

            if (alignmentComplete) {
                telemetry.addLine("Alignment Complete!");
            } else {
                telemetry.addLine("Alignment Failed or Interrupted");
            }
            telemetry.update();






            elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevation.setTargetPosition(-5500);
            elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevation.setPower(-1);
            while (true){ // wait until we hit position, then do a motor hold
                if (elevation.getCurrentPosition() <= -5500){
                    elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    elevation.setPower(-0.15);
                    dump.setPosition(0.45);
                    break;
                }
            }

            move(0.65, -0.5, -0.5);
            elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevation.setPower(0);

            dump.setPosition(0.3);  // -5900
            sleep(1100);
            dump.setPosition(0.45);


            elevation.setTargetPosition(-500);
            elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevation.setPower(0.3);

            move(0.7, 0.5, 0.5);

            sleep(500);
            elevation.setPower(0);




            sleep(1000); // Pause to show final status
        }
    }

    private void initializeHardware() {
        FL = hardwareMap.get(DcMotor.class, "leftfront");
        BL = hardwareMap.get(DcMotor.class, "leftback");
        FR = hardwareMap.get(DcMotor.class, "rightfront");
        BR = hardwareMap.get(DcMotor.class, "rightback");
        elevation = hardwareMap.get(DcMotor.class, "elevationMotor");
        slide = hardwareMap.get(DcMotor.class, "slideMotor");
        tilt = hardwareMap.get(Servo.class, "tilt");
        roller = hardwareMap.get(CRServo.class, "roller");
        dump = hardwareMap.get(Servo.class, "dump");
        button = hardwareMap.get(TouchSensor.class, "button");
    }

    private void setMotorConfigurations() {
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
    }

    private void handleAprilTagAssist() {
        boolean tagFound = false;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == 16 || detection.id == 13)) {
                tagFound = true;
                double bearing = detection.ftcPose.bearing;

                telemetry.addData("Bearing", bearing);

                float turnPower = (float) Math.min(0.15f, Math.abs(bearing) * 0.05f);
                if (turnPower < 0.08f) turnPower = 0.08f;

                if (Math.abs(bearing) < 1.5) {
                    stopMotors();
                    return;
                }

                turnPower *= (bearing > 0) ? 1 : -1;
                setTurnPowers(turnPower);
                telemetry.addData("Turn Power", turnPower);
                break;
            }
        }
    }

    private void stopMotors() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void setTurnPowers(float power) {
        FL.setPower(-power);
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(power);
    }

    public void move(double distance, double left_power, double right_power) {
        int edistance = (int)(distance * 532); // 532 ticks per foot

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setTargetPositions(edistance, left_power, right_power);
        setMotorPowers(left_power, right_power);
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (FL.isBusy() && opModeIsActive()) {
            telemetry.addData("Moving", distance);
            telemetry.update();
        }

        stopAndResetMotors();
        sleep(150);
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        FL.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        BR.setMode(mode);
    }

    private void setTargetPositions(int distance, double left_power, double right_power) {
        FL.setTargetPosition(left_power > 0 ? -distance : distance);
        BL.setTargetPosition(left_power > 0 ? -distance : distance);
        FR.setTargetPosition(right_power > 0 ? -distance : distance);
        BR.setTargetPosition(right_power > 0 ? -distance : distance);
    }

    private void setMotorPowers(double left_power, double right_power) {
        FL.setPower(Math.abs(left_power));
        BL.setPower(Math.abs(left_power));
        FR.setPower(Math.abs(right_power));
        BR.setPower(Math.abs(right_power));
    }

    private void stopAndResetMotors() {
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopMotors();
    }

    public void strafe(double distance, double fL, double fR, double bL, double bR) {
        int edistance = (int)(distance * 532);

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setTargetPosition(fL > 0 ? edistance : -edistance);
        FR.setTargetPosition(fR > 0 ? edistance : -edistance);
        BL.setTargetPosition(bL > 0 ? edistance : -edistance);
        BR.setTargetPosition(bR > 0 ? edistance : -edistance);

        FL.setPower(Math.abs(fL));
        FR.setPower(Math.abs(fR));
        BL.setPower(Math.abs(bL));
        BR.setPower(Math.abs(bR));

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (FL.isBusy() && opModeIsActive()) {
            telemetry.addData("Strafing", distance);
            telemetry.update();
        }

        stopAndResetMotors();
        sleep(250);
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

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
    }

    private void telemetryAprilTag() {
        telemetry.addData("# AprilTags Detected", currentDetections.size());

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
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}