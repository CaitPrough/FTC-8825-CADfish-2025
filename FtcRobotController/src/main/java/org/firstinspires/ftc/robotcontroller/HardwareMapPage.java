package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMapPage {
    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Public OpMode members. */
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    public BNO055IMU imu;
    public DigitalChannel button; // Add the button

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FL = hwMap.get(DcMotor.class, "leftfront");
        FR = hwMap.get(DcMotor.class, "leftback");
        BL = hwMap.get(DcMotor.class, "rightfront");
        BR = hwMap.get(DcMotor.class, "rightback");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setAllPower(0);

        // Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        button = hwMap.get(DigitalChannel.class, "slide_button");
        button.setMode(DigitalChannel.Mode.INPUT);
    }

    // Set power to all motors
    public void setAllPower(double p) {
        setMotorPower(p, p, p, p);
    }

    public void setMotorPower(double FrontLeft, double FrontRight, double BackLeft, double BackRight) {
        FL.setPower(FrontLeft);
        BL.setPower(BackLeft);
        BR.setPower(BackRight);
        FR.setPower(FrontRight);
    }

    // Method to check button state
    public boolean isButtonPressed() {
        return !button.getState(); // Assumes the button is active-low (pressed = false)
    }
}
