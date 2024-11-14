package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {

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

    final float normalPower = 0.7f;
    final float lowerPower = 0.4f;

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

        FL = hardwareMap.get(DcMotor.class, "leftfront");
        BL = hardwareMap.get(DcMotor.class, "leftback");
        FR = hardwareMap.get(DcMotor.class, "rightfront");
        BR = hardwareMap.get(DcMotor.class, "rightback");
        elevation = hardwareMap.get(DcMotor.class, "elevationMotor");
        slide = hardwareMap.get(DcMotor.class, "slideMotor");
        tilt = hardwareMap.get(Servo.class, "tilt");
        roller = hardwareMap.get(CRServo.class,"roller");
        dump = hardwareMap.get(Servo.class, "dump");
        // launch = hardwareMap.get(Servo.class, "launch");
        elevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if (opModeIsActive()) {
            FL.setDirection(DcMotorSimple.Direction.FORWARD);
            BL.setDirection(DcMotorSimple.Direction.FORWARD);
            FR.setDirection(DcMotorSimple.Direction.REVERSE);
            BR.setDirection(DcMotorSimple.Direction.REVERSE);
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive()) {
                telemetry.addData("leftstickX", gamepad1.left_stick_x);
                telemetry.addData("leftstickY", gamepad1.left_stick_y);
                telemetry.addData("rightstickX", gamepad1.right_stick_x);
                telemetry.update();
                //lower power for more precise robot movement
                if (gamepad1.x) {
                    if (gamepad1.right_stick_y > 0.1) {
                        // forward
                        strafe_BR_Y = gamepad1.right_stick_y * lowerPower;
                        strafe_FL_Y = gamepad1.right_stick_y * lowerPower;
                        strafe_FR_Y = gamepad1.right_stick_y * lowerPower;
                        strafe_BL_Y = gamepad1.right_stick_y * lowerPower;
                    } else if (gamepad1.right_stick_y < -0.1) {
                        // backward
                        strafe_BR_Y = gamepad1.right_stick_y * lowerPower;
                        strafe_FL_Y = gamepad1.right_stick_y * lowerPower;
                        strafe_FR_Y = gamepad1.right_stick_y * lowerPower;
                        strafe_BL_Y = gamepad1.right_stick_y * lowerPower;
                    } else if (gamepad1.left_stick_x > 0.1) {
                        // left turn
                        turn_FL_X = -gamepad1.left_stick_x * lowerPower;
                        turn_FR_X = gamepad1.left_stick_x * lowerPower;
                        turn_BL_X = -gamepad1.left_stick_x * lowerPower;
                        turn_BR_X = gamepad1.left_stick_x * lowerPower;
                    } else if (gamepad1.left_stick_x < -0.1) {
                        // right turn
                        turn_FL_X = -gamepad1.left_stick_x * lowerPower;
                        turn_FR_X = gamepad1.left_stick_x * lowerPower;
                        turn_BL_X = -gamepad1.left_stick_x * lowerPower;
                        turn_BR_X = gamepad1.left_stick_x * lowerPower;
                    } else {
                        turn_FL_X = 0;
                        turn_FR_X = 0;
                        turn_BL_X = 0;
                        turn_BR_X = 0;
                        strafe_FL_Y = 0;
                        strafe_FR_Y = 0;
                        strafe_BL_Y = 0;
                        strafe_BR_Y = 0;

                    }
                } else {
                    if (gamepad1.right_stick_y > 0.1) {
                        // forward
                        strafe_BR_Y = gamepad1.right_stick_y * normalPower;
                        strafe_FL_Y = gamepad1.right_stick_y * normalPower;
                        strafe_FR_Y = gamepad1.right_stick_y * normalPower;
                        strafe_BL_Y = gamepad1.right_stick_y * normalPower;
                    } else if (gamepad1.right_stick_y < -0.1) {
                        // backward
                        strafe_BR_Y = gamepad1.right_stick_y * normalPower;
                        strafe_FL_Y = gamepad1.right_stick_y * normalPower;
                        strafe_FR_Y = gamepad1.right_stick_y * normalPower;
                        strafe_BL_Y = gamepad1.right_stick_y * normalPower;
                    } else if (gamepad1.left_stick_x > 0.1) {
                        // left turn
                        turn_FL_X = -gamepad1.left_stick_x * normalPower;
                        turn_FR_X = gamepad1.left_stick_x * normalPower;
                        turn_BL_X = -gamepad1.left_stick_x * normalPower;
                        turn_BR_X = gamepad1.left_stick_x * normalPower;
                    } else if (gamepad1.left_stick_x < -0.1) {
                        // right turn
                        turn_FL_X = -gamepad1.left_stick_x * normalPower;
                        turn_FR_X = gamepad1.left_stick_x * normalPower;
                        turn_BL_X = -gamepad1.left_stick_x * normalPower;
                        turn_BR_X = gamepad1.left_stick_x * normalPower;
                    } else {
                        turn_FL_X = 0;
                        turn_FR_X = 0;
                        turn_BL_X = 0;
                        turn_BR_X = 0;
                        strafe_FL_Y = 0;
                        strafe_FR_Y = 0;
                        strafe_BL_Y = 0;
                        strafe_BR_Y = 0;

                    }
                }
                // strafe
                if (gamepad1.right_stick_x < -0.1) {
                    // right strafe
                    strafe_FL_X = -gamepad1.right_stick_x;
                    strafe_FR_X = gamepad1.right_stick_x;
                    strafe_BL_X = gamepad1.right_stick_x;
                    strafe_BR_X = -gamepad1.right_stick_x;
                } else if (gamepad1.right_stick_x > 0.1) {
                    // left strafe
                    strafe_FL_X = -gamepad1.right_stick_x;
                    strafe_FR_X = gamepad1.right_stick_x;
                    strafe_BL_X = gamepad1.right_stick_x;
                    strafe_BR_X = -gamepad1.right_stick_x;
                } else {
                    strafe_FL_X = 0;
                    strafe_FR_X = 0;
                    strafe_BL_X = 0;
                    strafe_BR_X = 0;
                }

                // elevation (formerly pitch)
                if (gamepad1.left_bumper) {

                    elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    elevation.setPower(1);
                }

                else if (gamepad1.right_bumper) {
                    if (tilt.getPosition() != 0.4){
                        tilt.setPosition(0.4);
                        sleep(50);
                    }
                    elevation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                   // elevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevation.setTargetPosition(elevation.getCurrentPosition());
                    elevation.setPower(-1);

                }
                else {
                    elevation.setPower(0);
                    // elevation.getCurrentPosition();
                    elevation.setTargetPosition(elevation.getCurrentPosition());
                    elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevation.setPower(1.0);

                }


                // slide
                if (gamepad1.dpad_up) {
                    slide.setPower(-1);
                }
                else if (gamepad1.dpad_down) {
                    slide.setPower(1);
                }
                else {
                    slide.setPower(0);
                }

                // roller
                if (gamepad1.a) {
                    roller.setPower(1000);
                }
                else {
                    roller.setPower(0);
                }

                //unroller
                if(gamepad1.b) {
                    roller.setPower(-255);
                }
                else {
                    roller.setPower(0);
                }

                // tilt
                if (gamepad1.dpad_left) {
                    tilt.setPosition(-8);
                }
                else if (gamepad1.dpad_right) {
                    tilt.setPosition(0.85);
                }


                // dump
                if (gamepad1.y) {
                    dump.setPosition(0.2);
                }
                else {
                    dump.setPosition(0.4);
                }

                FL.setPower(driveSpeed * (turn_FL_X + strafe_FL_X + strafe_FL_Y));
                FR.setPower(driveSpeed * (turn_FR_X + strafe_FR_X + strafe_FR_Y));
                BL.setPower(driveSpeed * (turn_BL_X + strafe_BL_X + strafe_BL_Y));
                BR.setPower(driveSpeed * (turn_BR_X + strafe_BR_X + strafe_BR_Y));
            }
        }
    }
}