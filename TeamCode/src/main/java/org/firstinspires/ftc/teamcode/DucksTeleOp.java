package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@TeleOp
public class DucksTeleOp extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    public static boolean FIELD_CENTRIC = false;
    public static double liftMotorKP = 1;
    public static double liftArmMotorKP = 0.03;
    public static double armMotorKP = 0.03;
    public static double liftMotorTarget = 0;
    public static double liftArmMotorTarget = 0;
    public static double armMotorTarget = 0;

//    public static int clawOutLiftArmClawServoPosition = 90;
//    public static int clawOutArmClawPosition = 120;
//    public static int clawOutLiftPositon = 2000;
//    public static int clawOutLiftArmPositon = 0;
//    public static int clawOutArmPositon = 20;
//
//
//    public static int clawInArmClawClosePosition = 0;
//    public static int clawInArmLiftClawOpenPosition = 0;
//
//    public static int clawInLiftUpPositon = 2000;
//    public static int clawInLiftArmUpPositon = 0;
//    public static int clawInLiftUpDelay = 500;
//
//    public static int clawInLiftDownPositon = 500;
//    public static int clawInLiftArmDownPositon = 0;
//    public static int clawInLiftDownDelay = 500;
//    public static int clawInArmLiftClawClosePosition = 0;
//
//    public static int clawInArmClawOpenPosition = 90;
//    public static int clawInArmClawOpenDelay = 500;
//    public static int clawInArmPositon = 0;

    public static int armInPosition = 0;
    public static int armOutPosition = -140;
    public static int armClawClosePosition = 0;
    public static int armClawOpenPosition = 90;
    public static int liftArmDownPositon = 0;
    public static int liftArmUpPositon = -90;
    public static int liftArmClawClosePosition = 180;
    public static int liftArmClawOpenPosition = 90;
    public static int liftUpPosition = -4500;
    public static int liftMiddlePosition = -3500;
    public static int liftDownPosition = 0;
    public static int servoDelay = 1000;

    private static MotorEx liftMotor;
    private static MotorEx liftArmMotor;
    private static MotorEx armMotor;
    private static ServoEx liftArmClawServo;
    private static ServoEx armClawServo;

    @Override
    public void runOpMode() throws InterruptedException {
        double mult = 1.0;

        MotorEx frontLeftMotor = new MotorEx(hardwareMap, "frontLeftMotor");
        MotorEx frontRightMotor = new MotorEx(hardwareMap, "frontRightMotor");
        MotorEx backLeftMotor = new MotorEx(hardwareMap, "backLeftMotor");
        MotorEx backRightMotor = new MotorEx(hardwareMap, "backRightMotor");

        backRightMotor.setInverted(true);

        liftMotor = new MotorEx(hardwareMap, "liftMotor");
        liftArmMotor = new MotorEx(hardwareMap, "liftArmMotor");
        armMotor = new MotorEx(hardwareMap, "armMotor");
        liftArmClawServo = new SimpleServo(hardwareMap, "liftArmClawServo", 0, 180);
        armClawServo = new SimpleServo(hardwareMap, "armClawServo", 0, 180);

        liftMotor.setRunMode(Motor.RunMode.PositionControl);
        liftArmMotor.setRunMode(Motor.RunMode.PositionControl);
        armMotor.setRunMode(Motor.RunMode.PositionControl);


        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftArmMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        liftMotor.stopAndResetEncoder();
        liftArmMotor.stopAndResetEncoder();
        armMotor.stopAndResetEncoder();

        liftMotor.setPositionCoefficient(liftMotorKP);
        liftArmMotor.setPositionCoefficient(liftArmMotorKP);
        armMotor.setPositionCoefficient(armMotorKP);


        liftMotor.setInverted(true);

        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        MecanumDrive drive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor
        );

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /        /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx armOp = new GamepadEx(gamepad2);

        waitForStart();

        while (!isStopRequested()) {

            // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
            // These are related to the left stick x value, left stick y value, and
            // right stick x value respectively. These values are passed in to represent the
            // strafing speed, the forward speed, and the turning speed of the robot frame
            // respectively from [-1, 1].

            if (!FIELD_CENTRIC) {

                // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
                // will move the robot in the direction of its current heading. Every movement
                // is relative to the frame of the robot itself.
                //
                //                 (0,1,0)
                //                   /
                //                  /
                //           ______/_____
                //          /           /
                //         /           /
                //        /___________/
                //           ____________
                //          /  (0,0,1)  /
                //         /     ↻     /
                //        /___________/

                // optional fourth parameter for squared inputs
                drive.driveRobotCentric(
                        -driverOp.getRightX() * mult,
                        -driverOp.getRightY() * mult,
                        driverOp.getLeftX() * mult,
                        false
                );
            } else {

                // Below is a model for how field centric will drive when given the inputs
                // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
                // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
                // regardless of the heading.
                //
                //                   heading
                //                     /
                //            (0,1,0) /
                //               |   /
                //               |  /
                //            ___|_/_____
                //          /           /
                //         /           / ---------- (1,0,0)
                //        /__________ /

                // optional fifth parameter for squared inputs
                drive.driveFieldCentric(
                        -driverOp.getRightX() * mult,
                        -driverOp.getRightY() * mult,
                        driverOp.getLeftX() * mult,
                        imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                        false
                );
            }

            if (driverOp.wasJustPressed(GamepadKeys.Button.A) && driverOp.isDown(GamepadKeys.Button.Y)) FIELD_CENTRIC = !FIELD_CENTRIC;
            if (driverOp.wasJustPressed(GamepadKeys.Button.B) && driverOp.isDown(GamepadKeys.Button.Y)) imu.reset();

            if (driverOp.isDown(GamepadKeys.Button.LEFT_BUMPER)) mult = 0.5;
            else if (driverOp.isDown(GamepadKeys.Button.RIGHT_BUMPER)) mult = 0.25;
            else mult = 1.0;

//            liftMotor.set(armOp.getRightY());
//            armMotor1.set(armOp.getLeftY());
//            armMotor2.set(armOp.getLeftY());
//
//            if (armOp.isDown(GamepadKeys.Button.B)) clawServo.rotateByAngle(0.4);
//            if (armOp.isDown(GamepadKeys.Button.X)) clawServo.rotateByAngle(-0.3);
//
//            if (armOp.isDown(GamepadKeys.Button.DPAD_UP)) wristMotor.set(0.5);
//            else if (armOp.isDown(GamepadKeys.Button.DPAD_DOWN)) wristMotor.set(-0.5);
//            else wristMotor.set(armOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - armOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//
//            if (driverOp.isDown(GamepadKeys.Button.X) && armOp.isDown(GamepadKeys.Button.X)) {
//                drive.driveRobotCentric(-0.5, 0.0, 0.0);
//                sleep(2000);
//                drive.stop();
//
//                liftMotor.set(-1.0);
//                sleep(3000);
//                liftMotor.stopMotor();
//            }

            liftMotor.setPositionCoefficient(liftMotorKP);
            liftArmMotor.setPositionCoefficient(liftArmMotorKP);
            armMotor.setPositionCoefficient(armMotorKP);


            // Was DPAD Up Pressed?
            if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                // If so put claw out and put lift in ready position
                clawOut();
            } else if (driverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                clawIn();
            } else if (driverOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                liftMotor.setTargetPosition(liftMiddlePosition);
                while (!liftMotor.atTargetPosition()) {
                    liftMotor.set(1.0);
                }
                liftArmMotor.setTargetPosition(liftArmUpPositon);
                while (!liftArmMotor.atTargetPosition()) {
                    liftArmMotor.set(1.0);
                }
//            } else if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                liftMotor.setTargetPosition(liftUpPosition);
//                while (!liftMotor.atTargetPosition()) {
//                    liftMotor.set(1.0);
//                }
//                liftArmMotor.setTargetPosition(liftArmUpPositon);
//                while (!liftArmMotor.atTargetPosition()) {
//                    liftArmMotor.set(1.0);
//                }
            } else if (driverOp.wasJustPressed(GamepadKeys.Button.A)) {
                liftArmClawServo.turnToAngle(liftArmClawOpenPosition);
                sleep(100);
                sleep(servoDelay - 100);
            }


            driverOp.readButtons();
            armOp.readButtons();

            telemetry.addData("FIELD_CENTRIC", FIELD_CENTRIC);
            // Current position for servos
            telemetry.addData("armClawServoPosition", armClawServo.getPosition());
            telemetry.addData("liftArmClawServoPosition", liftArmClawServo.getPosition());
            // Current position for motors
            telemetry.addData("liftMotorCurrentPosition", liftMotor.getCurrentPosition());
            telemetry.addData("liftArmMotorCurrentPosition", liftArmMotor.getCurrentPosition());
            telemetry.addData("armMotorCurrentPosition", armMotor.getCurrentPosition());
            // Target position for motors
            telemetry.addData("liftMotorTargetPosition", liftMotorTarget);
            telemetry.addData("liftArmMotorTargetPosition", liftArmMotorTarget);
            telemetry.addData("armMotorTargetPosition", armMotorTarget);
            telemetry.update();
        }
    }
//    private void clawOut() {
//        // Set liftArmClawServo position
//        liftArmClawServo.setPosition(clawOutLiftArmClawServoPosition);
//        sleep(1000);
//
//        // Set lift and liftArm Position
//        liftMotor.setTargetPosition(clawOutLiftPositon);
//        liftArmMotor.setTargetPosition(clawOutLiftArmPositon);
//        while (!liftMotor.atTargetPosition() || !liftArmMotor.atTargetPosition()) {
//            if (!liftMotor.atTargetPosition()) {
//                liftMotor.set(1);
//            }
//            if (!liftArmMotor.atTargetPosition()) {
//                liftArmMotor.set(1);
//            }
//        }
//        liftMotor.set(0);
//        liftArmMotor.set(0);
//
//        // Set armMotor position
//        armMotor.set(clawOutArmPositon);
//        while (!armMotor.atTargetPosition()) {
//            armMotor.set(1);
//        }
//        armMotor.set(0);
//
//        // Set armClawServo position
//        armClawServo.setPosition(clawOutArmClawPosition);
//        sleep(1000);
//    }
//
//    private void clawIn() {
//        // Set armClaw and liftArmClaw position
//        armClawServo.setPosition(clawInArmClawClosePosition);
//        liftArmClawServo.setPosition(clawInArmLiftClawOpenPosition);
//
//        // Set lift and liftArm up position
//        liftMotor.setTargetPosition(clawInLiftUpPositon);
//        liftArmMotor.setTargetPosition(clawInLiftArmUpPositon);
//        while (!liftMotor.atTargetPosition() || !liftArmMotor.atTargetPosition()) {
//            if (!liftMotor.atTargetPosition()) {
//                liftMotor.set(1);
//            }
//            if (!liftArmMotor.atTargetPosition()) {
//                liftArmMotor.set(1);
//            }
//        }
//        sleep(clawInLiftUpDelay);
//        liftMotor.set(0);
//        liftArmMotor.set(0);
//
//        armMotor.set(clawInArmPositon);
//        while (!armMotor.atTargetPosition()) {
//            armMotor.set(1);
//        }
//        armMotor.set(0);
//
//        // Set lift and liftArm Down Position
//        liftMotor.setTargetPosition(clawInLiftDownPositon);
//        liftArmMotor.setTargetPosition(clawInLiftArmDownPositon);
//        while (!liftMotor.atTargetPosition() || !liftArmMotor.atTargetPosition()) {
//            if (!liftMotor.atTargetPosition()) {
//                liftMotor.set(1);
//            }
//            if (!liftArmMotor.atTargetPosition()) {
//                liftArmMotor.set(1);
//            }
//        }
//        sleep(clawInLiftDownDelay);
//        liftMotor.set(0);
//        liftArmMotor.set(0);
//
//        armClawServo.setPosition(clawInArmClawOpenPosition);
//        liftArmClawServo.setPosition(clawInArmLiftClawClosePosition);
//        sleep(clawInArmClawOpenDelay);
//    }
    public void clawOut() {
        armClawServo.turnToAngle(armClawOpenPosition);
        sleep(100);
        sleep(servoDelay - 100);
        liftArmClawServo.turnToAngle(liftArmClawOpenPosition);

        liftArmMotor.setTargetPosition(liftArmUpPositon);
        while (!liftArmMotor.atTargetPosition()) {
            liftArmMotor.set(1.0);
        }
        sleep(100);

        armMotor.setTargetPosition(armOutPosition);
        while (!armMotor.atTargetPosition()) {
            armMotor.set(1.0);
        }
    }
    public void clawIn() {
        armClawServo.turnToAngle(armClawClosePosition);
        liftArmClawServo.turnToAngle(liftArmClawOpenPosition);
        sleep(100);
        sleep(servoDelay - 100);

        liftArmMotor.setTargetPosition(liftArmDownPositon);
        while (!liftArmMotor.atTargetPosition()) {
            liftArmMotor.set(1.0);
        }

        liftMotor.setTargetPosition(liftDownPosition);
        while (!liftMotor.atTargetPosition()) {
            liftMotor.set(1.0);
        }

        liftArmMotor.setTargetPosition(liftArmUpPositon);
        while (!liftArmMotor.atTargetPosition()) {
            liftArmMotor.set(1.0);
        }

        armMotor.setTargetPosition(armInPosition);
        while (!armMotor.atTargetPosition()) {
            armMotor.set(1.0);
        }

        liftArmMotor.setTargetPosition(liftArmDownPositon);
        while (!liftArmMotor.atTargetPosition()) {
            liftArmMotor.set(1.0);
        }

        liftArmClawServo.turnToAngle(liftArmClawClosePosition);
        sleep(100);
        sleep(servoDelay - 100);
        armClawServo.turnToAngle(armClawOpenPosition);
        sleep(100);
        sleep(servoDelay - 100);
    }
}