package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class DucksTeleOp extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static boolean FIELD_CENTRIC = false;

    @Override
    public void runOpMode() throws InterruptedException {
        double mult = 1.0;

        Motor frontLeftMotor = new Motor(hardwareMap, "frontLeftMotor");
        Motor frontRightMotor = new Motor(hardwareMap, "frontRightMotor");
        Motor backLeftMotor = new Motor(hardwareMap, "backLeftMotor");
        Motor backRightMotor = new Motor(hardwareMap, "backRightMotor");

        Motor liftMotor = new Motor(hardwareMap, "liftMotor");
        Motor armMotor1 = new Motor(hardwareMap, "armMotor1");
        Motor armMotor2 = new Motor(hardwareMap, "armMotor2");
        Motor wristMotor = new Motor(hardwareMap, "wristMotor");
        SimpleServo clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 180);

        backRightMotor.setInverted(true);

        liftMotor.setInverted(true);
        wristMotor.setInverted(true);

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

            liftMotor.set(armOp.getRightY());
            armMotor1.set(armOp.getLeftY());
            armMotor2.set(armOp.getLeftY());

            if (armOp.isDown(GamepadKeys.Button.B)) clawServo.rotateByAngle(0.4);
            if (armOp.isDown(GamepadKeys.Button.X)) clawServo.rotateByAngle(-0.3);

            if (armOp.isDown(GamepadKeys.Button.DPAD_UP)) wristMotor.set(0.5);
            else if (armOp.isDown(GamepadKeys.Button.DPAD_DOWN)) wristMotor.set(-0.5);
            else wristMotor.set(armOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - armOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            if (driverOp.isDown(GamepadKeys.Button.X) && armOp.isDown(GamepadKeys.Button.X)) {
                drive.driveRobotCentric(-0.5, 0.0, 0.0);
                sleep(2000);
                drive.stop();

                liftMotor.set(-1.0);
                sleep(3000);
                liftMotor.stopMotor();
            }

            driverOp.readButtons();
            armOp.readButtons();

            telemetry.addData("FC", FIELD_CENTRIC);
            telemetry.addData("CP", clawServo.getPosition());
            telemetry.update();
        }
    }

}