package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class DucksTankTeleOp extends LinearOpMode {
    static boolean ARCADE_MODE = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Motor frontLeftMotor = new Motor(hardwareMap, "frontLeftMotor");
        Motor frontRightMotor = new Motor(hardwareMap, "frontRightMotor");
        Motor backLeftMotor = new Motor(hardwareMap, "backLeftMotor");
        Motor backRightMotor = new Motor(hardwareMap, "backRightMotor");

        frontRightMotor.setInverted(true);

        MotorGroup left = new MotorGroup(
                frontLeftMotor,
                backLeftMotor
        );

        MotorGroup right = new MotorGroup(
                frontRightMotor,
                backRightMotor
        );

        DifferentialDrive drive = new DifferentialDrive(
                left,
                right
        );

        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            if (!ARCADE_MODE) {
                drive.tankDrive(
                        driverOp.getLeftY(),
                        driverOp.getRightY()
                );
            } else {
                drive.arcadeDrive(
                        driverOp.getLeftY(),
                        driverOp.getLeftX()
                );
            }

            if (driverOp.wasJustPressed(GamepadKeys.Button.A)) ARCADE_MODE = !ARCADE_MODE;

            driverOp.readButtons();
        }
    }
}