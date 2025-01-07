package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DucksTankTeleOp extends LinearOpMode {
    static boolean ARCADE_MODE = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MotorGroup left = new MotorGroup(
                new Motor(hardwareMap, "frontLeftMotor"),
                new Motor(hardwareMap, "backLeftMotor")
        );

        MotorGroup right = new MotorGroup(
                new Motor(hardwareMap, "frontRightMotor"),
                new Motor(hardwareMap, "backRightMotor")
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