package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Parker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx[] motors = {
                new MotorEx(hardwareMap, "frontLeftMotor"),
                new MotorEx(hardwareMap, "frontRightMotor"),
                new MotorEx(hardwareMap, "backLeftMotor"),
                new MotorEx(hardwareMap, "backRightMotor")
        };

        motors[3].setInverted(true);

        motors[0].resetEncoder();
        motors[1].resetEncoder();
        motors[2].resetEncoder();
        motors[3].resetEncoder();

        MecanumAuto robot = new MecanumAuto(
                motors,
                hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()),
                new PIDController(DriveConstants.p, DriveConstants.i, DriveConstants.d),
                DriveConstants.pt,
                DriveConstants.tpi,
                DriveConstants.tpli,
                DriveConstants.tpd,
                new Pose2d(0, 0, new Rotation2d(0))
        );

        waitForStart();

        robot.driveByIn(new Pose2d(42, 0, new Rotation2d(0)));

        telemetry.addData("Done", "Done");
        telemetry.update();
        sleep(1000);
    }
}
