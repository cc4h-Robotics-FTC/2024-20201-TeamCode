package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class SpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor liftMotor = new Motor(hardwareMap, "liftMotor");

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

        MecanumDrive drive = new MecanumDrive(
                motors[0],
                motors[1],
                motors[2],
                motors[3]
        );

        MecanumAuto robot = new MecanumAuto(
                motors,
                hardwareMap,
                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()),
                new PIDController(DriveConstants.p, DriveConstants.i, DriveConstants.d),
                100,
                DriveConstants.tpi,
                DriveConstants.tpli,
                DriveConstants.tpd,
                new Pose2d(0, 0, new Rotation2d(0))
        );

        waitForStart();

        // sleep(20000);

        liftMotor.set(1.0);
        sleep(7000);
        liftMotor.stopMotor();

        drive.driveRobotCentric(-0.5, 0.0, 0.0);
        sleep(2500);
        drive.stop();

        drive.driveRobotCentric(0.5, 0.0, 0.0);
        sleep(100);
        drive.stop();

        liftMotor.set(-1.0);
        sleep(2000);
        liftMotor.stopMotor();

        drive.driveRobotCentric(0.5, -0.5, 0.0);
        sleep(2000);
        drive.stop();

//        robot.driveByIn(new Pose2d(-20, 0, new Rotation2d(0)));
//        robot.driveByIn(new Pose2d(0, 48, new Rotation2d(0)));
//        robot.driveByIn(new Pose2d(-10, 0, new Rotation2d(0)));

        telemetry.addData("Done", "Done");
        telemetry.update();
        sleep(1000);
    }
}
