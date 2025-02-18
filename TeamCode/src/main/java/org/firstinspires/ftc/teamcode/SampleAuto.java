package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor liftMotor = new Motor(hardwareMap, "liftMotor");

        Motor armMotor = new Motor(hardwareMap, "armMotor");
        Motor wristMotor = new Motor(hardwareMap, "wristMotor");
        SimpleServo clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 180);

        liftMotor.setInverted(true);
        wristMotor.setInverted(true);

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

        clawServo.setPosition(0);
        // sleep(20000);

        armMotor.set(1.0);
        wristMotor.set(0.5);
        robot.driveByIn(new Pose2d(0, 20, new Rotation2d(0)));
        wristMotor.stopMotor();
        armMotor.stopMotor();

        wristMotor.set(0.5);
        clawServo.setPosition(0.5);
        sleep(1000);
        wristMotor.stopMotor();

        robot.driveByIn(new Pose2d(0, -60, new Rotation2d(0)));
        robot.driveByIn(new Pose2d(-10, 0, new Rotation2d(0)));

        telemetry.addData("Done", "Done");
        telemetry.update();
        sleep(1000);
    }
}
