package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DucksAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor armLiftMotor = new Motor(hardwareMap, "armLiftMotor");
        Motor wristMotor = new Motor(hardwareMap, "wristMotor");
        ServoEx clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 90);

        Motor liftMotor = new Motor(hardwareMap, "liftMotor");
        ServoEx dumpServo = new SimpleServo(hardwareMap, "dumpServo", 0, 180);

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

        armLiftMotor.set(1.0); // Put down arm
        robot.driveByIn(new Pose2d(10.0, 32.0, new Rotation2d(0.0))); // Light up with bar
        armLiftMotor.set(0.0);
        robot.driveByIn(new Pose2d(0.0, 18.0, new Rotation2d(0.0))); // Drive forward some more
        wristMotor.set(1.0); // Clip specimen
        sleep(500);
        wristMotor.set(0.0);
        clawServo.turnToAngle(45.0); // Let go of specimen
        wristMotor.set(1.0); // Put wrist down all the way
        sleep(500);
        wristMotor.set(0.0);
        robot.driveByIn(new Pose2d(-48.0, -6.0, new Rotation2d(0.0))); // Drive to first sample
        clawServo.turnToAngle(0); // Grab sample
        wristMotor.set(-1.0); // Curl up wrist
        sleep(500);
        wristMotor.set(0.0);
        armLiftMotor.set(1.0); // Lift Arm
        sleep(2000);
        armLiftMotor.set(0.0);
        clawServo.turnToAngle(45); // Drop sample into dumper
        sleep(500);
        armLiftMotor.set(-1.0); // Put down arm
        liftMotor.set(1.0); // Lift lift
        robot.driveByIn(new Pose2d(-4.0, -12.0, new Rotation2d(-45)));
        liftMotor.set(0.0);
        armLiftMotor.set(0.0);
        dumpServo.turnToAngle(180.0); // Dump sample
        liftMotor.set(-1.0); // Put down lift
        robot.driveByIn(new Pose2d(88.0, -8.0, new Rotation2d(0))); // Park
        liftMotor.set(0.0);
    }
}
