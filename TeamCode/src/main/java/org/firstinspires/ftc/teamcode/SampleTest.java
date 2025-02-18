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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous
public class SampleTest extends LinearOpMode {
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

        robot.driveByIn(new Pose2d(32, 0, new Rotation2d(0)));
        robot.driveByIn(new Pose2d(0, 14, new Rotation2d(0)));
        armLiftMotor.set(0.5);
        sleep(2000);
        armLiftMotor.stopMotor();
        wristMotor.set(-1.0);
        sleep(500);
        wristMotor.stopMotor();
        clawServo.turnToAngle(0);
        sleep(2000);
        wristMotor.set(1.0);
        sleep(500);
        wristMotor.stopMotor();
        armLiftMotor.set(-0.5);
        sleep(2000);
        armLiftMotor.stopMotor();

        telemetry.addData("Done", "Done");
        telemetry.update();
        sleep(1000);
    }
}
