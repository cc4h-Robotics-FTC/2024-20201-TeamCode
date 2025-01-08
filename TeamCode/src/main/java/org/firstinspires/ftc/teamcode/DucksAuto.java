package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class DucksAuto extends LinearOpMode {
    public static double p = 1.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double tpi = 43.577;
    public static double tpli = 39.191;
    public static double tpd = 10.0;



    @Override
    public void runOpMode() throws InterruptedException {
        Motor[] motors = {
                new Motor(hardwareMap, "frontLeftMotor"),
                new Motor(hardwareMap, "frontRightMotor"),
                new Motor(hardwareMap, "backLeftMotor"),
                new Motor(hardwareMap, "backRightMotor")
        };

        motors[1].setInverted(true);

        motors[0].resetEncoder();
        motors[1].resetEncoder();
        motors[2].resetEncoder();
        motors[3].resetEncoder();

        MecanumAuto robot = new MecanumAuto(
                motors,
                hardwareMap,
                telemetry,
                new PIDController(p, i, d),
                tpi,
                tpli,
                tpd,
                new Pose2d(0, 0, new Rotation2d(0))
        );

        waitForStart();

        Pose2d target = new Pose2d(0, 64, new Rotation2d(0));

        while (!isStopRequested()) {
            robot.forward(64);
            robot.forward(-64);
        }
    }
}
