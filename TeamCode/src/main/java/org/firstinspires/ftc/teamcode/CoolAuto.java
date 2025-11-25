package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Cool Auto")
@Config
public class CoolAuto extends LinearOpMode {
    public static double kp = 0;
    public static double kd = 0;



    private void turnTo(MotorGroup turn, PDController pd, RevIMU imu, double degrees) {
        boolean done = false;
        while (!done) {
            turn.set(pd.calculate(imu.getHeading(), degrees));
            done = pd.atSetPoint();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MotorGroup left = new MotorGroup(
                new MotorEx(hardwareMap, "frontLeftMotor"),
                new MotorEx(hardwareMap, "backLeftMotor")
        );
        MotorGroup right = new MotorGroup(
                new MotorEx(hardwareMap, "frontRightMotor"),
                new MotorEx(hardwareMap, "backRightMotor")
        );
        right.setInverted(true);
        MotorGroup turn = new MotorGroup(
                left,
                right
        );

        PDController pd = new PDController(kp, kd);
        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();
        imu.reset();

        waitForStart();
        while (!isStopRequested()) {
            turnTo(turn, pd, imu, 90);
            sleep(1000);
            turnTo(turn, pd, imu, 180);
            sleep(1000);
            turnTo(turn, pd, imu, 0);
        }
    }
}
