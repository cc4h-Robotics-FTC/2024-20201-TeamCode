package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MecanumAuto {
    private MecanumDrive drive;
    private Motor[] motors;
    private HardwareMap hw;
    private Telemetry telemetry;
    private RevIMU imu;
    private PIDController pid;
    private double ticksPerInch;
    private double ticksPerLateralInch;
    private double ticksPerDegree;
    private Pose2d pos;

    public MecanumAuto(Motor[] motors, HardwareMap hw, Telemetry telemetry, PIDController pid, double ticksPerInch, double ticksPerLateralInch, double ticksPerDegree, Pose2d startPos) {
        this.motors = motors;
        this.drive = new MecanumDrive(
                this.motors[0],
                this.motors[1],
                this.motors[2],
                this.motors[3]
        );
        this.hw = hw;
        this.imu = new RevIMU(hw);
        this.telemetry = telemetry;
        this.pid = pid;
        this.ticksPerInch = ticksPerInch;
        this.ticksPerLateralInch = ticksPerLateralInch;
        this.ticksPerDegree = ticksPerDegree;
        this.pos = startPos;
    }

    public void updateRotation() {
        pos = new Pose2d(pos.getX(), pos.getY(), imu.getRotation2d());
    }

    public void drive(Pose2d pos) {
        int avgTicks = (Math.abs(motors[0].getCurrentPosition()) +
                Math.abs(motors[1].getCurrentPosition()) +
                Math.abs(motors[2].getCurrentPosition()) +
                Math.abs(motors[3].getCurrentPosition())
        ) / 4;
        drive.driveRobotCentric(
                pid.calculate(avgTicks, avgTicks + pos.getX() * ticksPerLateralInch),
                pid.calculate(avgTicks, avgTicks + pos.getY() * ticksPerInch),
                pid.calculate(pos.getHeading(), pos.getHeading() * ticksPerDegree)
        );
    }

    public void forward(double in) {
        int avgTicks = (
                motors[0].getCurrentPosition() +
                motors[1].getCurrentPosition() +
                motors[2].getCurrentPosition() +
                motors[3].getCurrentPosition()
        ) / 4;
        double avgTarget = avgTicks + in * ticksPerInch;
        double[] targets = {
                motors[0].getCurrentPosition() + in * ticksPerInch,
                motors[1].getCurrentPosition() + in * ticksPerInch,
                motors[2].getCurrentPosition() + in * ticksPerInch,
                motors[3].getCurrentPosition() + in * ticksPerInch
        };

        double avgError = Math.abs(
                targets[0] - motors[0].getCurrentPosition() +
                targets[1] - motors[1].getCurrentPosition() +
                targets[2] - motors[2].getCurrentPosition() +
                targets[3] - motors[3].getCurrentPosition()
        ) / 4;

        while (avgError > 3) {
            drive.driveWithMotorPowers(
                    pid.calculate(motors[0].getCurrentPosition(), targets[0]),
                    pid.calculate(motors[1].getCurrentPosition(), targets[1]),
                    pid.calculate(motors[2].getCurrentPosition(), targets[2]),
                    pid.calculate(motors[3].getCurrentPosition(), targets[3])
            );
            avgTicks = (
                    motors[0].getCurrentPosition() +
                    motors[1].getCurrentPosition() +
                    motors[2].getCurrentPosition() +
                    motors[3].getCurrentPosition()
            ) / 4;
            avgError = Math.abs(
                    targets[0] - motors[0].getCurrentPosition() +
                    targets[1] - motors[1].getCurrentPosition() +
                    targets[2] - motors[2].getCurrentPosition() +
                    targets[3] - motors[3].getCurrentPosition()
            ) / 4;
            telemetry.addData("avgTarget", avgTarget);
            telemetry.addData("avgTicks", avgTicks);
            telemetry.update();
        }
    }
}
