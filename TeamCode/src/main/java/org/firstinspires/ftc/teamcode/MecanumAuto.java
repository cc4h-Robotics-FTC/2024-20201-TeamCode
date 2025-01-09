package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MecanumAuto {
//    private MecanumDrive drive;
    private MotorEx[] motors;
    private HardwareMap hw;
    private Telemetry telemetry;
    private RevIMU imu;
    private PIDController pid;
    private double ticksPerInch;
    private double ticksPerLateralInch;
    private double ticksPerDegree;
    private Pose2d pos;

    public MecanumAuto(MotorEx[] motors, HardwareMap hw, Telemetry telemetry, PIDController pid, double ticksPerInch, double ticksPerLateralInch, double ticksPerDegree, Pose2d startPos) {
        this.motors = motors;
//        this.motors[0].setRunMode(Motor.RunMode.VelocityControl);
//        this.motors[1].setRunMode(Motor.RunMode.VelocityControl);
//        this.motors[2].setRunMode(Motor.RunMode.VelocityControl);
//        this.motors[3].setRunMode(Motor.RunMode.VelocityControl);
        this.hw = hw;
        this.imu = new RevIMU(this.hw);
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

    public void driveByTicks(Pose2d pos) {
        double[] targets = {
                motors[0].getCurrentPosition() + pos.getY() + pos.getX() + pos.getHeading(),
                motors[1].getCurrentPosition() + pos.getY() - pos.getX() - pos.getHeading(),
                motors[2].getCurrentPosition() + pos.getY() - pos.getX() + pos.getHeading(),
                motors[3].getCurrentPosition() + pos.getY() + pos.getX() - pos.getHeading()
        };

        boolean[] motorsDone = {
                false,
                false,
                false,
                false
        };
        boolean done = false;

        double avgError = Math.abs(
                targets[0] - motors[0].getCurrentPosition() +
                        targets[1] - motors[1].getCurrentPosition() +
                        targets[2] - motors[2].getCurrentPosition() +
                        targets[3] - motors[3].getCurrentPosition()
        ) / 4;

        while (!done) {
//            pid.setSetPoint(targets[0]);
            motors[0].setVelocity(pid.calculate(motors[0].getCurrentPosition(), targets[0]));
            motorsDone[0] = pid.atSetPoint();

//            pid.setSetPoint(targets[1]);
            motors[1].setVelocity(pid.calculate(motors[1].getCurrentPosition(), targets[1]));
            motorsDone[1] = pid.atSetPoint();

//            pid.setSetPoint(targets[2]);
            motors[2].setVelocity(pid.calculate(motors[2].getCurrentPosition(), targets[2]));
            motorsDone[2] = pid.atSetPoint();

//            pid.setSetPoint(targets[3]);
            motors[3].setVelocity(pid.calculate(motors[3].getCurrentPosition(), targets[3]));
            motorsDone[3] = pid.atSetPoint();

            done = motorsDone[0] && motorsDone[1] && motorsDone[2] && motorsDone[3];

            avgError = Math.abs(
                    targets[0] - motors[0].getCurrentPosition() +
                            targets[1] - motors[1].getCurrentPosition() +
                            targets[2] - motors[2].getCurrentPosition() +
                            targets[3] - motors[3].getCurrentPosition()
            ) / 4;
            telemetry.addData("avgError", avgError);
            telemetry.addData("FL target", targets[0]);
            telemetry.addData("FR target", targets[1]);
            telemetry.addData("BL target", targets[2]);
            telemetry.addData("BR target", targets[3]);
            telemetry.addData("FL position", motors[0].getCurrentPosition());
            telemetry.addData("FR position", motors[1].getCurrentPosition());
            telemetry.addData("BL position", motors[2].getCurrentPosition());
            telemetry.addData("BR position", motors[0].getCurrentPosition());
            telemetry.addData("FL error", motors[0].getCurrentPosition() - targets[0]);
            telemetry.addData("FR error", motors[1].getCurrentPosition() - targets[1]);
            telemetry.addData("BL error", motors[2].getCurrentPosition() - targets[2]);
            telemetry.addData("BR error", motors[3].getCurrentPosition() - targets[3]);
            telemetry.addData("FL velocity", motors[0].getVelocity());
            telemetry.addData("FR velocity", motors[1].getVelocity());
            telemetry.addData("BL velocity", motors[2].getVelocity());
            telemetry.addData("BR velocity", motors[3].getVelocity());
            telemetry.addData("FL done", motorsDone[0]);
            telemetry.addData("FR done", motorsDone[1]);
            telemetry.addData("BL done", motorsDone[2]);
            telemetry.addData("BR done", motorsDone[3]);
            telemetry.update();
        }
    }

    public void driveByIn(Pose2d pos) {
        driveByTicks(new Pose2d(pos.getX() * ticksPerInch, pos.getY() * ticksPerLateralInch, new Rotation2d(pos.getHeading() * ticksPerDegree)));
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
            motors[0].setVelocity(pid.calculate(motors[0].getCurrentPosition(), targets[0]));
            motors[1].setVelocity(pid.calculate(motors[1].getCurrentPosition(), targets[1]));
            motors[2].setVelocity(pid.calculate(motors[2].getCurrentPosition(), targets[2]));
            motors[3].setVelocity(pid.calculate(motors[3].getCurrentPosition(), targets[3]));

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
