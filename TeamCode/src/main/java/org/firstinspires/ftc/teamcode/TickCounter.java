package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TickCounter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor[] motors = {
                new Motor(hardwareMap, "frontLeftMotor"),
                new Motor(hardwareMap, "frontRightMotor"),
                new Motor(hardwareMap, "backLeftMotor"),
                new Motor(hardwareMap, "backRightMotor")
        };

        motors[1].setInverted(true);
        motors[1].encoder.setDirection(Motor.Direction.REVERSE);
        motors[3].encoder.setDirection(Motor.Direction.REVERSE);

        motors[0].resetEncoder();
        motors[1].resetEncoder();
        motors[2].resetEncoder();
        motors[3].resetEncoder();

        waitForStart();

        while (!isStopRequested()) {
            int avgTicks = (motors[0].getCurrentPosition() +
                    motors[1].getCurrentPosition() +
                    motors[2].getCurrentPosition() +
                    motors[3].getCurrentPosition()
            ) / 4;
            int absTicks = (Math.abs(motors[0].getCurrentPosition()) +
                    Math.abs(motors[1].getCurrentPosition()) +
                    Math.abs(motors[2].getCurrentPosition()) +
                    Math.abs(motors[3].getCurrentPosition())
            ) / 4;
            telemetry.addData("avgTicks", avgTicks);
            telemetry.addData("absTicks", absTicks);
            telemetry.addData("FL Ticks", motors[0].getCurrentPosition());
            telemetry.addData("FR Ticks", motors[1].getCurrentPosition());
            telemetry.addData("BL Ticks", motors[2].getCurrentPosition());
            telemetry.addData("BR Ticks", motors[3].getCurrentPosition());
            telemetry.update();
        }
    }
}
