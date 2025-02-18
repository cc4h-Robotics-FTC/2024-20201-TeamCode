package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx motor = new MotorEx(hardwareMap, "frontLeftMotor");
        motor.resetEncoder();
        double targetPos = 1000000;

        waitForStart();

        while(!isStopRequested()) {
            motor.set(Math.sin(motor.getCurrentPosition() / targetPos));
//            if (target - motor.getCurrentPosition() <= 5 && motor.getVelocity() <= 1) { motor.setVelocity(0); target += 1000; sleep(5000); }
            telemetry.addData("pos", motor.getCurrentPosition());
            telemetry.addData("pi", "%.100f",motor.getCurrentPosition() / targetPos);
            telemetry.addData("vel", motor.getVelocity());
            telemetry.addData("pwr", motor.get());
            telemetry.update();
        }
    }
}
