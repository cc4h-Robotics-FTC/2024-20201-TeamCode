package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TickCounter extends LinearOpMode {
    private DcMotor liftMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Ticks", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
