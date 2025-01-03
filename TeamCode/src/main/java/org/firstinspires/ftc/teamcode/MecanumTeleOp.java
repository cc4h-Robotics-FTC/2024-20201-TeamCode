package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

@TeleOp
@Config
public final class MecanumTeleOp extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotorEx liftMotor;
    private Servo dumpServo;

    private DcMotor armExtendMotor;
    private DcMotor armLiftMotor;
    private DcMotor wristMotor;
    private Servo clawServo;

    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        dumpServo = hardwareMap.servo.get("dumpServo");

        armExtendMotor = hardwareMap.dcMotor.get("armExtendMotor");
        armLiftMotor = hardwareMap.dcMotor.get("armLiftMotor");
        wristMotor = hardwareMap.dcMotor.get("wristMotor");
        clawServo = hardwareMap.servo.get("clawServo");

        backRightMotor.setDirection(Direction.REVERSE);

        armLiftMotor.setDirection(Direction.REVERSE);

        liftMotor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        int[] posList = {0, -4600, -11400};

        waitForStart();

        liftMotor.setTargetPosition(0);
        liftMotor.setMode(RunMode.RUN_TO_POSITION);
        if (!this.isStopRequested()) {
            while(this.opModeIsActive()) {
                double y = -((double)this.gamepad1.left_stick_y);
                double x = (double)this.gamepad1.left_stick_x * 1.1;
                double rx = (double)this.gamepad1.right_stick_x;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;
                double mult = Math.min(1.1 - (double)this.gamepad1.right_trigger, 1.0);
                if (this.gamepad1.left_bumper) {
                    mult = 0.5;
                } else if (this.gamepad1.right_bumper) {
                    mult = 0.25;
                }

                frontLeftMotor.setPower(frontLeftPower * mult);
                backLeftMotor.setPower(backLeftPower * mult);
                frontRightMotor.setPower(frontRightPower * mult);
                backRightMotor.setPower(backRightPower * mult);


                armExtendMotor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                armLiftMotor.setPower(gamepad2.left_stick_y / 3.0);

                if (gamepad2.a) {
                    dumpServo.setPosition(dumpServo.getPosition() + 0.005);
                } else if (gamepad2.b) {
                    dumpServo.setPosition(dumpServo.getPosition() - 0.005);
                }

                if (gamepad2.x && clawServo.getPosition() < 1.0) {
                    clawServo.setPosition(clawServo.getPosition() + 0.001);
                } else if (gamepad2.y && clawServo.getPosition() > 0.0) {
                    clawServo.setPosition(clawServo.getPosition() - 0.001);
                }

                if (gamepad2.dpad_up) {
                    wristMotor.setPower(1.0);
                } else if (gamepad2.dpad_down) {
                    wristMotor.setPower(-1.0);
                } else {
                    wristMotor.setPower(0.0);
                }

                if (gamepad2.dpad_left) {
                    liftMotor.setTargetPosition(posList[1]);
                    liftMotor.setMode(RunMode.RUN_TO_POSITION);
                } else if (gamepad2.dpad_right) {
                    liftMotor.setTargetPosition(posList[2]);
                    liftMotor.setMode(RunMode.RUN_TO_POSITION);
                } else if (gamepad2.right_bumper) {
                    liftMotor.setTargetPosition(posList[0]);
                    liftMotor.setMode(RunMode.RUN_TO_POSITION);
                }

                liftMotor.setTargetPosition(liftMotor.getTargetPosition() + (int)((double)gamepad2.right_stick_y / 0.1));
                if (gamepad2.left_bumper) {
                    liftMotor.setVelocity(1.0 - liftMotor.getPower());
                    sleep(300L);
                }

                if (liftMotor.getCurrentPosition() == liftMotor.getTargetPosition()) {
                    liftMotor.setVelocity(0.0);
                }

                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);
                telemetry.addData("Multiplier", mult);
                telemetry.addData("Bucket Position", dumpServo.getPosition());
                telemetry.addData("Lift Positoin", liftMotor.getCurrentPosition());
                telemetry.addData("Lift Power", liftMotor.getVelocity());
                telemetry.update();
            }
        }
    }
}
