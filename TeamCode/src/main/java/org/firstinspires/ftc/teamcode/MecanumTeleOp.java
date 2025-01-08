package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
@Config
public final class MecanumTeleOp extends LinearOpMode {
    private Motor frontLeftMotor;
    private Motor backLeftMotor;
    private Motor frontRightMotor;
    private Motor backRightMotor;
    public static boolean FIELD_CENTRIC = false;
    private RevIMU imu;

    private DcMotor liftMotor;
    private Servo dumpServo;

    private DcMotor armExtendMotor;
    private DcMotor armLiftMotor;
    private DcMotor wristMotor;
    private Servo clawServo;

    private TouchSensor liftDownSensor;
    private TouchSensor liftUpSensor;

    private boolean autoDown = false;
    private boolean autoUp = false;

    public void runOpMode() throws InterruptedException {
        frontLeftMotor = new Motor(hardwareMap, "frontLeftMotor");
        frontRightMotor = new Motor(hardwareMap, "frontRightMotor");
        backLeftMotor = new Motor(hardwareMap, "backLeftMotor");
        backRightMotor = new Motor(hardwareMap, "backRightMotor");

        imu = new RevIMU(hardwareMap);
        imu.init();

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        dumpServo = hardwareMap.servo.get("dumpServo");

        armExtendMotor = hardwareMap.dcMotor.get("armExtendMotor");
        armLiftMotor = hardwareMap.dcMotor.get("armLiftMotor");
        wristMotor = hardwareMap.dcMotor.get("wristMotor");
        clawServo = hardwareMap.servo.get("clawServo");

        liftDownSensor = hardwareMap.touchSensor.get("liftDownSensor");
        liftUpSensor = hardwareMap.touchSensor.get("liftDownSensor");

        backRightMotor.setInverted(true);
        armLiftMotor.setDirection(Direction.REVERSE);
        wristMotor.setDirection(Direction.REVERSE);

        MecanumDrive drive = new MecanumDrive(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor
        );

        waitForStart();

        if (!this.isStopRequested()) {
            while(this.opModeIsActive()) {
                double y = -this.gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

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

                if (gamepad1.a) FIELD_CENTRIC = !FIELD_CENTRIC; sleep(300);
                if (!FIELD_CENTRIC) {
                    drive.driveRobotCentric(x * mult, y * mult, rx * mult);
                } else {
                    drive.driveFieldCentric(x * mult, y * mult, rx * mult, imu.getHeading());
                }

//                frontLeftMotor.setPower(frontLeftPower * mult);
//                backLeftMotor.setPower(backLeftPower * mult);
//                frontRightMotor.setPower(frontRightPower * mult);
//                backRightMotor.setPower(backRightPower * mult);


                armExtendMotor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
                armLiftMotor.setPower(gamepad2.left_stick_y / 3.0);

                if (gamepad2.a) {
                    dumpServo.setPosition(dumpServo.getPosition() + 0.005);
                } else if (gamepad2.b) {
                    dumpServo.setPosition(dumpServo.getPosition() - 0.005);
                }

                if (gamepad2.x && clawServo.getPosition() < 1.0) {
                    clawServo.setPosition(clawServo.getPosition() + 0.005);
                } else if (gamepad2.y && clawServo.getPosition() > 0.0) {
                    clawServo.setPosition(clawServo.getPosition() - 0.005);
                }

                if (gamepad2.dpad_up) {
                    wristMotor.setPower(1.0);
                } else if (gamepad2.dpad_down) {
                    wristMotor.setPower(-1.0);
                } else {
                    wristMotor.setPower(0.0);
                }

                if (gamepad2.right_bumper) {
                    autoDown = !autoDown;
                    sleep(300);
                }

                if (gamepad2.left_bumper) {
                    autoUp = !autoUp;
                    sleep(300);
                }

                if (autoDown) {
                    liftMotor.setPower(1.0);
                } else if (autoUp) {
                    liftMotor.setPower(-1.0);
                } else {
                    liftMotor.setPower(gamepad2.right_stick_y);
                }

                if (liftDownSensor.isPressed() && autoDown) {
                    liftMotor.setPower(-0.5);
                    sleep(500);
                    liftMotor.setPower(0.0);
                    autoDown = false;
                    autoUp = false;
                }

                if (liftUpSensor.isPressed() && autoUp) {
                    liftMotor.setPower(0.5);
                    sleep(500);
                    liftMotor.setPower(0.0);
                    autoDown = false;
                    autoUp = false;
                }

                if (gamepad1.dpad_left && gamepad2.dpad_left) {
                    requestOpModeStop();
                }

                telemetry.addData("Front Left Power", frontLeftPower);
                telemetry.addData("Back Left Power", backLeftPower);
                telemetry.addData("Front Right Power", frontRightPower);
                telemetry.addData("Back Right Power", backRightPower);
                telemetry.addData("Multiplier", mult);
                telemetry.addData("Bucket Position", dumpServo.getPosition());
                telemetry.addData("Claw Position", clawServo.getPosition());
//                telemetry.addData("Lift Positoin", liftMotor.getCurrentPosition());
                telemetry.addData("Lift Power", liftMotor.getPower());
                telemetry.addData("Lift Down Sensor", liftDownSensor.isPressed());
                telemetry.addData("Lift Up Sensor", liftUpSensor.isPressed());
                telemetry.addData("Field Centric", FIELD_CENTRIC);
                telemetry.update();
            }
        }
    }
}
