package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@TeleOp
class MecanumTeleOp : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        val frontLeftMotor = hardwareMap.dcMotor["frontLeftMotor"]
        val backLeftMotor = hardwareMap.dcMotor["backLeftMotor"]
        val frontRightMotor = hardwareMap.dcMotor["frontRightMotor"]
        val backRightMotor = hardwareMap.dcMotor["backRightMotor"]

        val liftMotor = hardwareMap.dcMotor["liftMotor"]
        val dumpServo = hardwareMap.servo["dumpServo"]

        val armExtendMotor = hardwareMap.dcMotor["armExtendMotor"]
        val armLiftLeftMotor = hardwareMap.dcMotor["armLiftLeftMotor"]
        val armLiftRightMotor = hardwareMap.dcMotor["armLiftRightMotor"]

        val wristServo = hardwareMap.crservo["wristServo"]
        val clawServo = hardwareMap.servo["clawServo"]

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        backRightMotor.direction = DcMotorSimple.Direction.REVERSE

        armLiftRightMotor.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        if (isStopRequested) return

        while (opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick value is reversed
            val x = gamepad1.left_stick_x * 1.1 // Counteract imperfect strafing
            val rx = gamepad1.right_stick_x.toDouble()

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
            val frontLeftPower = (y + x + rx) / denominator
            val backLeftPower = (y - x + rx) / denominator
            val frontRightPower = (y - x - rx) / denominator
            val backRightPower = (y + x - rx) / denominator

            var mult = min(1.1 - gamepad1.right_trigger.toDouble(), 1.0)
            if (gamepad1.left_bumper) mult = 0.5
            else if (gamepad1.right_bumper) mult = 0.25

            frontLeftMotor.power = frontLeftPower * mult
            backLeftMotor.power = backLeftPower * mult
            frontRightMotor.power = frontRightPower * mult
            backRightMotor.power = backRightPower * mult

            liftMotor.power = gamepad2.right_stick_y.toDouble() / if (gamepad2.right_stick_y < 0) 1 else 5
            armExtendMotor.power = gamepad2.left_trigger.toDouble() - gamepad2.right_trigger.toDouble()
            armLiftLeftMotor.power = gamepad2.left_stick_y.toDouble()
            armLiftRightMotor.power = gamepad2.left_stick_y.toDouble()

            if (gamepad2.a) {
                dumpServo.position += 0.005
            } else if (gamepad2.b) {
                dumpServo.position -= 0.005
            }

            if (gamepad2.x && clawServo.position < 1.0) {
                clawServo.position += 0.001
            } else if (gamepad2.y && clawServo.position > 0.0) {
                clawServo.position -= 0.001
            }

            if (gamepad2.dpad_up) {
                wristServo.power = 1.0
            } else if (gamepad2.dpad_down) {
                wristServo.power = -1.0
            } else {
                wristServo.power = 0.0
            }

            telemetry.addData("Front Left Power", frontLeftPower)
            telemetry.addData("Back Left Power", backLeftPower)
            telemetry.addData("Front Right Power", frontRightPower)
            telemetry.addData("Back Right Power", backRightPower)
            telemetry.addData("Multiplier", mult)
            telemetry.addData("Position", dumpServo.position)
            telemetry.update()
        }
    }
}
