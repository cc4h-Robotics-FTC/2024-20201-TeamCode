package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public final class DucksAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPos = new Pose2d(-32.0, -62.0, Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);
        Action traj = drive.actionBuilder(startPos)
                .strafeTo(new Vector2d(0.0, -52.0)) // Get into position
                .strafeTo(new Vector2d(0.0, -34.0)) // Ram specimin into bar
                .strafeToSplineHeading(new Vector2d(-48.0, -40.0), Math.toRadians(90.0)) // Line up with first sample
                .waitSeconds(1.0) // TODO: Pick Up Sample
                .strafeToSplineHeading(new Vector2d(-52.0, -52.0), Math.toRadians(45.0)) // Line up with basket
                .waitSeconds(1.0) // TODO: Dump Sample
                .strafeToSplineHeading(new Vector2d(-58.5, -40.0), Math.toRadians(90.0))
                .waitSeconds(1.0) // TODO: Pick up second sample
                .strafeToSplineHeading(new Vector2d(-52.0, -52.0), Math.toRadians(45.0))
                .waitSeconds(1.0) // TODO: Dump second sample
                .strafeToSplineHeading(new Vector2d(-52.0, -10.0), Math.toRadians(90.0))
                .strafeTo(new Vector2d(-61.0, -10.0))
                .strafeTo(new Vector2d(-61.0, -52.0))
                .waitSeconds(10.0)
                .strafeTo(new Vector2d(36.0, -60.0)) // Parking
                .build();
        Actions.runBlocking(traj);
    }
}
