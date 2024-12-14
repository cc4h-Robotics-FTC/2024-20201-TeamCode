package io.github.cc4h_robotics_ftc.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-32.0, -62.0, Math.toRadians(90.0)))
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
            .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}