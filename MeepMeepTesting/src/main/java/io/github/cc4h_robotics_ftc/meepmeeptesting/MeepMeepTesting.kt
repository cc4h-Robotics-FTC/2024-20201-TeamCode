package io.github.cc4h_robotics_ftc.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)

        val myBot =
            DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
                .build()

        myBot.runAction(
            myBot.drive.actionBuilder(Pose2d(-25.0, -62.0, Math.PI / 2))
                .splineTo(Vector2d(0.0, -34.0), Math.PI / 2)
                .waitSeconds(1.0) // TODO: Hang Specimen
                .splineTo(Vector2d(0.0, -54.0), Math.PI / 2)
                .splineTo(Vector2d(-48.0, -40.0), Math.PI / 2)
                .waitSeconds(1.0) // TODO: Pick Up Sample
                .splineTo(Vector2d(-52.0, -52.0), Math.PI / 4)
                .waitSeconds(1.0) // TODO: Dump Sample
                .turnTo(Math.PI / 2)
                .strafeTo(Vector2d(18.0, -60.0))
                .waitSeconds(12.0)
                .strafeTo(Vector2d(36.0, -60.0))


//                .lineToX(30.0)
//                .turn(Math.toRadians(90.0))
//                .lineToY(30.0)
//                .turn(Math.toRadians(90.0))
//                .lineToX(0.0)
//                .turn(Math.toRadians(90.0))
//                .lineToY(0.0)
//                .turn(Math.toRadians(90.0))
                .build()
        )

        meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}