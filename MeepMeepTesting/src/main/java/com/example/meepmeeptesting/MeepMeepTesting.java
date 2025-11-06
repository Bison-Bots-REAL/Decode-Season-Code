package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(61, 10, Math.toRadians(180)))
                .strafeTo(new Vector2d(56,15))
                .turn(Math.toRadians(40))
                .waitSeconds(0.5)

                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(36,28))
                .strafeTo(new Vector2d(36,52))

                .turnTo(Math.toRadians(220))
                .splineToConstantHeading(new Vector2d(56,15),0)
                .waitSeconds(0.5)

                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(12,28))
                .strafeTo(new Vector2d(12,52))

                .turnTo(Math.toRadians(220))
                .splineToConstantHeading(new Vector2d(56,15),0)
                .waitSeconds(0.5)

                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(-11,28))
                .strafeTo(new Vector2d(-11,52))

                .turnTo(Math.toRadians(220))
                .splineToConstantHeading(new Vector2d(56,15),0)
                .waitSeconds(0.5)

                .turnTo(180)
                .strafeTo(new Vector2d(46,15))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}