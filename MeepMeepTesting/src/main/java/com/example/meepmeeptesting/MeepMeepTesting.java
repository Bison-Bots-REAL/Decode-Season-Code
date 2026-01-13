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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, 46, Math.toRadians(-55)))
                //.strafeTo(new Vector2d(50,15))
                //.turn(Math.toRadians(-20))
                .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                .waitSeconds(0.5)

                //.turnTo(Math.toRadians(90))
                .splineTo(new Vector2d(-11,28),Math.toRadians(90))
                //.strafeTo(new Vector2d(-11,28))
                .strafeTo(new Vector2d(-11,42))

                //.turnTo(Math.toRadians(160))
                .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                .waitSeconds(0.5)

                //.turnTo(Math.toRadians(90))
                .splineTo(new Vector2d(13,28),Math.toRadians(90))
                //.strafeTo(new Vector2d(13,28))
                .strafeTo(new Vector2d(13,42))

                //.turnTo(Math.toRadians(160))
                .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                .waitSeconds(0.5)

                //.turnTo(Math.toRadians(90))
                .splineTo(new Vector2d(36,28),Math.toRadians(90))
                //.strafeTo(new Vector2d(37,28))
                .strafeTo(new Vector2d(36,46))

                //.turnTo(Math.toRadians(160))
                .splineTo(new Vector2d(-14,15),Math.toRadians(140))
                .waitSeconds(0.5)

                .splineTo(new Vector2d(6,15),Math.toRadians(0))
                //.turnTo(Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}