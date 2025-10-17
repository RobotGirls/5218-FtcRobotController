package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62,10,Math.toRadians(90)))
                        .turn(Math.toRadians(180))
                        .lineToY(25)
                        .strafeTo(new Vector2d(-22,25))
                        .turn(Math.toRadians(45))
                        .waitSeconds(1.5)
                        .turn(Math.toRadians(-136))
                        .strafeTo(new Vector2d(-30,47))
                        .strafeTo(new Vector2d(-14,47))
                        .strafeTo(new Vector2d(-40,20))
                        .turn(Math.toRadians(-36))
                        .waitSeconds(1.5)
                        .strafeTo(new Vector2d(38,-22))
                        .turn(Math.toRadians(45))



                       //.turn(Math.toRadians(90))
//                        .lineToX(30)
//                        .turn(Math.toRadians(90))
//                        .lineToX(30)
//                        .turn(Math.toRadians(90))
//                        .lineToY(30)
//                        .turn(Math.toRadians(90))
                          .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}