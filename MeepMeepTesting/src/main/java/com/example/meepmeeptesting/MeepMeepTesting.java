package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.LinearHeadingPath;
import com.acmerobotics.roadrunner.Pose2d;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[]0.001992958214309 args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
                    //Red Bottom Auto
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60,10,Math.toRadians(180)))

                                .strafeToLinearHeading(new Vector2d(-22,25),Math.toRadians(145))
                                .waitSeconds(1.5)

                                 .strafeToLinearHeading(new Vector2d(-14,35),Math.toRadians(90))

                                .strafeToLinearHeading(new Vector2d(-14,50),Math.toRadians(90))
                                .waitSeconds(1.5)
                                 .strafeToLinearHeading(new Vector2d(-22,25),Math.toRadians(145))
                                    .waitSeconds(1.5)


                                        .strafeToLinearHeading(new Vector2d(26,-20),Math.toRadians(90))

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