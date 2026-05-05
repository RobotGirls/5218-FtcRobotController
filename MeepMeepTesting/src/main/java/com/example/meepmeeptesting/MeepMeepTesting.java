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
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
                    //Red Bottom Auto
                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58,6,Math.toRadians(180)))
                                //moving up and shooting the pre-loaded artifacts
                                .strafeToLinearHeading(new Vector2d(54, 6), Math.toRadians(-205))
                                .waitSeconds(1)
                                //turning to the first set of balls
                                .strafeToLinearHeading(new Vector2d(36, 25), Math.toRadians(-270))
                                .waitSeconds(1)
                                //picking up the balls
                                .strafeToLinearHeading(new Vector2d(36, 45), Math.toRadians(-270))
                                //turning back and shooting the balls
                                .strafeToLinearHeading(new Vector2d(54, 6), Math.toRadians(-205))
                                .waitSeconds(1)
                                //turning to the second set of balls
                                .strafeToLinearHeading(new Vector2d(12, 28), Math.toRadians(-270))
                                .waitSeconds(1)
                                //picking up the balls
                                .strafeToLinearHeading(new Vector2d(12, 45), Math.toRadians(-270))
                                //turning back and shooting the balls
                                .strafeToLinearHeading(new Vector2d(-16, 10), Math.toRadians(-210))
                                .waitSeconds(1)
                                //park
                                .strafeToLinearHeading(new Vector2d(30, -34), Math.toRadians(-180))



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