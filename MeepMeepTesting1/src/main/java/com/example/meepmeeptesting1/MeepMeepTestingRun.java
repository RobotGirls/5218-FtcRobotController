package com.example.meepmeeptesting1;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingRun {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, 50, Math.toRadians(130)))

                .strafeToLinearHeading(new Vector2d(-14,20),Math.toRadians(145))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-14,35),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-14,49),Math.toRadians(90))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(-14,20),Math.toRadians(145))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(35,-20),Math.toRadians(90))











                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}