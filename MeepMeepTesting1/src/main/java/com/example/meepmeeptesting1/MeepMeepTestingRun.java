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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-52, -52, Math.toRadians(45)))

                        .strafeTo(new Vector2d(-34,-34))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(-16,-34))
                        .turn(Math.toRadians(45))
                        .strafeTo(new Vector2d(-16,-45))
                        .strafeTo(new Vector2d(-16,-16))
                        .turn(Math.toRadians(135))
                        .waitSeconds(2)
                        .strafeTo(new Vector2d(38,25))
                        .turn(Math.toRadians(45))












                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}