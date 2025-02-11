package com.example.mepmeeptesting;

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
                .setConstraints(70, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 65, 300))
        // ^ positions for specimen auto

                .lineToY(36)
                //raise lift
                .lineToY(45)
                //lower lift and open claw
                .splineTo(new Vector2d(-35,35),-300)
                .strafeTo(new Vector2d(-35,10))
                .strafeTo(new Vector2d(-45,10))
                .strafeTo(new Vector2d(-45,58))
                .lineToY(45)
                .turn(3.2)
                .lineToY(60)
                //close claw
                .waitSeconds(0.09)
                .lineToY(58)
                .splineTo(new Vector2d(0,48),-300)
                //raise lift
                .lineToY(36)
                .lineToY(45)
//                .waitSeconds(0.5)
//                //lower lift
//                .splineTo(new Vector2d(-50,50),300)
//                .lineToY(58)                                       2 specimines
                .splineTo(new Vector2d(-40,36),300)
                .splineTo(new Vector2d(-55,10),-300)
                .strafeTo(new Vector2d(-55,58))
                .lineToY(45)
                .turn(3.1)
                .lineToY(60)
                .waitSeconds(0.09)
                .lineToY(58)
                .splineTo(new Vector2d(0,50),-300)
                //raise lift
                .lineToY(36)
                .lineToY(45)
                .splineTo(new Vector2d(-50,55),-300)


                //with spinning

//
//                .lineToY(36)
//                //raise lift
//                .lineToY(45)
//                //lower lift and open claw
//                .strafeTo(new Vector2d(-35,35))
//                .strafeTo(new Vector2d(-35,10))
//                .strafeTo(new Vector2d(-45,10))
//                .strafeTo(new Vector2d(-45,58))
//                .lineToY(45)
//                .waitSeconds(0.2)
//                .lineToY(60)
//                //close claw
//                .waitSeconds(0.2)
//                .lineToY(58)
//                .splineTo(new Vector2d(0,48),300)
                //raise lift
//                .lineToY(36)
//                .lineToY(45)
//                //2 specimines
//                .splineTo(new Vector2d(-35,36),-300)
//                .strafeTo(new Vector2d(-35,10))
//                .strafeTo(new Vector2d(-55,10))
//                .strafeTo(new Vector2d(-55,58))
//                .lineToY(45)
//                .waitSeconds(0.2)
//                .lineToY(60)
//                .waitSeconds(0.2)
//                .lineToY(58)
//                .splineTo(new Vector2d(0,50),300)
//                //raise lift
//                .lineToY(36)
//                .lineToY(45)
//                .splineTo(new Vector2d(-50,45),-300)
//                .lineToY(58)
//  // without spinning


//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(32, 62, 0))
//
//                .lineToX(45)
//                //raise lift
//                .splineTo(new Vector2d(56,50),20)
//                .waitSeconds(1)
//                //open claw
//                .splineTo(new Vector2d(48,40),300)
//                //grab sample
//                .waitSeconds(1)
//                .splineTo(new Vector2d(56,50),20)
//                //raise lift
//                .waitSeconds(1)
//                //open claw
//                .splineTo(new Vector2d(57,40),300)
//                //grab sample
//                .waitSeconds(1)
//                .splineTo(new Vector2d(56,51),20)
//                .waitSeconds(1)
//                //open claw
//                .splineTo(new Vector2d(30,10),600)
//      //samples with spinning


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}