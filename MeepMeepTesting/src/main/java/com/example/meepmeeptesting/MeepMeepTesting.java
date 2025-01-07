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
                 myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 65,300 ))

                // raise lift stimiltanulous
                .lineToY(35)

                //lower lift before this
                .lineToY(60)
                //open claw after
//

                .strafeTo(new Vector2d(-42,8))
                .strafeTo(new Vector2d(-42,60)) // Adjusted for consistency
                .strafeTo(new Vector2d(-42,8))

                .strafeTo(new Vector2d(-52,8))
                .strafeTo(new Vector2d(-52,60))
                .strafeTo(new Vector2d(-52,35))
//lowwer lift
                .turn(3.2)
                .strafeTo(new Vector2d(-52,58))
//close claw then raise liftt
                .strafeTo(new Vector2d(-8,38))
                .turn(-3.2)
                .strafeTo(new Vector2d(-8,35))
                         //lower lift
                .strafeTo(new Vector2d(-8,42))
                         //open claw


                .strafeTo(new Vector2d(-54,54))
                .turn(3.2)
                         //close claw
                .strafeTo(new Vector2d(-54,56))
                         //raise lift
                .turn(-3.2)

                .strafeTo(new Vector2d(-8,38))
                         //lower lift
                .strafeTo(new Vector2d(-8,54))
                         //open claw
                         //lower lift


                .strafeTo(new Vector2d(-60.5,8))
                .strafeTo(new Vector2d(-60.5,58))






//sample auto
//                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(360), 15)
//                .build();
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, -65, Math.toRadians(180) ))
//                .strafeTo(new Vector2d(-52,-56))
//
//                .turn(.7)
//                .strafeTo(new Vector2d(-54,-56))
//
//                //add a forward?
//                .turn(-2.3)
//                .strafeTo(new Vector2d(-45,-28))
//                .strafeTo(new Vector2d(-52,-56))
//                .turn(2.3)
//                .strafeTo(new Vector2d(-54,-56))
//                .turn(-2.3)
//                .strafeTo(new Vector2d(-56,-28))
//                .strafeTo(new Vector2d(-52,-56))
//                .turn(2.3)
//                .strafeTo(new Vector2d(-54,-56))
//












//




// red specimens auto paths
//                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -65,-300 ))
//
//                .lineToY(-35)
//                .lineToY(-60)
//
//
//                .strafeTo(new Vector2d(42,-8))
//                .strafeTo(new Vector2d(42,-60)) // Adjusted for consistency
//                .strafeTo(new Vector2d(42,-8))
//
//
//                .strafeTo(new Vector2d(52,-8))
//                .strafeTo(new Vector2d(52,-60))
//                .strafeTo(new Vector2d(52,-35))
//
//                .turn(3.2)
//                .strafeTo(new Vector2d(52,-58))
//
//
//                .strafeTo(new Vector2d(8,-38))
//                .turn(-3.2)
//                .strafeTo(new Vector2d(8,-35))
//                .strafeTo(new Vector2d(8,-42))
//
//
//                .strafeTo(new Vector2d(54,-54))
//                .turn(3.2)
//                .strafeTo(new Vector2d(54,-56))
//                .turn(-3.2)
//
//                .strafeTo(new Vector2d(8,-38))
//                .strafeTo(new Vector2d(8,-54))
//
//
//                .strafeTo(new Vector2d(60.5,-8))
//                .strafeTo(new Vector2d(60.5,-58))
//
//
//






// blue specimen auto
        // myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 65,300 ))

                // raise lift stimiltanulous
//                .lineToY(35)

                //lower lift before this
//                .lineToY(60)
                //open claw after
//
//lowwer lift
//                .strafeTo(new Vector2d(-42,8))
//                .strafeTo(new Vector2d(-42,60)) // Adjusted for consistency
//                .strafeTo(new Vector2d(-42,8))

//
//                .strafeTo(new Vector2d(-52,8))
//                .strafeTo(new Vector2d(-52,60))
//                .strafeTo(new Vector2d(-52,35))
//
//                .turn(3.2)
//                .strafeTo(new Vector2d(-52,58))
//
//
//                .strafeTo(new Vector2d(-8,38))
//                .turn(-3.2)
//                .strafeTo(new Vector2d(-8,35))
//                .strafeTo(new Vector2d(-8,42))
//
//
//                .strafeTo(new Vector2d(-54,54))
//                .turn(3.2)
//                .strafeTo(new Vector2d(-54,56))
//                .turn(-3.2)
//
//                .strafeTo(new Vector2d(-8,38))
//                .strafeTo(new Vector2d(-8,54))
//
//
//                .strafeTo(new Vector2d(-60.5,8))
//                .strafeTo(new Vector2d(-60.5,58))
















                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}