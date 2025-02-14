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














        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
//                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 65,300 ))
//                        .strafeTo(new Vector2d(-2, 44))
//                        .lineToY(30)
//                        .lineToY(55)
//                     //   .strafeTo(new Vector2d(-30,30))
//                .strafeTo(new Vector2d(-35, 40))
//                .strafeTo(new Vector2d(-35, 10))
//                .strafeTo(new Vector2d(-45, 10))
//
//                .strafeTo(new Vector2d(-45, 60))
//                       // .lineToY(10)
//                        .lineToX(-45)
//                        .strafeTo(new Vector2d(-45, 60))
////                        .strafeTo(new Vector2d(-47, 10))
////                        .strafeTo(new Vector2d(-52, 10))
////                        .strafeTo(new Vector2d(-52, 60))
//                        .lineToY(45)
//                        .turn(3.21)
//                        //open claw
//                        .lineToY(60)
//
//                        .strafeTo(new Vector2d(-2, 44))
//                        .turn(3.15)
////                        //raise lift
////                        .lineToY(35)
////                        //lower lift
////                        .lineToY(50)
////                        .turn(3.15)
//                        .strafeTo(new Vector2d(-42, 60))
////                        //raise lift
////                        .strafeTo(new Vector2d(-2, 44))
////                        .turn(3.15)
////                        .lineToY(35)
////                                        //lower lift
////                        .lineToY(55)
////                        .strafeTo(new Vector2d(-42, 60))
////
//
//
//
//
//
//
//
//
//













//blue sample auto NEW
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, 65,Math.toRadians(2) ))
//
//                .strafeTo(new Vector2d(30,56) )
//                .turn(0.8 )
//                .strafeTo(new Vector2d(35,60))
//                //open claw after toBasket
//                .lineToY(50)
//                .strafeTo(new Vector2d(48,45))
//                .turn(-2.3)
//                .lineToY(33)
//                //close claw
//        // lift the lift up stimutanously with to Basket2
//                .lineToY(50)
//                .turn(2.3)
//                .strafeTo(new Vector2d(55,53))
//// open claw
//                .strafeTo(new Vector2d(57,45))
//                .turn(-2.3)
//                .lineToY(33)
//                //close claw
//                .lineToY(46)
//                .turn(2.3)
//                .strafeTo(new Vector2d(55,53))
//                //open claw
//                .strafeTo(new Vector2d(50,48))
//                //lower lift
//                .strafeTo(new Vector2d(35,5))
//                .turn(2.5)
//                .lineToX(23)

////blue specimen auto NEW
////                myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 65,300 ))
////                        .strafeTo(new Vector2d(-2, 44))
////                        .lineToY(35)
////                        .lineToY(60)
////
//                        .strafeTo(new Vector2d(-35, 30))
//                       // .lineToY(5)
//
//                       // .lineToY(10)
//                        .lineToX(-45)
//                        .strafeTo(new Vector2d(-45, 60))
//                        .strafeTo(new Vector2d(-47, 10))
//                        .strafeTo(new Vector2d(-52, 10))
//                        .strafeTo(new Vector2d(-52, 60))
//                        .lineToY(35)
//                        .turn(3.21)
//                        //open claw
//                        .lineToY(60)
//
//                        .strafeTo(new Vector2d(-2, 44))
//                        .turn(3.15)
//                        //raise lift
//                        .lineToY(35)
//                        //lower lift
//                        .lineToY(50)
//                        .turn(3.15)
//                        .strafeTo(new Vector2d(-42, 60))
//                        //raise lift
//                        .strafeTo(new Vector2d(-2, 44))
//                        .turn(3.15)
//                        .lineToY(35)
//                                        //lower lift
//                        .lineToY(55)
//                        .strafeTo(new Vector2d(-42, 60))











//                 .strafeTo(new Vector2d(-2, 44))
//                .lineToY(35)
//                .lineToY(55)
//
//                        .strafeTo(new Vector2d(-42,35))
//                .lineToY(8)
//                .lineToY(60)
//                .strafeTo(new Vector2d(-54,8))
//                .lineToY(60)
//                .lineToY(35)

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, 65,Math.toRadians(2) ))
//                .strafeTo(new Vector2d(45,56))
//                .turn(0.7)
//                .strafeTo(new Vector2d(52,55))
//                .strafeTo(new Vector2d(38,10))
                //.build();


//        Pose2d initialPose = new Pose2d(-8, 65, 300);
////               .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
////                .build();
//
////

//                 .lineToY(44)
//
//                 .lineToY(40)
//
//                 .lineToY(55)
//
//                 .strafeTo(new Vector2d(-42, 8))
//                 .strafeTo(new Vector2d(-42, 60))
//                 .strafeTo(new Vector2d(-42, 8))
//                 .strafeTo(new Vector2d(-52, 8))
//                 .strafeTo(new Vector2d(-52, 60))
//                 .strafeTo(new Vector2d(-52, 35))
//
//                 .turn(3.2)
//                 .lineToY(58)
//                 .strafeTo(new Vector2d(-8, 44))
//                 .turn(-3.2)
//                 .lineToY(35)
//
//

//          // raise lift stimiltanulous
//                .lineToY(35)
//
//           //lower lift before this
//                .lineToY(60)
//           //open claw after
//
////lowwer lift
//                .strafeTo(new Vector2d(-42,8))
//                .strafeTo(new Vector2d(-42,60)) // Adjusted for consistency
//                .strafeTo(new Vector2d(-42,8))
//
//
//                .strafeTo(new Vector2d(-52,8))
//                .strafeTo(new Vector2d(-52,60))
//                .strafeTo(new Vector2d(-52,35))
//
//                .turn(3.2)
//
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



















//                 myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 65,300 ))
//
//                // raise lift stimiltanulous
//                .lineToY(35)
//
//                //lower lift before this
//                .lineToY(60)
//                //open claw after
////
//
//                .strafeTo(new Vector2d(-42,8))
//                .strafeTo(new Vector2d(-42,60)) // Adjusted for consistency
//                .strafeTo(new Vector2d(-42,8))
//
//                .strafeTo(new Vector2d(-52,8))
//                .strafeTo(new Vector2d(-52,60))
//                .strafeTo(new Vector2d(-52,35))
////lowwer lift
//                .turn(3.2)
//                .strafeTo(new Vector2d(-52,58))
////close claw then raise liftt
//                .strafeTo(new Vector2d(-8,38))
//                .turn(-3.2)
//                .strafeTo(new Vector2d(-8,35))
//                         //lower lift
//                .strafeTo(new Vector2d(-8,42))
//                         //open claw
//
//
//                .strafeTo(new Vector2d(-54,54))
//                .turn(3.2)
//                         //close claw
//                .strafeTo(new Vector2d(-54,56))
//                         //raise lift
//                .turn(-3.2)
//
//                .strafeTo(new Vector2d(-8,38))
//                         //lower lift
//                .strafeTo(new Vector2d(-8,54))
//                         //open claw
//                         //lower lift
//
//
//                .strafeTo(new Vector2d(-60.5,8))
//                .strafeTo(new Vector2d(-60.5,58))






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
//
//                .strafeTo(new Vector2d(-52,-56))
//                .turn(2.3)
//                .strafeTo(new Vector2d(-54,-56))
//
//
//                .turn(-2.3)
//                .strafeTo(new Vector2d(-56,-28))
//
//                .strafeTo(new Vector2d(-52,-56))
//                .turn(2.3)
//                .strafeTo(new Vector2d(-54,-56))
//                .turn(-1)
//                .strafeTo(new Vector2d(-56,-28))
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

              //  // raise lift stimiltanulous
//                .lineToY(35)

             //   //lower lift before this
//                .lineToY(60)
             //   //open claw after
//
////lowwer lift
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