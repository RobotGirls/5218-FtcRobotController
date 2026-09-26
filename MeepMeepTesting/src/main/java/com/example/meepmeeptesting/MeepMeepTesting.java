package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        BufferedImage fieldImage = null;
        try (InputStream in = MeepMeepTesting.class.getResourceAsStream("/biobuzz-field.png")) {
            if (in == null) {
                System.out.println("Field image not found on classpath");
            } else {
                fieldImage = ImageIO.read(in);
            }
        } catch (IOException e) {
            System.out.println("Could not load field image: " + e.getMessage());
        }
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //Long Blue side code without strafing (tank drive)
        myBot.runAction(
                myBot.getDrive().actionBuilder(
                                new Pose2d(-60, 10, Math.toRadians(0))
                        )

                        // (-60, 10) -> (-39, 10)
                        .lineToX(-39)
                        .waitSeconds(1)

                        // (-39, 10) -> (-30, 10)
                        .lineToX(-30)
                        .waitSeconds(6)

                        // (-30, 10) -> (55, 17)
                        // Turn toward the point
                        .turn(Math.toRadians(4.7))
                        .lineToX(55)
                        .turn(Math.toRadians(-4.7))

                        // (55, 17) -> (40, 15)
                        .turn(Math.toRadians(-172.4))
                        .lineToX(40)
                        .turn(Math.toRadians(-7.6))
                        .waitSeconds(1)

                        //collect flower pollen
                        .turn(Math.toRadians(190))
                        .lineToX(60)
                        .waitSeconds(2)

                        //shoot the pollen collected
                        .turn(Math.toRadians(-180))
                        .waitSeconds(1.5)

//                      //collect the dropped nectar(3) and pollen(1)
//                        .lineToX(30)
//                        .waitSeconds(6)

                         //shoot pollen into upturned box
//                        .turn(Math.toRadians(-10))
//                        .lineToX(-52)
//                        .turn(Math.toRadians(180))
//                        .waitSeconds(2)

                        //park
                        .turn(Math.toRadians(-90))
                        .lineToY(56)
//                        .turn(Math.toRadians(-32))
                        .build()
        );




                        // (-39, 10) -> (-54, 50)



        //Long Blue side code with strafing (meccanum drive)
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 10, 0))
//                .strafeToLinearHeading(new Vector2d(-39,10),Math.toRadians(0))
//                                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-30,10),Math.toRadians(0))
//                .waitSeconds(6)
//                .strafeToLinearHeading(new Vector2d(55,17),Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(40,15),Math.toRadians(180))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(55,17),Math.toRadians(45))
//                .strafeToLinearHeading(new Vector2d(60,17),Math.toRadians(45))
//                .waitSeconds(2)
//                .strafeToLinearHeading(new Vector2d(-39,10),Math.toRadians(0))
//                .waitSeconds(1.5)
//                .strafeToLinearHeading(new Vector2d(-54,50),Math.toRadians(125))
//                .strafeToLinearHeading(new Vector2d(-60,60),Math.toRadians(180))
//                .strafeToLinearHeading(new Vector2d(35,50),Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(35,60),Math.toRadians(0))
//        Julias Parking code(blue side)
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-5, 56, 0))
//                .strafeToLinearHeading(new Vector2d(35,64),Math.toRadians(0))





        if (fieldImage != null) {
            meepMeep.setBackground(fieldImage);
        } else {
            meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK); // fallback
        }

        meepMeep.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}