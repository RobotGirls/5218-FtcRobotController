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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-5, 56, 0))
                .strafeToLinearHeading(new Vector2d(35,64),Math.toRadians(0))


                //.splineTo(new Vector2d())




//                .lineToY(-8)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                .build());

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