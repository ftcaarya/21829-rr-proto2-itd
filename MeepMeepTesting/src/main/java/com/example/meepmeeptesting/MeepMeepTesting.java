package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.8, 15.25)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0,-36))
                .strafeToLinearHeading(new Vector2d(38,-40),Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(47,-40),Math.toRadians(315))
                .turnTo(45)
                .strafeToLinearHeading(new Vector2d(47,-47), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(0,-36), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(47,-47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,-36), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(47,-47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,-36), Math.toRadians(90))

                .strafeToLinearHeading(new Vector2d(47,-47), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,-36), Math.toRadians(90))




                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}