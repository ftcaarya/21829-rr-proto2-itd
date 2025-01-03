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
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        // red left side
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, -48, Math.toRadians(90)))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(-48, -40 , Math.PI/2), Math.PI/2)
                .strafeToLinearHeading(new Vector2d(-57, -57), Math.toRadians(45))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-60, -41, Math.toRadians(90)), Math.toRadians(540))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-52, -40, Math.toRadians(135)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45))
                .build());

        // testing
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -63, Math.toRadians(270)))
//                .strafeToLinearHeading(new Vector2d(0,-36.5),Math.toRadians(270))
//                // next thing
//                .strafeTo(new Vector2d(20,-36.5))
//                .splineToSplineHeading(new Pose2d(46,-13,Math.toRadians(270)),Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(46,-53),Math.toRadians(270))
//                .waitSeconds(0.5)
//                .strafeTo(new Vector2d(44,-13))
//                .splineToLinearHeading(new Pose2d(55,-11.5,Math.toRadians(270)),Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(55,-53),Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(55,-33),Math.toRadians(0))
//                // next thing
//                .strafeToLinearHeading(new Vector2d(36,-56.5),Math.toRadians(0))
//                //next thing
//                .strafeToLinearHeading(new Vector2d(0,-36.5),Math.toRadians(270))
//                // next thing
//                .strafeToLinearHeading(new Vector2d(36,-56.5),Math.toRadians(0))
//                // next thing
//                .strafeToLinearHeading(new Vector2d(0,-36.5),Math.toRadians(270))
//                // next thiing
//                .strafeToLinearHeading(new Vector2d(36,-56.5),Math.toRadians(0))
//                // wef
//                .strafeToLinearHeading(new Vector2d(0,-36.5),Math.toRadians(270))
//                // samplep get
//                .strafeToLinearHeading(new Vector2d(36,-56.5),Math.toRadians(0))
//                .build());

        // red right side
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -63, Math.toRadians(270)))
//                .strafeTo(new Vector2d(0, -34))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(48, -38, Math.toRadians(90)), Math.PI / 4)
//                .strafeToLinearHeading(new Vector2d(48, -55), Math.toRadians(90))
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(40, -10, Math.toRadians(0)), Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(58, -10), Math.toRadians(270))
//                .strafeToConstantHeading(new Vector2d(58, -55))
//                .strafeToConstantHeading(new Vector2d(58, -38))
//                .strafeToConstantHeading(new Vector2d(58, -55))
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(50, -30, Math.toRadians(0)), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(63, -10, Math.toRadians(-90)), Math.PI/6)
//                .strafeToConstantHeading(new Vector2d(62, -55))
//                .strafeToLinearHeading(new Vector2d(25, -58), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(-90))
//                .build());
//

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}