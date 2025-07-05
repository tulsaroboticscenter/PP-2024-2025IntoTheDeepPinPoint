package org.firstinspires.ftc.teamcode.opmodes;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;


@Autonomous(name = "Auto - 5 Specimen CRI", group = "Competition", preselectTeleOp = "WorldsBestTeleopFINAL")
public class CRIMiddleSpec extends LinearOpMode{

    public static String TEAM_NAME = "Project Peacock";
    public static int TEAM_NUMBER = 10355;

    public final static HWProfile robot = new HWProfile();
    public LinearOpMode opMode = this;
    public CSAutoParams params = new CSAutoParams();
    public RRMechOps mechOps = new RRMechOps(robot, opMode);
    public Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
    public Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
    public Pose2d specimenScoringPosition2 = new Pose2d(0, 0, 0);
    public Pose2d specimenScoringPosition3 = new Pose2d(0, 0, 0);
    public Pose2d specimenScoringPosition4 = new Pose2d(0, 0, 0);
    public Pose2d specimenScoringPosition5 = new Pose2d(0, 0, 0);
    public Pose2d specimenScoringPrep = new Pose2d(0, 0, 0);
    public Pose2d specimenScoringPush = new Pose2d(0, 0, 0);
    public Pose2d coloredSample1PositionGrab = new Pose2d(0, 0, 0);
    public Pose2d coloredSample2PositionGrab = new Pose2d(0, 0, 0);
    public Pose2d coloredSample3PositionGrab = new Pose2d(0, 0, 0);
    public Pose2d coloredSample4PositionGrab = new Pose2d(0, 0, 0);
    public Pose2d coloredSample5PositionGrab = new Pose2d(0, 0, 0);
    public Pose2d coloredSample6PositionGrab = new Pose2d(0, 0, 0);
    public Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
    public Pose2d specimenGrabPrepCycle = new Pose2d(0, 0, 0);
    public Pose2d dropPose = new Pose2d(0, 0, 0);
    public Pose2d specimenGrabPrep = new Pose2d(0, 0, 0);
    public Pose2d parkPose = new Pose2d(0, 0, 0);
    public double botHeading = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        specimenScoringPosition = new Pose2d(-28.5, 2, Math.toRadians(0));
        specimenScoringPosition2 = new Pose2d(-27.5, 0, Math.toRadians(5));
        specimenScoringPosition3 = new Pose2d(-27, -1, Math.toRadians(5));
        specimenScoringPosition4 = new Pose2d(-26.5, -3, Math.toRadians(5));
        specimenScoringPosition5 = new Pose2d(-26.5, -5, Math.toRadians(5));
        specimenGrabPrep = new Pose2d(-9, 27.5, Math.toRadians(-180)); // specimen grabbing prep
        specimenGrabPrepCycle = new Pose2d(-9, 27.25, Math.toRadians(-180));
        specimenScoringPrep = new Pose2d(-20, 2, Math.toRadians(0));
        //specimenScoringPush = new Pose2d(-28, -8, Math.toRadians(0));
        grabSpecimenPosition = new Pose2d(5, -31.2, Math.toRadians(0));
        coloredSample1PositionGrab = new Pose2d(11.5, -28.5, Math.toRadians(0));
        coloredSample2PositionGrab = new Pose2d(11.75, -33.5, Math.toRadians(0));
        coloredSample3PositionGrab = new Pose2d(33.6, -29.3, Math.toRadians(0));
        coloredSample4PositionGrab = new Pose2d(33.9, -33.6, Math.toRadians(0));
        coloredSample5PositionGrab = new Pose2d(21, -30.0, Math.toRadians(0));
        coloredSample6PositionGrab = new Pose2d(21, -34.5, Math.toRadians(0));
        dropPose = new Pose2d(10, -31.2, Math.toRadians(0)); //prep for grabbing first sample
        parkPose = new Pose2d(-5, 45, Math.toRadians(-180));

        robot.init(hardwareMap, false);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        //Key Pay inputs to selecting Starting Position of robot
//        selectStartingPosition();
        mechOps.scoreClawClosed();
        mechOps.extClawOpen();
        mechOps.extForeBarRetract();


        telemetry.addData("5 Specimen Auto - CRI", "");
        telemetry.addData("Team Name   : ", TEAM_NAME);
        telemetry.addData("Team Number   : ", TEAM_NUMBER);
        telemetry.addLine("PRESS PLAY TO START");
        telemetry.update();

        // intialize the heading file to 0
        mechOps.writeToFile(botHeading, "HeadingFile");

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            scoreSpecimen1(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

//            retreiveColoredSamples(drive);
//            // save heading to local file for teleop if bot gets stopped prematurely
//            if(isStopRequested()){
//                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
//                mechOps.writeToFile(botHeading, "HeadingFile");
//            }
//
//            scoreSpecimen2(drive);
//            // save heading to local file for teleop if bot gets stopped prematurely
//            if(isStopRequested()){
//                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
//                mechOps.writeToFile(botHeading, "HeadingFile");
//            }
//
//            scoreSpecimen3(drive);
//            // save heading to local file for teleop if bot gets stopped prematurely
//            if(isStopRequested()){
//                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
//                mechOps.writeToFile(botHeading, "HeadingFile");
//            }
//
//            scoreSpecimen4(drive);
//            // save heading to local file for teleop if bot gets stopped prematurely
//            if(isStopRequested()){
//                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
//                mechOps.writeToFile(botHeading, "HeadingFile");
//            }
//
//            scoreSpecimen5(drive);
//            // save heading to local file for teleop if bot gets stopped prematurely
//            if(isStopRequested()){
//                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
//                mechOps.writeToFile(botHeading, "HeadingFile");
//            }
//
//            park(drive);
//            // save heading to local file for teleop if bot gets stopped prematurely
//            if(isStopRequested()){
//                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
//                mechOps.writeToFile(botHeading, "HeadingFile");
//            }
        }

        // write the bot heading to a local file for retrieval for field centric drive in TeleOp
        botHeading = Math.toDegrees(drive.pose.heading.toDouble());
        botHeading = 180 - botHeading;
        mechOps.writeToFile(botHeading, "HeadingFile");

        requestOpModeStop();
    }

    //end runOpMode();

    public void scoreSpecimen1(PinpointDrive drive) {
        //Initialize Pose2d as desired


        // Drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(coloredSample1PositionGrab.position,coloredSample1PositionGrab.heading)
                        .build());

        //Release the sample into the basket
        // Lower the arm
        if(opModeIsActive()) {
            mechOps.scoreClawOpen();
            mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
            mechOps.setAutoExtensionPosition();
            mechOps.extForeBarDeploy();
            safeWaitSeconds(.25);
            robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
            robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
            robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
            safeWaitSeconds(.3);

        }
        if(opModeIsActive()) mechOps.scoreForeGrab();
        if(opModeIsActive()) mechOps.scoreClawOpen();
        if(opModeIsActive()) mechOps.autoSampleCRI();
        if(opModeIsActive()) mechOps.scoreClawOpen();


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(dropPose.position,dropPose.heading)
                        .strafeToLinearHeading(coloredSample2PositionGrab.position,coloredSample2PositionGrab.heading)
                        .build());




        if(opModeIsActive()) {
            mechOps.scoreClawOpen();
            mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
            mechOps.setAutoExtensionPosition();
            mechOps.extForeBarDeploy();
            safeWaitSeconds(.25);
            robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
            robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
            robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
            safeWaitSeconds(.3);


        }
//        if(opModeIsActive()) mechOps.scoreForeGrab();
//        safeWaitSeconds(.5);
//        if(opModeIsActive()) mechOps.scoreClawOpen();
//        if(opModeIsActive()) robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
//        safeWaitSeconds(0.2);
//        if(opModeIsActive()) mechOps.autoSampleCRI();
        if(opModeIsActive()) mechOps.scoreForeGrab();
        if(opModeIsActive()) mechOps.scoreClawOpen();
        if(opModeIsActive()) mechOps.autoSampleCRI();
        if(opModeIsActive()) mechOps.scoreClawOpen();


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(dropPose.position,dropPose.heading)
                        .strafeToLinearHeading(coloredSample5PositionGrab.position,coloredSample5PositionGrab.heading)
                        .build());





        if(opModeIsActive()) {
            mechOps.scoreClawOpen();
            mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
            mechOps.setAutoExtensionPosition();
            mechOps.extForeBarDeploy();
            safeWaitSeconds(.25);
            robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
            robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
            robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
            safeWaitSeconds(.3);

        }
        if(opModeIsActive()) mechOps.scoreForeGrab();
        if(opModeIsActive()) mechOps.scoreClawOpen();
        if(opModeIsActive()) mechOps.autoSampleCRI();
        if(opModeIsActive()) mechOps.scoreClawOpen();


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropPose.position,dropPose.heading)
                        .strafeToLinearHeading(coloredSample6PositionGrab.position,coloredSample6PositionGrab.heading)
                        .build());



        if(opModeIsActive()) {
            mechOps.scoreClawOpen();
            mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
            mechOps.setAutoExtensionPosition();
            mechOps.extForeBarDeploy();
            safeWaitSeconds(.25);
            robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
            robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
            robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
            safeWaitSeconds(.3);


        }
        if(opModeIsActive()) mechOps.scoreForeGrab();
        if(opModeIsActive()) mechOps.scoreClawOpen();
        if(opModeIsActive()) mechOps.autoSampleCRI();
        if(opModeIsActive()) mechOps.scoreClawOpen();



        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropPose.position,dropPose.heading)
                        .strafeToLinearHeading(grabSpecimenPosition.position,grabSpecimenPosition.heading)
                        //.strafeToLinearHeading(coloredSample5PositionGrab.position,coloredSample5PositionGrab.heading)
                        .build());

        if (opModeIsActive()) robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_HOLD);
        if (opModeIsActive()) robot.motorLiftFront.setPower(1);
        if (opModeIsActive()) robot.motorLiftBack.setPower(1);
        if (opModeIsActive()) mechOps.specimenPrepPosition();

        // Drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                        .build());

        //Release the sample into the basket
        // Lower the arm
        if (opModeIsActive()) mechOps.specimenScorePosition();
        safeWaitSeconds(.25);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());







//        if(opModeIsActive()) {
//            mechOps.scoreClawOpen();
//            mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
//            mechOps.setAutoExtensionPosition();
//            mechOps.extForeBarDeploy();
//            safeWaitSeconds(.25);
//            robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
//            robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
//            robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
//            safeWaitSeconds(.25);
//
//
//        }
//        if(opModeIsActive()) mechOps.scoreForeGrab();
//        if(opModeIsActive()) mechOps.scoreClawOpen();
//        if(opModeIsActive()) mechOps.autoSampleCRI();
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(dropPose.position,dropPose.heading)
     //                     .strafeToLinearHeading(coloredSample6PositionGrab.position,coloredSample6PositionGrab.heading)
//                        .build());
//
//        if(opModeIsActive()) mechOps.scoreClawOpen();
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//
//                        .build());
//
//        if(opModeIsActive()) {
//            mechOps.scoreClawOpen();
//            mechOps.extensionPosition =  ((int)robot.EXTENSION_OUT_MAX);
//            mechOps.setAutoExtensionPosition();
//            mechOps.extForeBarDeploy();
//            safeWaitSeconds(.25);
//            robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
//            robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
//            robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
//            safeWaitSeconds(.25);
//        }
//        if(opModeIsActive()) mechOps.scoreForeGrab();
//        if(opModeIsActive()) mechOps.scoreClawOpen();
//        if(opModeIsActive()) mechOps.autoSampleCRI();
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(dropPose.position,dropPose.heading)
//                        .build());
//
//        if(opModeIsActive()) mechOps.scoreClawOpen();

    }



    public void retreiveColoredSamples(PinpointDrive drive) {
        // Drive to color sample1 Position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        // Push Color Sample1 into the Observation area
        // Drive to color sample1 Position

        if(opModeIsActive()) mechOps.extForeBarSweep();

        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();

//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(coloredSample1PositionGrab.position, coloredSample1PositionGrab.heading)
//                        .strafeToLinearHeading(coloredSample1PositionDrop.position, coloredSample1PositionDrop.heading)
//                        .build());
//
//        // Raise Arm to high basket scoring position
//        if (opModeIsActive()) mechOps.extForePart();
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(coloredSample2PositionGrab.position, coloredSample2PositionGrab.heading)
//                        .build());
//
//        //safeWaitSeconds(0.1);
//        if (opModeIsActive()) mechOps.extForeBarSweep();
//        //safeWaitSeconds(.1);
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(coloredSample2PositionDrop.position, coloredSample2PositionDrop.heading)
//                        .build());
//
//        if (opModeIsActive()) mechOps.extForePart();
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(coloredSample3PositionGrab.position, coloredSample3PositionGrab.heading)
//                        .build());
//
//
//        // safeWaitSeconds(0.1);
//        if (opModeIsActive()) mechOps.extForeBarSweep();
//        //safeWaitSeconds(.1);
//
//
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(coloredSample3PositionDrop.position, coloredSample3PositionDrop.heading)
//                        .build());
    }

    public void scoreSpecimen2(PinpointDrive drive) {
        if (opModeIsActive()) mechOps.extClawOpen();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        if (opModeIsActive()) mechOps.scoreForeSpecimen();
        if (opModeIsActive()) mechOps.extPitchGrab();
        if (opModeIsActive()) robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenGrabPrep.position, specimenGrabPrep.heading)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());

        if (opModeIsActive()) mechOps.scoreClawClosed();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenPrepPosition();

        // Push Color Sample1 into the Observation area
        // Drive to color sample1 Position


        // Raise Arm to high basket scoring position

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenScoringPrep.position, specimenScoringPrep.heading)
                        .strafeToLinearHeading(specimenScoringPosition2.position, specimenScoringPosition2.heading)
                        .build());

        if (opModeIsActive()) mechOps.specimenScorePosition();
        if (opModeIsActive()) safeWaitSeconds(.25);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        //safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();
//
    }

    public void scoreSpecimen3(PinpointDrive drive) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenGrabPrep.position, specimenGrabPrep.heading)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());


        // Grab the specimen
        if (opModeIsActive()) mechOps.scoreClawClosed();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenPrepPosition();
        if (opModeIsActive()) mechOps.extPitchGrab();


        // Raise Arm to high basket scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        // .strafeToLinearHeading(specimenScoringPrep.position,specimenScoringPrep.heading)
                        .strafeToLinearHeading(specimenScoringPosition3.position, specimenScoringPosition4.heading)
                        .build());


//
        if (opModeIsActive()) mechOps.specimenScorePosition();
        safeWaitSeconds(.25);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        //safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();

    }

    public void scoreSpecimen4(PinpointDrive drive) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenGrabPrep.position, specimenGrabPrep.heading)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());


        // Grab the specimen
        if (opModeIsActive()) mechOps.scoreClawClosed();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenPrepPosition();


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        // .strafeToLinearHeading(specimenScoringPrep.position,specimenScoringPrep.heading)
                        .strafeToLinearHeading(specimenScoringPosition4.position, specimenScoringPosition4.heading)
                        .build());
//

        if (opModeIsActive()) mechOps.specimenScorePosition();
        safeWaitSeconds(.25);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        //safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();

    }

    public void scoreSpecimen5(PinpointDrive drive) {

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenGrabPrep.position, specimenGrabPrep.heading)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());


        // Grab the specimen
        if (opModeIsActive()) mechOps.scoreClawClosed();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenPrepPosition();



        // Raise Arm to high basket scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenScoringPosition5.position, specimenScoringPosition5.heading)
                        .build());
//

        if (opModeIsActive()) mechOps.specimenScorePosition();
        safeWaitSeconds(.35);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        //safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) mechOps.scoreForeHold();




    }

    public void park(PinpointDrive drive){


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());

    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public class SetLiftPosition implements Action {
        int targetPosition;

        public SetLiftPosition(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.motorLiftBack.setPower(1);
            robot.motorLiftBack.setTargetPosition(this.targetPosition);
            robot.motorLiftFront.setPower(1);
            robot.motorLiftFront.setTargetPosition(this.targetPosition);
            robot.motorLiftTop.setPower(1);
            robot.motorLiftTop.setTargetPosition(this.targetPosition);
            return false;
        }
    }

}   // end class
