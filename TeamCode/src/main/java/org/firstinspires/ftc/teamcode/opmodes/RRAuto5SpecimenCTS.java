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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;


@Autonomous(name = "Auto - 5 Specimen 5+0", group = "Competition", preselectTeleOp = "GoBildaRi3D2425")
public class RRAuto5SpecimenCTS extends LinearOpMode{

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
    public Pose2d coloredSample1PositionDrop = new Pose2d(0, 0, 0);
    public Pose2d coloredSample2PositionDrop = new Pose2d(0, 0, 0);
    public Pose2d coloredSample3PositionDrop = new Pose2d(0, 0, 0);
    public Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
    public Pose2d specimenGrabPrepCycle = new Pose2d(0, 0, 0);
    public Pose2d midwayPose1 = new Pose2d(0, 0, 0);
    public Pose2d specimenGrabPrep = new Pose2d(0, 0, 0);
    public Pose2d parkPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        specimenScoringPosition = new Pose2d(-28.5, 3, Math.toRadians(0));
        specimenScoringPosition2 = new Pose2d(-28, 2, Math.toRadians(5));
        specimenScoringPosition3 = new Pose2d(-28, -1, Math.toRadians(5));
        specimenScoringPosition4 = new Pose2d(-27.5, -3, Math.toRadians(5));
        specimenScoringPosition5 = new Pose2d(-28, -5, Math.toRadians(5));
        specimenGrabPrep = new Pose2d(-9, 27.69, Math.toRadians(-180)); // specimen grabbing prep
        specimenGrabPrepCycle = new Pose2d(-9, 27.25, Math.toRadians(-180));
        specimenScoringPrep = new Pose2d(-20, 2, Math.toRadians(0));
        //specimenScoringPush = new Pose2d(-28, -8, Math.toRadians(0));
        grabSpecimenPosition = new Pose2d(-2.75, 27, Math.toRadians(-180));
        coloredSample1PositionGrab = new Pose2d(-30, 34, Math.toRadians(120));
        coloredSample1PositionDrop = new Pose2d(-15, 28, Math.toRadians(40));
        coloredSample2PositionGrab = new Pose2d(-27, 25, Math.toRadians(118));
        coloredSample2PositionDrop = new Pose2d(-17, 24, Math.toRadians(50));
        coloredSample3PositionGrab = new Pose2d(-27, 16.5, Math.toRadians(110));
        coloredSample3PositionDrop = new Pose2d(-11, 19, Math.toRadians(50));
        midwayPose1 = new Pose2d(-19, 30, Math.toRadians(103)); //prep for grabbing first sample
        parkPose = new Pose2d(-20, -5, Math.toRadians(-160));

        robot.init(hardwareMap, false);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        //Key Pay inputs to selecting Starting Position of robot
//        selectStartingPosition();
        mechOps.scoreClawClosed();
        mechOps.extForeBarRetract();
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_TRANSFER);
        mechOps.tightenStrings();

        telemetry.addData("5 Specimen Auto - CTS Ready", "");
        telemetry.addData("Team Name   : ", TEAM_NAME);
        telemetry.addData("Team Number   : ", TEAM_NUMBER);
        telemetry.addLine("PRESS PLAY TO START");
        telemetry.update();

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            scoreSpecimen1(drive);
            retreiveColoredSamples(drive);
            scoreSpecimen2(drive);
            scoreSpecimen3(drive);
            scoreSpecimen4(drive);
            scoreSpecimen5(drive);
            park(drive);
        }
        requestOpModeStop();
    }

    //end runOpMode();

    public void scoreSpecimen1(PinpointDrive drive) {
        //Initialize Pose2d as desired

        if(opModeIsActive()) robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_HOLD);
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
        safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();
        if (opModeIsActive()) mechOps.extClawClose();

    }

    public void retreiveColoredSamples(PinpointDrive drive) {
        // Drive to color sample1 Position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)

                        .build());

        // Push Color Sample1 into the Observation area
        // Drive to color sample1 Position

//        safeWaitSeconds(0.1);
//        if (opModeIsActive()) mechOps.extClawClose();
        safeWaitSeconds(0.1);
        if(opModeIsActive()) mechOps.extForeBarSweep();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(coloredSample1PositionGrab.position, coloredSample1PositionGrab.heading)
                        .strafeToLinearHeading(coloredSample1PositionDrop.position, coloredSample1PositionDrop.heading)
                        .build());

        // Raise Arm to high basket scoring position
        if (opModeIsActive()) mechOps.extForePart();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(coloredSample2PositionGrab.position, coloredSample2PositionGrab.heading)
                        .build());

        //safeWaitSeconds(0.1);
        if (opModeIsActive()) mechOps.extForeBarSweep();
        safeWaitSeconds(.1);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(coloredSample2PositionDrop.position, coloredSample2PositionDrop.heading)
                        .build());

        if (opModeIsActive()) mechOps.extForePart();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(coloredSample3PositionGrab.position, coloredSample3PositionGrab.heading)
                        .build());


        // safeWaitSeconds(0.1);
        if (opModeIsActive()) mechOps.extForeBarSweep();
        safeWaitSeconds(.1);


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(coloredSample3PositionDrop.position, coloredSample3PositionDrop.heading)
                        .build());
    }

    public void scoreSpecimen2(PinpointDrive drive) {
        if (opModeIsActive()) robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
        if (opModeIsActive()) mechOps.extClawOpen();
        if (opModeIsActive()) robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        if (opModeIsActive()) mechOps.scoreForeSpecimen();
        if (opModeIsActive()) mechOps.extPitchGrab();

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
        safeWaitSeconds(.25);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        //safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
        if (opModeIsActive())
            robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
        if (opModeIsActive()) robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
        if (opModeIsActive()) mechOps.extPitchGrab();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        if (opModeIsActive()) mechOps.setExtensionPosition();
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

}   // end class
