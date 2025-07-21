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

import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

//@Disabled
@Autonomous(name = "CRI BLUE Left 2 High + 1 Low Spec", group = "Competition", preselectTeleOp = "WorldsBestTeleopFINAL")
public class CRIBLUE3HLLeft extends LinearOpMode{

    public static String TEAM_NAME = "Project Peacock";
    public static int TEAM_NUMBER = 10355;
    public double yAxisOffset = 8;
    public double xAxisOffset = 48;
    public double buttonPressDelay = 0.2;       // button press delay for menu

    public final static HWProfile robot = new HWProfile();
    public LinearOpMode opMode = this;
    public CSAutoParams params = new CSAutoParams();
    public RRMechOps mechOps = new RRMechOps(robot, opMode);

    //Initialize Pose2d as desired
    public Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
    public Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
    public Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
    public Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
    public Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
    public Pose2d yellowSample5Position = new Pose2d(0, 0, 0);
    public Pose2d yellowSample6Position = new Pose2d(0, 0, 0);
    public Pose2d yellowSample7Position = new Pose2d(0, 0, 0);
    public Pose2d specimenPrepPosition = new Pose2d(0, 0, 0);
    public Pose2d specimenScorePosition = new Pose2d(0,0,0);
    public Pose2d specimenScore2Position = new Pose2d(0,0,0);
    public Pose2d specimenScore3Position = new Pose2d(0,0,0);
    public Pose2d specimenScoreDrive = new Pose2d(0,0,0);
    public Pose2d humanPlayerSpecGrabPrep = new Pose2d(0,0,0);
    public Pose2d humanPlayerSpecGrab = new Pose2d(0,0,0);
    public Pose2d midwayPose1 = new Pose2d(0, 0, 0);
    public Pose2d midwayPose2 = new Pose2d(0, 0, 0);
    public Pose2d midwayPose3 = new Pose2d(0, 0, 0);
    public Pose2d midwayPose4 = new Pose2d(0,0,0);
    public Pose2d moveAwayFromHuman = new Pose2d(0,0,0);
    public Pose2d submersiblePose = new Pose2d(0, 0, 0);
    public Pose2d parkPrepPose = new Pose2d(0, 0, 0);
    public Pose2d parkPose = new Pose2d(0, 0, 0);
    double botHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        sampleScoringPosition = new Pose2d(9.5, -25, Math.toRadians(45));
        yellowSample1Position = new Pose2d(10.75, -15.75, Math.toRadians(0));
        yellowSample2Position = new Pose2d(12, -25, Math.toRadians(5));
        yellowSample3Position = new Pose2d(37.5, -7.5, Math.toRadians(-90));
        specimenPrepPosition = new Pose2d(55,10, Math.toRadians(90));
        specimenScorePosition = new Pose2d(60,3.25, Math.toRadians(90));
        specimenScore2Position = new Pose2d(57,3, Math.toRadians(90));
        specimenScore3Position = new Pose2d(58,3, Math.toRadians(90));
        specimenScoreDrive = new Pose2d(58,5, Math.toRadians(90));
        humanPlayerSpecGrabPrep = new Pose2d(8.7,-55, Math.toRadians(0));
        humanPlayerSpecGrab = new Pose2d(2, -55, Math.toRadians(0));
        midwayPose1 = new Pose2d(15,30, Math.toRadians(0));
        midwayPose2 = new Pose2d(20,10, Math.toRadians(0));
        midwayPose3 = new Pose2d(36,-5, Math.toRadians(-90));
        midwayPose4 = new Pose2d(55,10, Math.toRadians(0));
        moveAwayFromHuman = new Pose2d(5,10, Math.toRadians(0));
        parkPrepPose = new Pose2d(48, -20, Math.toRadians(90));
        parkPose = new Pose2d(53, -12, Math.toRadians(90));

        //Initialize hardware
        robot.init(hardwareMap, false);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        //Key Pay inputs to selecting Starting Position of robot



        mechOps.scoreClawClosed();
        mechOps.extForeBarRetract();
        mechOps.scoreForeHold();
        mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        mechOps.setExtensionPosition();


        //selectYellowSamples();

        // Wait for the DS start button to be touched.

        //telemetry.addLine("-------------------------------------------------");
        telemetry.addData("2 Specimen, 2 Sample", "");
        telemetry.addData("Team Name   : ", TEAM_NAME);
        telemetry.addData("Team Number   : ", TEAM_NUMBER);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // intialize the heading file to 0
        mechOps.writeToFile(botHeading, "HeadingFile");

        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            scorePreLoadSpec(drive);
            // save heading to local file if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            specGrab2(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

        }

        // write the bot heading to a local file for retrieval for field centric drive in TeleOp
        botHeading = Math.toDegrees(drive.pose.heading.toDouble());
        mechOps.writeToFile(botHeading, "HeadingFile");

        requestOpModeStop();
    } //end runOpMode();

    // method to score the preloaded sample into the high basket
    public void scorePreLoadSpec(PinpointDrive drive) {

        /**
         * For Sample Scoring into high basket
         **/
        telemetry.addLine("program has started");
        telemetry.update();


        if (opModeIsActive()) robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_HOLD);
        if (opModeIsActive()) robot.motorLiftFront.setPower(1);
        if (opModeIsActive()) robot.motorLiftBack.setPower(1);
        if (opModeIsActive()) mechOps.specimenPrepPositionCRI();




        // Drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        .strafeToLinearHeading(specimenPrepPosition.position, specimenPrepPosition.heading)
                        .strafeToLinearHeading(specimenScorePosition.position, specimenScorePosition.heading)
                        .build());

        //Release the sample into the basket
        // Lower the arm
        if (opModeIsActive()) mechOps.scoreForeSpecimen();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenScorePosition();
        safeWaitSeconds(.35);
        if (opModeIsActive()) mechOps.scoreClawOpen();
        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();
        if (opModeIsActive()) mechOps.extClawOpen();
        if (opModeIsActive()) mechOps.extPitchGrab();
        if (opModeIsActive()) mechOps.extForeBarPart();

    }

    public void specGrab2(PinpointDrive drive) {
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenPrepPosition.position, specimenPrepPosition.heading)
                        .stopAndAdd(new SetLiftPosition(params.LIFT_RESET))
                        .build());

        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2.position,midwayPose2.heading)
                        .strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
                        .strafeToLinearHeading(humanPlayerSpecGrab.position,humanPlayerSpecGrab.heading)
                        .build());

        if (opModeIsActive()) mechOps.scoreClawClosed();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenPrepPosition();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
                        .strafeToLinearHeading(midwayPose2.position,midwayPose2.heading)
                        //.strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        .strafeToLinearHeading(specimenPrepPosition.position,specimenPrepPosition.heading)
                        .strafeToLinearHeading(specimenScore2Position.position, specimenScore2Position.heading)
                        .build());

//        if (opModeIsActive()) mechOps.scoreForeSpecimen();
//        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenScorePosition();
        safeWaitSeconds(.35);

        if (opModeIsActive()) mechOps.scoreClawOpen();
        if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) mechOps.scoreForeSpecimen();



        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenPrepPosition.position,specimenPrepPosition.heading)
                        //.strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
                        //.strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        .strafeToLinearHeading(midwayPose2.position,midwayPose2.heading)
                        .strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
                        .strafeToLinearHeading(humanPlayerSpecGrab.position,humanPlayerSpecGrab.heading)
                        //.strafeToLinearHeading(specimenScore2Position.position, specimenScore2Position.heading)
                        .build());
        if (opModeIsActive()) mechOps.scoreClawClosed();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenPrepPositionCRILow();



        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
                        .strafeToLinearHeading(midwayPose2.position,midwayPose2.heading)
                        //.strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        .strafeToLinearHeading(specimenPrepPosition.position,specimenPrepPosition.heading)
                        .strafeToLinearHeading(specimenScore3Position.position, specimenScore3Position.heading)
                        .build());

        if (opModeIsActive()) mechOps.scoreForeSpecimen();
        safeWaitSeconds(.1);
        if (opModeIsActive()) mechOps.specimenScorePositionLow();
        safeWaitSeconds(.75);

//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        //.strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
//                        //.strafeToLinearHeading(midwayPose2.position,midwayPose2.heading)
//                        .strafeToLinearHeading(specimenScoreDrive.position,specimenScoreDrive.heading)
//                        //.strafeToLinearHeading(specimenScore2Position.position, specimenScore2Position.heading)
//                        .build());

        if (opModeIsActive()) mechOps.scoreClawOpen();
        if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) mechOps.scoreForeSpecimen();



        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
                        .strafeToLinearHeading(moveAwayFromHuman.position,moveAwayFromHuman.heading)
                        .strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        .strafeToLinearHeading(midwayPose2.position,midwayPose2.heading)
                        .strafeToLinearHeading(humanPlayerSpecGrabPrep.position,humanPlayerSpecGrabPrep.heading)
                        .strafeToLinearHeading(humanPlayerSpecGrab.position,humanPlayerSpecGrab.heading)
                        //.strafeToLinearHeading(specimenScore2Position.position, specimenScore2Position.heading)
                        .build());


    }

    // method to part the robot
    public void park(PinpointDrive drive) {

        // Park

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        .strafeToLinearHeading(parkPrepPose.position,parkPrepPose.heading)
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

    public class SetServoPositionScoreSample implements Action {


        public SetServoPositionScoreSample() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE);
            robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE);
            return false;
        }
    }
    public class SetServoPositionScoreReset implements Action {


        public SetServoPositionScoreReset() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_GRAB);
            robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_GRAB);
            return false;
        }
    }

    public class SetServoPositionScorePark implements Action {


        public SetServoPositionScorePark() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_HOLD_AUTON);
            robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_HOLD_AUTON);
            return false;
        }
    }

    enum State {
        SAMPLE_5, SAMPLE_6, SAMPLE_7, EXIT
    }
}   // end class
