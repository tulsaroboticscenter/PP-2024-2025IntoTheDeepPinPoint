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
@Autonomous(name = "CRI BLUE Right 0 Spec + 5 Sample", group = "Competition", preselectTeleOp = "WorldsBestTeleopFINAL")
public class CRI05RightAuton extends LinearOpMode{

    public static String TEAM_NAME = "Project Peacock";
    public static int TEAM_NUMBER = 10355;
    public double yAxisOffset = 40;
    public double xAxisOffset = 3;
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
    public Pose2d humanPlayerSpecGrabPrep = new Pose2d(0,0,0);
    public Pose2d humanPlayerSpecGrab = new Pose2d(0,0,0);
    public Pose2d midwayPose1 = new Pose2d(0, 0, 0);
    public Pose2d midwayPose2 = new Pose2d(0, 0, 0);
    public Pose2d midwayPose3 = new Pose2d(0, 0, 0);
    public Pose2d midwayPose4 = new Pose2d(0,0,0);
    public Pose2d submersiblePose = new Pose2d(0, 0, 0);
    public Pose2d parkPrepPose = new Pose2d(0, 0, 0);
    public Pose2d parkPose = new Pose2d(0, 0, 0);
    double botHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        sampleScoringPosition = new Pose2d(9.5, -25, Math.toRadians(45));
        yellowSample1Position = new Pose2d(11.25, -16, Math.toRadians(2.5));
        yellowSample2Position = new Pose2d(12.25, -25, Math.toRadians(5));
        yellowSample3Position = new Pose2d(37.5, -7.5, Math.toRadians(-90));
        specimenPrepPosition = new Pose2d(55,-6, Math.toRadians(-90));
        specimenScorePosition = new Pose2d(55,-3.5, Math.toRadians(-90));
        specimenScore2Position = new Pose2d(55,-3.5, Math.toRadians(-85));
        humanPlayerSpecGrabPrep = new Pose2d(15,57.4, Math.toRadians(0));
        humanPlayerSpecGrab = new Pose2d(2.7, 57.4, Math.toRadians(0));
        midwayPose1 = new Pose2d(15,-23, Math.toRadians(45));
        midwayPose2 = new Pose2d(9,0, Math.toRadians(0));
        midwayPose3 = new Pose2d(36,-5, Math.toRadians(-90));
        midwayPose4 = new Pose2d(20,19, Math.toRadians(0));
        parkPrepPose = new Pose2d(10, 22, Math.toRadians(90));
        parkPose = new Pose2d(5, 60, Math.toRadians(0));

        //Initialize hardware
        robot.init(hardwareMap, false);
        PinpointDrive drive = new PinpointDrive(hardwareMap, initPose);

        //Key Pay inputs to selecting Starting Position of robot



        mechOps.scoreClawClosed();
        mechOps.extForeBarRetract();


        //selectYellowSamples();



        selectYellowSamples();

        // Wait for the DS start button to be touched.

        //telemetry.addLine("-------------------------------------------------");
        telemetry.addData("0 Specimen, 5 Sample", "");
        telemetry.addData("Team Name   : ", TEAM_NAME);
        telemetry.addData("Team Number   : ", TEAM_NUMBER);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // intialize the heading file to 0
        mechOps.writeToFile(botHeading, "HeadingFile");

        waitForStart();

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            scorePreLoad(drive);
            // save heading to local file if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            scoreSample1(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            scoreSample2(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            scoreSample3(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            scoreSample5(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            //scoreSample6(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            //scoreSample7(drive);
            // save heading to local file for teleop if bot gets stopped prematurely
            if(isStopRequested()){
                botHeading = Math.toDegrees(drive.pose.heading.toDouble());
                mechOps.writeToFile(botHeading, "HeadingFile");
            }

            park(drive);
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
    public void scorePreLoad(PinpointDrive drive) {

        /**
         * For Sample Scoring into high basket
         **/
        telemetry.addLine("program has started");
        telemetry.update();

        if (opModeIsActive()) mechOps.extClawOpen();
        if (opModeIsActive()) robot.motorLiftBack.setPower(1);
        if (opModeIsActive()) robot.motorLiftFront.setPower(1);
        if (opModeIsActive()) robot.motorLiftTop.setPower(1);
        //if (opModeIsActive()) mechOps.raiseLiftHighBasketPrep7();

        //safeWaitSeconds(.95);

        // Drive to scoring prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .stopAndAdd(new SetLiftPosition(params.LIFT_SCORE))
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .waitSeconds(.2)
                        .stopAndAdd(new SetServoPositionScoreSample())
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        // score the sample into the high basket


        // Drive to scoring position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .build());

        // Release the preloaded sample into the basket
        if (opModeIsActive()) mechOps.scoreClawOpen();

    }

    // method to retrieve the first sample from the field and score into the high basket
    public void scoreSample1(PinpointDrive drive) {
        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetServoPositionScoreReset())
                        .stopAndAdd(new SetLiftPosition(params.LIFT_RESET))
                        .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
                        .build());


        // lower the arm and prepare to grab sample from the field
        //if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) mechOps.scoreForeGrab();
        if (opModeIsActive()) mechOps.scoreClawOpen();
//        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
//        if (opModeIsActive()) mechOps.setExtensionPosition();
        if (opModeIsActive()) robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        if (opModeIsActive()) mechOps.extForeBarDeploy();
        if (opModeIsActive()) mechOps.scoreForeGrab();

        // Drive to pick up Sample1 Position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
//                        .build());

        // Pick up Sample1 from the field and prepare to score the sample
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        safeWaitSeconds(0.5);
        if (opModeIsActive()) robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
        safeWaitSeconds(0.2);
        if (opModeIsActive()) mechOps.auto7SampleScorePrep();
        safeWaitSeconds(0.25);

        // Drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .stopAndAdd(new SetLiftPosition(params.LIFT_SCORE))
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetServoPositionScoreSample())
                        .waitSeconds(.2)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        // Release sample1 into the basket
        if (opModeIsActive()) mechOps.scoreClawOpen();

    }

    // method to retrieve the second sample from the field and score into the high basket
    public void scoreSample2(PinpointDrive drive) {

        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .stopAndAdd(new SetServoPositionScoreReset())
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetLiftPosition(params.LIFT_RESET))
                        .strafeToLinearHeading(yellowSample2Position.position, yellowSample2Position.heading)
                        .build());

        // lower the lift and prepare to grab the sample from the field
        //if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) mechOps.extClawRotateZero();
        if (opModeIsActive()) mechOps.autoExtension();
//        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
//        if (opModeIsActive()) mechOps.setExtensionPosition();
        if (opModeIsActive()) robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        if (opModeIsActive()) robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);

        //Drive to pickup Sample2 Position
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//
//                        .strafeToLinearHeading(yellowSample2Position.position, yellowSample2Position.heading)
//                        .build());


        // Pick up Sample2 and prepare to score in the high basket
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        safeWaitSeconds(0.5);
        if (opModeIsActive()) robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
        safeWaitSeconds(0.2);
        if (opModeIsActive()) mechOps.auto7SampleScorePrep();
        //safeWaitSeconds(0.1);

        // drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetServoPositionScoreSample())
                        .waitSeconds(.5)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        // Release sample2 into the high basket
        if (opModeIsActive()) mechOps.scoreClawOpen();
    }

    // method to retrieve the third sample from the field and score into the high basket
    public void scoreSample3(PinpointDrive drive) {


        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetLiftPosition(params.LIFT_RESET))
                        .build());

        // reset the lift to prepare to grab the next sample
        //if (opModeIsActive()) mechOps.liftReset();

        if (opModeIsActive()) mechOps.scoreForeHold();

        // Drive to Sample3 Position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose3.position, midwayPose3.heading)
                        .strafeToLinearHeading(yellowSample3Position.position, yellowSample3Position.heading)
                        .build());

        // prepare the mechanisms for grabbing sample 3
        if (opModeIsActive()) mechOps.extClawRotateNinety();
        //if (opModeIsActive()) mechOps.extClawRotate45(); POSSIBILITY 1
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        //if (opModeIsActive()) mechOps.autoExtension();
        if (opModeIsActive()) mechOps.autoExtensionThirdSample(); //POSSIBILITY 2
        safeWaitSeconds(0.5);
        if (opModeIsActive()) mechOps.extClawClose();
        safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.auto7SampleScorePrep();
        //safeWaitSeconds(0.5);


        // drive to sample 3 position



        // Pick up Sample3 and prepare to score in the high basket

//        if (opModeIsActive()) mechOps.extClawClose();
//        safeWaitSeconds(0.2);
//        if (opModeIsActive()) mechOps.autoSampleScorePrep();
//        safeWaitSeconds(0.5);

        // drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .stopAndAdd(new SetLiftPosition(params.LIFT_SCORE))
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetServoPositionScoreSample())
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        // Release the sample into the basket
        if (opModeIsActive()) mechOps.scoreClawOpen();

        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetServoPositionScoreReset())
                        .stopAndAdd(new SetLiftPosition(params.LIFT_RESET))
                        .build());

        // Lower the arm & reset the mechanisms
        //if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        if (opModeIsActive()) mechOps.extPitchGrab();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        if (opModeIsActive()) mechOps.autoSpecimenLiftReset();

    }

    // method to retrieve the a sample from the submersible and score into the high basket
    public void scoreSample5(PinpointDrive drive) {


        // Drive to submersible to grab Sample 5
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        .strafeToLinearHeading(yellowSample5Position.position, yellowSample5Position.heading)
                        .build());

        // Pick up Sample3 and prepare to score in the high basket
        if (opModeIsActive()) mechOps.autoExtension();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        safeWaitSeconds(0.2);
        if (opModeIsActive()) mechOps.extForeBarDeploy();
        safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.extClawClose();
        safeWaitSeconds(0.2);
        if (opModeIsActive()) mechOps.auto7SampleScorePrep();
        //safeWaitSeconds(0.5);

        // drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .stopAndAdd(new SetServoPositionScoreSample())
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        // Release the sample into the basket
        if (opModeIsActive()) mechOps.scoreClawOpen();

        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        // Lower the arm & reset the mechanisms
        if(opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        if (opModeIsActive()) mechOps.extPitchGrab();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        if (opModeIsActive()) mechOps.setExtensionPosition();

    }

    // method to retrieve the a sample from the submersible and score into the high basket
    public void scoreSample6(PinpointDrive drive) {


        if(opModeIsActive()) mechOps.extForePart();

        // Drive to submersible to grab Sample 5
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(yellowSample6Position.position, yellowSample6Position.heading)
                        .build());

        // Pick up Sample3 and prepare to score in the high basket
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.autoExtension();
        safeWaitSeconds(.2);
        if (opModeIsActive()) mechOps.extClawClose();
        safeWaitSeconds(0.2);
        if (opModeIsActive()) mechOps.auto7SampleScorePrep();
        //safeWaitSeconds(0.5);

        // drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        // Release the sample into the basket
        if (opModeIsActive()) mechOps.scoreClawOpen();

        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        // Lower the arm & reset the mechanisms
        if (opModeIsActive()) mechOps.liftReset();
        if (opModeIsActive()) robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        if (opModeIsActive()) mechOps.extPitchGrab();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        if (opModeIsActive()) mechOps.setExtensionPosition();

    }

    // method to retrieve the a sample from the submersible and score into the high basket
    public void scoreSample7(PinpointDrive drive) {

        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        // reset the lift to prepare to grab the next sample
        if (opModeIsActive()) mechOps.liftReset();

        // prepare the mechanisms for grabbing sample 3

        if (opModeIsActive()) mechOps.extForePart();

        // Drive to submersible to grab Sample 5
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(yellowSample5Position.position, yellowSample5Position.heading)
                        .build());

        // Pick up Sample3 and prepare to score in the high basket
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_OUT_MAX);
        if (opModeIsActive()) mechOps.setExtensionPosition();
        safeWaitSeconds(0.5);
        if (opModeIsActive()) mechOps.extForeBarDeploy();
        safeWaitSeconds(0.25);
        if (opModeIsActive()) mechOps.extClawClose();
        safeWaitSeconds(0.2);
        if (opModeIsActive()) mechOps.autoSampleScorePrep();

        // drive to scoring position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        // Release the sample into the basket
        if (opModeIsActive()) mechOps.scoreClawOpen();

        // Drive to prep position
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .stopAndAdd(new SetServoPositionScorePark())
                        .stopAndAdd(new SetLiftPosition(params.LIFT_RESET))
                        .build());

        // Lower the arm & reset the mechanisms
        if (opModeIsActive()) robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        if (opModeIsActive()) robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        if (opModeIsActive()) mechOps.extPitchGrab();
        if (opModeIsActive()) mechOps.extensionPosition = ((int) robot.EXTENSION_RESET);
        if (opModeIsActive()) mechOps.setExtensionPosition();
    }

    // method to part the robot
    public void park(PinpointDrive drive) {

        // Park

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2.position,midwayPose2.heading)
//                        .strafeToLinearHeading(midwayPose4.position,midwayPose4.heading)
                        //.strafeToLinearHeading(parkPrepPose.position,parkPrepPose.heading)
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

    public void selectYellowSamples(){
        boolean selectionsDone = false;
        State selectionState = State.SAMPLE_5;
        int x5=0, x6=0, x7=0;
        int y5=0, y6=0, y7=0;
        ElapsedTime delay = new ElapsedTime();

        while (!selectionsDone && !isStopRequested()){

            switch(selectionState){
                case SAMPLE_5:
                    telemetry.addLine("Select the coordinates for Sample 5");
                    telemetry.addLine("DO NOT CHANGE ANYTHING, JUST PRESS A!!!!");
                    telemetry.addLine("Press A When Done");
                    telemetry.update();
                    if (gamepad1.dpad_up && delay.time() > buttonPressDelay){
                        x5++;
                        delay.reset();
                    }else if (gamepad1.dpad_down && delay.time() > buttonPressDelay){
                        x5--;
                        delay.reset();
                    }else if (gamepad1.dpad_right && delay.time() > buttonPressDelay){
                        y5--;
                        delay.reset();
                    }else if (gamepad1.dpad_left && delay.time() > buttonPressDelay){
                        y5++;
                        delay.reset();
                    }else if(gamepad1.a && delay.time() > 0.5) {
                        selectionState = State.SAMPLE_6;
                        yellowSample5Position = new Pose2d(x5 + xAxisOffset, y5 +yAxisOffset, Math.toRadians(90));
                        delay.reset();
                    }
                    break;

                case SAMPLE_6:
                    telemetry.addLine("Select the coordinates for Sample 6");
                    telemetry.addLine("DO NOT CHANGE ANYTHING, JUST PRESS A!!!!");
                    telemetry.addLine("Press A When Done");
                    telemetry.update();
                    if (gamepad1.dpad_up && delay.time() > buttonPressDelay){
                        x6++;
                        delay.reset();
                    }else if (gamepad1.dpad_down && delay.time() > buttonPressDelay){
                        x6--;
                        delay.reset();
                    }else if (gamepad1.dpad_right && delay.time() > buttonPressDelay){
                        y6++;
                        delay.reset();
                    }else if (gamepad1.dpad_left && delay.time() > buttonPressDelay){
                        y6--;
                        delay.reset();
                    }else if(gamepad1.a && delay.time() > 0.5) {
                        selectionState = State.SAMPLE_7;
                        yellowSample6Position = new Pose2d(x6 + xAxisOffset, y6 + yAxisOffset, Math.toRadians(-90));
                        delay.reset();
                    }
                    break;

                case SAMPLE_7:
                    telemetry.addLine("Select the coordinates for Sample 7");
                    telemetry.addLine("DO NOT CHANGE ANYTHING, JUST PRESS A!!!!");
                    telemetry.addLine("Press A When Done");
                    telemetry.update();
                    if (gamepad1.dpad_up && delay.time() > buttonPressDelay){
                        x7++;
                        delay.reset();
                    }else if (gamepad1.dpad_down && delay.time() > buttonPressDelay){
                        x7--;
                        delay.reset();
                    }else if (gamepad1.dpad_right && delay.time() > buttonPressDelay){
                        y7++;
                        delay.reset();
                    }else if (gamepad1.dpad_left && delay.time() > buttonPressDelay){
                        y7--;
                        delay.reset();
                    }else if(gamepad1.a && delay.time() > 0.5) {
                        selectionState = State.EXIT;
                        yellowSample7Position = new Pose2d(x7 + xAxisOffset, y7 + yAxisOffset, Math.toRadians(-90));
                        delay.reset();
                    }
                    break;

                case EXIT:
                    selectionsDone = true;
                    break;
            }
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
