package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

import java.io.File;

public class RRMechOps {

    // variables to control sampleTransfer function
    public boolean transferSample = false;
    public ElapsedTime sampleTransferTime = new ElapsedTime();
    public ElapsedTime twoStageTransferTime = new ElapsedTime();
    public boolean transferReady = false;

    public HWProfile robot;
    public LinearOpMode opMode;
    public int extensionPosition = 0;
    public int liftPosition = 0;


    public RRMechOps(HWProfile myRobot, LinearOpMode myOpMode) {
        robot = myRobot;
        opMode = myOpMode;

    }

    public void scoreForeSpecimen(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
    }

    public void extClawOpen() {
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void extPitchReset(){robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);}

    public void extClawClose() {
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
    }

    public void extClawRotateZero() {
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
    }

    public void extClawRotateNinety() {
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
    }

    public void extClawRotate180() {
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_180);
    }

    public void extForeBarDeploy() {
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
    }

    public void extForeBarRetract() {
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
    }

    public void scoreForeReset() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_RESET);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_RESET);
    }

    public void extForePart() {
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY_PART);
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY_PART);
    }

    public void scoreForeGrab() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_GRAB);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_GRAB);

    }


    public void scoreForeSample() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE);

    }

    public void sampleTransferPrep() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_RESET);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_RESET);
    }

    public void specimenPrepPosition() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
        robot.motorLiftTop.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
    }

    public void specimenPrepPositionCRI() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
        robot.motorLiftTop.setTargetPosition((int)robot.LIFT_SPECIMEN_PREP);
    }

    public void specimenPrepPositionCRILow() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_HOLD);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_HOLD);

    }

    public void specimenPrepPositionCRILowTeleop() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_HOLD_TELEOP);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_HOLD_TELEOP);

    }

    public void specimenScorePosition() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_SPECIMEN_SCORE);
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_SPECIMEN_SCORE);
        robot.motorLiftTop.setTargetPosition((int)robot.LIFT_SPECIMEN_SCORE);
    }

    public void specimenScorePositionLow() {
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE_LOW);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE_LOW);

    }

    public void liftPark(){
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_PARK);
        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_PARK);
        robot.motorLiftTop.setTargetPosition((int)robot.LIFT_PARK);
    }

    public void raiseLiftHighBasket() {
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftTop.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftTop.setPower(1);
    }
    public void lowBarScore() {
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftTop.setTargetPosition((int) robot.LIFT_RESET);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE_LOW);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE_LOW);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftTop.setPower(1);
    }
    public void raiseLiftHighBasketTeleop() {
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET_TELEOP);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET_TELEOP);
        robot.motorLiftTop.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET_TELEOP);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftTop.setPower(1);
        scoreForeSample();
    }


    public void liftReset() {
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftTop.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftTop.setPower(1);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_GRAB);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_GRAB);
    }

    public void autoSpecimenLiftReset(){
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftTop.setPower(1);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_RESET);
        robot.motorLiftTop.setTargetPosition((int) robot.LIFT_RESET);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SPECIMEN);
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SPECIMEN);
    }

    public void raiseLiftHighBasketPrep() {
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftTop.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftTop.setPower(1);
        scoreForeSample();
    }

    public void raiseLiftHighBasketPrep7() {
        robot.motorLiftBack.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftTop.setTargetPosition((int) robot.LIFT_SCORE_HIGH_BASKET);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftBack.setPower(1);
        robot.motorLiftTop.setPower(1);

    }

    public void autoExtension(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);

    }
    public void autoExtensionThirdSample(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_PARTIAL_OPEN);

    }

    public void autoMechanismReset(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);

    }
    public void extForeBarSweep(){
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY_AUTON);
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY_AUTON);
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_AUTON);
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
        robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
    }

    public void foreBarUp(){
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY_PART);
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY_PART);
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);
    }
    public void extPitchGrab(){
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
    }

    public void scoreForeHold(){
        robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_HOLD);
        robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_HOLD);
    }

    public void extForeBarPart(){
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT_PART);
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT_PART);
    }
    public void extPitchHold(){
        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_HOLD);
    }

    public void extForeBarRetractHalf(){
        robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT_HALF);
        robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT_HALF);
    }

    public void l2Down(){
        robot.level3RightServo.setPosition(robot.L3DOWNRIGHT);
        robot.level3LeftServo.setPosition(robot.L3DOWNLEFT);
    }

    public void l2Up(){
        robot.level3LeftServo.setPosition(robot.L3UPLEFT);
        robot.level3RightServo.setPosition(robot.L3UPRIGHT);
    }

    public void l2Stop(){
        robot.level3RightServo.setPosition(robot.L3HALFR);
        robot.level3LeftServo.setPosition(robot.L3HALFL);
    }
    public void extClawRotate45(){
        robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_45);
    }
    /**
     * Method: transferSample()
     * How it works:
     * - transferSample() is in the base loop of the code. The boolean condition transferSample is
     * checked every cycle. If the condition is not true, the method is exited with no changes other
     * than updating telemetry as requested.
     * - In the base opMode, setting the global boolean transferSample variable to true initiates
     * the process of transfering the sample to the scoring sample.
     * - When transferSample is true, but transferReady is false:
     * * the opmode will assume that the extClaw is closed.
     * * The extForeBar will be retracted to handoff position
     * * The extension arms will be retracted to handoff position
     * * The extClaw will be rotated to the 0 degree position
     * * The scoreClaw will be placed in the samplePrep position
     * * The system will verify that the extendMotor encoder position has been retracted before
     * moving to the next step of handing off the sample to the scoring claw.
     * - When transferSample is true and transferReady is true:
     * * This condition indicates that the scoreClaw should close and the extClaw should release
     * the sample. Once the sample is released by the extClaw, the score claw should extend to
     * scoring position
     * * Note, it is likely that a delay will be needed before opening the extClaw
     */
    public void transferSample() {
        opMode.telemetry.addData("Transfer Sample = ", transferSample);

        if (transferSample) {
            if (transferReady) {
                scoreClawClosed();

                // allow time for the score claw to close before opening the extClaw
                if (sampleTransferTime.time() > 0.100){
                    extClawOpen();
                }

                // allow time for the extClaw to open before extending the scoring claw
                if (sampleTransferTime.time() > 0.450) {
                    // move the scoring arm into position
//                    raiseLiftHighBasket();
                    transferSample = false;
                    transferReady = false;
                }
            } else {
                scoreForeGrab();
                extPitchGrab();
                extForeBarRetract();
                this.extensionPosition = (int) robot.EXTENSION_RESET_TELEOP;
                extClawRotateZero();

                scoreForeGrab();
                scoreClawOpen();
                if (robot.extendMotor.getCurrentPosition() <= robot.EXTENSION_RESET_TELEOP) {
                    transferReady = true;
                    sampleTransferTime.reset();
                }
            }
        }
    }

    public void twoStageTransfer(int stage) {
        if (stage == 1) {
            extForeBarRetract();
            this.extensionPosition = (int) robot.EXTENSION_RESET;
            extClawRotateZero();
            extPitchReset();
            scoreForeGrab();
            scoreClawOpen();
        } else if (stage == 2) {
            twoStageTransferTime.reset();

            scoreClawClosed();

            // allow time for the score claw to close before opening the extClaw
            if (twoStageTransferTime.time() > 0.150) {
                extClawOpen();
            }

            // allow time for the extClaw to open out of the way before moving the score claw
            if(twoStageTransferTime.time() > 0.4) {
                scoreForeSample();
            }

        }
    }

    public void extensionPowerMonitor(){
        if (robot.extendMotor.getCurrentPosition() < robot.EXTENSION_POWER_REDUX) {
            robot.extendMotor.setPower(0.5);
        } else robot.extendMotor.setPower(1);
    }


    public void setExtensionPosition(){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(this.extensionPosition);
    }

    public void setLiftPosition(){
        robot.motorLiftBack.setPower(1);
        robot.motorLiftBack.setTargetPosition(this.liftPosition);
        robot.motorLiftFront.setPower(1);
        robot.motorLiftFront.setTargetPosition(this.liftPosition);
        robot.motorLiftTop.setPower(1);
        robot.motorLiftTop.setTargetPosition(this.liftPosition);
    }

    public void setLiftPositionTeleop(){
        robot.motorLiftBack.setTargetPosition(this.liftPosition);
        robot.motorLiftFront.setTargetPosition(this.liftPosition);
        robot.motorLiftTop.setTargetPosition(this.liftPosition);

        if(robot.motorLiftTop.getCurrentPosition() == liftPosition){
            robot.motorLiftTop.setPower(1);
            robot.motorLiftBack.setPower(0);
            robot.motorLiftFront.setPower(0);
        } else {
            robot.motorLiftTop.setPower(1);
            robot.motorLiftBack.setPower(1);
            robot.motorLiftFront.setPower(1);
        }

    }

    public void setAutoExtensionPosition(){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(this.extensionPosition);
    }

    public void tightenStrings() {
        boolean extensionRetraction = false;
        boolean liftRetraction = false;
        ElapsedTime retractTime = new ElapsedTime();
        retractTime.reset();


        this.extensionPosition = 0;
        setExtensionPosition();
        robot.motorLiftFront.setPower(.75);
        robot.motorLiftBack.setPower(.75);
        robot.motorLiftTop.setPower(.75);
        robot.motorLiftFront.setTargetPosition(0);
        robot.motorLiftBack.setTargetPosition(0);
        robot.motorLiftTop.setTargetPosition(0);
        robot.extendMotor.setPower(0.5);
        robot.extendMotor.setTargetPosition(0);

        while (opMode.opModeIsActive() && (!extensionRetraction || !liftRetraction)) {

            if ((retractTime.time() > 0.2) && robot.extendMotor.getCurrent(CurrentUnit.AMPS) > 5 && !extensionRetraction) {
                extensionRetraction = true;
                robot.extendMotor.setPower(0);
                robot.extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.extendMotor.setTargetPosition(0);
                robot.extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else if (!extensionRetraction){
                this.extensionPosition = this.extensionPosition - 25;
                setExtensionPosition();
            }

            if ((retractTime.time() > 0.1) && (robot.motorLiftTop.getCurrent(CurrentUnit.AMPS) > 2.5) && !liftRetraction) {
                liftRetraction = true;
                robot.motorLiftFront.setPower(0);
                robot.motorLiftBack.setPower(0);
                robot.motorLiftTop.setPower(0);
                robot.motorLiftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorLiftFront.setTargetPosition(0);
                robot.motorLiftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLiftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorLiftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorLiftBack.setTargetPosition(0);
                robot.motorLiftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLiftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorLiftTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.motorLiftTop.setTargetPosition(0);
                robot.motorLiftTop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.motorLiftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else if(!liftRetraction) {
                this.liftPosition = this.liftPosition - 30;
                setLiftPosition();
            }

        }

        this.extensionPosition = (int)robot.EXTENSION_RESET_TELEOP;
        setExtensionPosition();
        this.liftPosition = (int)robot.LIFT_RESET_TELEOP;
        setLiftPosition();
    }

    public void scoreClawOpen() {
        robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_OPEN);
    }

    public void scoreClawClosed() {
        robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_CLOSED);
    }

    public void sampleHandoff() {
        //close extension claw
        //set wrist rotation 0
        //extensionRetracted
        //scoreclaw hold position
        //extForeBarRetract
        //lift retracted
        //move scoreclaw to grab position
        //close score claw
        //safe wait
        //when lift, open up intake claw


    }

    public void autoSampleScorePrep() {

        liftReset();
        extClawClose();
        scoreForeGrab();
        scoreClawOpen();
        opMode.sleep(200);


        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        extForeBarRetract();
        extClawRotateZero();

        this.extensionPosition = (int) robot.EXTENSION_RESET;
        setExtensionPosition();
        opMode.sleep(500);


        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
        opMode.sleep(200);
        scoreClawClosed();
        opMode.sleep(100);
        extClawOpen();
        opMode.sleep(100);

        raiseLiftHighBasket();
        scoreForeSample();

    }

    public void auto5SampleScorePrep() {

        liftReset();
        extClawClose();
        scoreForeGrab();
        scoreClawOpen();
        opMode.sleep(200);


        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);

        extForeBarRetract();
        extClawRotateZero();

        this.extensionPosition = (int) robot.EXTENSION_RESET;
        setAutoExtensionPosition();
        opMode.sleep(1000);


        scoreClawClosed();
        opMode.sleep(100);
        extClawOpen();
        opMode.sleep(100);

        raiseLiftHighBasket();
        scoreForeSample();

    }
    public void autoSampleCRI() {


        extClawClose();
        opMode.sleep(100);

        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);

        extForeBarRetract();
        extClawRotateZero();

        this.extensionPosition = (int) robot.EXTENSION_RESET;
        setAutoExtensionPosition();
        opMode.sleep(750);
        scoreClawClosed();
        opMode.sleep(100);
        extClawOpen();
        opMode.sleep(100);
        scoreForeSample();
        opMode.sleep(150);



    }
    public void autoSampleCRIDrop() {


        extClawClose();
        opMode.sleep(100);

        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);

        extForeBarRetract();
        extClawRotateZero();

        this.extensionPosition = (int) robot.EXTENSION_RESET;
        setAutoExtensionPosition();
        opMode.sleep(750);
        scoreClawClosed();
        opMode.sleep(100);
        extClawOpen();
        opMode.sleep(100);
        scoreForeSpecimen();
        extClawRotateNinety();
        opMode.sleep(150);



    }
    public void auto7SampleScorePrep() {

        liftReset();
        extClawClose();
        scoreForeGrab();
        scoreClawOpen();
        opMode.sleep(200);


        robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);

        extForeBarRetract();
        extClawRotateZero();

        this.extensionPosition = (int) robot.EXTENSION_RESET;
        setAutoExtensionPosition();
        opMode.sleep(650);


        scoreClawClosed();
        opMode.sleep(100);
        extClawOpen();
        opMode.sleep(100);

        raiseLiftHighBasket();

    }

    public void writeToFile (double headingValue, String fileName){
        File headingValueAfterAuto = AppUtil.getInstance().getSettingsFile(fileName);
        ReadWriteFile.writeFile(headingValueAfterAuto, String.valueOf(headingValue));
    }

    public double readFromFile (String fromFileName) {

        // Using the properties of the specified "from" file name,
        // declare a filename to be used in this method.  See Note 1 above.
        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        // Read and store a number from the newly declared filename.
        // See Note 4 above.
        double myNumber = Double.parseDouble(ReadWriteFile.readFile(myFileName).trim());

        return myNumber;       // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()

}

