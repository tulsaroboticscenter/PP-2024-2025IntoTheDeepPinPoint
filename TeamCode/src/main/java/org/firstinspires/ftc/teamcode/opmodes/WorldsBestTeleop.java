package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Locale;

/** @noinspection ALL*/
@TeleOp(name="Worlds Best Teleop gang", group="Robot")
//@Disabled
public class WorldsBestTeleop extends LinearOpMode {


    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    private final RRMechOps mechOps = new RRMechOps(robot,opMode);


    public static double NEW_P = 20;
    public static double NEW_I = 3;
    public static double NEW_D = 0.005;
    public static double NEW_F = 1;


    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double elbowLiftComp = 0;


    /* Variables that are used to set the arm to a specific position */
    double liftPosition = robot.LIFT_RESET_TELEOP;
    double elbowPositionFudgeFactor;
    private HardwareMap hwMap;


    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double servoWristPosition = robot.INTAKE_WRIST_ROTATED_ZERO;
        int extensionButtonPress = 1;
        ElapsedTime extensionButtionPressTime = new ElapsedTime();



        robot.init(hardwareMap, true);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();


        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        double storedHeading = mechOps.readFromFile("HeadingFile");
        double botHeading = robot.pinpoint.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("Stored Heading from File", storedHeading);
        telemetry.addData("Current Bot Heading", botHeading);
        telemetry.update();

        PIDFCoefficients pidfOrig = robot.extendMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        robot.extendMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        PIDFCoefficients pidfModified = robot.extendMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        /* Wait for the game driver to press play */
        waitForStart();
        extensionButtionPressTime.reset();
        mechOps.extForeBarRetract();
        mechOps.scoreForeGrab();
        mechOps.extClawRotateZero();
        mechOps.extClawOpen();
        mechOps.scoreClawOpen();
        mechOps.extClawClose();
        mechOps.tightenStrings();
        mechOps.extPitchGrab();

        // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
        ElapsedTime totalRuntime = new ElapsedTime();
        ElapsedTime clawRuntime = new ElapsedTime();
        ElapsedTime scoreClawRuntime = new ElapsedTime();
        ElapsedTime rotateClawRuntime = new ElapsedTime();
        ElapsedTime armExtensionRuntime = new ElapsedTime();
        ElapsedTime armClimbRuntime = new ElapsedTime();
        ElapsedTime twoStageTransferRuntime = new ElapsedTime();

        totalRuntime.reset();
        clawRuntime.reset();
        scoreClawRuntime.reset();
        rotateClawRuntime.reset();
        armExtensionRuntime.reset();
        armClimbRuntime.reset();


        // booleans for keeping track of toggles
        boolean clawOpened = false;
        boolean clawRotated = false;
        boolean armRetracted = true;
        boolean armClimb = false;
        boolean scoreClawOpened = false;
        boolean isTransferReady = false;


        TelemetryPacket packet = new TelemetryPacket();


//        requestOpModeStop();
        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                robot.pinpoint.resetYaw();
                storedHeading = 0;
            }

            robot.pinpoint.update();    //update the IMU value
            Pose2D pos = robot.pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            //botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = Math.toRadians(pos.getHeading(AngleUnit.DEGREES) - storedHeading);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.leftFrontDrive.setPower(frontLeftPower);
            robot.leftBackDrive.setPower(backLeftPower);
            robot.rightFrontDrive.setPower(frontRightPower);
            robot.rightBackDrive.setPower(backRightPower);

            mechOps.liftPosition = (int)liftPosition;
            mechOps.setLiftPosition();

            mechOps.transferSample();
            mechOps.setExtensionPosition();
            //mechOps.extensionPowerMonitor();

            /* Here we handle the three buttons that have direct control of the intake speed.
            These control the continuous rotation servo that pulls elements into the robot,
            If the user presses A, it sets the intake power to the final variable that
            holds the speed we want to collect at.
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/

            /* TECH TIP: If Else statement:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intake will turn on. If we just had
            three if statements, then it will set the intake servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */

            // a boolean to keep track of whether the claw is opened or closed.

            if (gamepad1.right_bumper && clawRuntime.time() > 0.15) {
                if (clawOpened) {
                    robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);
                    clawOpened = false;
                } else if (!clawOpened) {
                    robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);
                    clawOpened = true;
                }
                clawRuntime.reset();
            }

            if (gamepad1.left_bumper & scoreClawRuntime.time() > 0.15) {
                if (scoreClawOpened) {
                    robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_CLOSED);
                    scoreClawOpened = false;
                } else {
                    robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_OPEN);
                    scoreClawOpened = true;
                }
                scoreClawRuntime.reset();
            }

            if (gamepad1.right_stick_button && rotateClawRuntime.time() > 0.15) {
                if (clawRotated) {
                    servoWristPosition = robot.INTAKE_WRIST_ROTATED_ZERO;
                    clawRotated = false;
                } else if (!clawRotated) {
                    servoWristPosition = robot.INTAKE_WRIST_ROTATED_NINETY;
                    clawRotated = true;

                    rotateClawRuntime.reset();
                }
            }


            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm down, and right trigger to move the arm up.
            So we add the left trigger's variable to the inverse of the right trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

           /* ExtensionPositionFudgeFactor = robot * EXTENSION_TICKS_PER_DEGREE(gamepad2.right_trigger + (-gamepad2.left_trigger));

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if (gamepad1.a) {
                if((extensionButtonPress == 1) && (extensionButtionPressTime.time() > 0.2)) {
                    /* This is the intaking/collecting arm position */
//                    robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                    mechOps.extensionPosition = (int) robot.EXTENSION_OUT_MAX;
                    robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                    robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY_PART);
                    robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY_PART);

                    extensionButtonPress = 2;
                    extensionButtionPressTime.reset();
                } else if((extensionButtonPress == 2) && extensionButtionPressTime.time() > 0.2){
                    robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
                    robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
//                    mechOps.extensionPosition = (int) robot.EXTENSION_OUT_MAX;
//                    robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
//                    robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
                    extensionButtonPress = 3;
                    extensionButtionPressTime.reset();

                }else if((extensionButtonPress == 3) && extensionButtionPressTime.time() > 0.2){
                    robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY_PART);
                    robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY_PART);
                    extensionButtonPress = 1;
                    extensionButtionPressTime.reset();
                }

            } else if (gamepad1.x){
                mechOps.transferSample = true;
//                // ready the transfer (stage 1)
//                if (!isTransferReady) {
//                    mechOps.twoStageTransfer(1);
//                    isTransferReady = true;
//                    if (robot.extendMotor.getCurrentPosition() <= robot.EXTENSION_RESET) {
//                        isTransferReady = true;
//                        twoStageTransferRuntime.reset();
//                    }
//                } else if (isTransferReady && twoStageTransferRuntime.time() < 5) {
//                    mechOps.twoStageTransfer(2);
//                    isTransferReady = false;
//                } else {
//                    isTransferReady = false;
//                }

            } else if (gamepad1.y) {
                liftPosition = robot.LIFT_SCORE_HIGH_BASKET_TELEOP;
                mechOps.scoreForeSample();

            } else if (gamepad2.a){
                liftPosition = robot.LIFT_RESET;
                mechOps.scoreForeSpecimen();

            } else if (gamepad1.b){
                liftPosition = robot.LIFT_RESET;
                mechOps.scoreForeGrab();
            }

            if (gamepad1.dpad_right) {
                robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);

            } else if (gamepad1.dpad_up) {
                robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);

            } else if (gamepad1.dpad_left) {
                mechOps.scoreForeSpecimen();

            } else if (gamepad1.dpad_right) {
                liftPosition = robot.LIFT_SPECIMEN_PREP;

            } else if (gamepad2.dpad_up) {
                liftPosition = robot.LIFT_SPECIMEN_PREP;

            } else if (gamepad1.dpad_down){
                liftPosition = robot.LIFT_SCORE_SPECIMEN;

            } else if (gamepad2.dpad_down){
                liftPosition = robot.LIFT_SCORE_SPECIMEN_TELEOP;

            } else if (gamepad2.dpad_left) {
                robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_CLOSED);

            }else if (gamepad2.dpad_right){
                robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_OPEN);

            } else if (gamepad2.b){
                robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_RETRACT);
                robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_RETRACT);

            } else if (gamepad2.y){
                robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
                robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
                robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);

            } else if (gamepad2.left_bumper){
                mechOps.autoSampleScorePrep();

            } else if (gamepad1.right_stick_button){
                liftPosition = robot.LIFT_CLIMB;
            }

            if (gamepad2.left_trigger > 0.05 && (mechOps.extensionPosition + (40 * -gamepad2.right_stick_y)) > 0 && (mechOps.extensionPosition + (40 * -gamepad2.right_stick_y)) < robot.EXTENSION_DOWN_MAX){
                mechOps.extensionPosition += (20 * -gamepad2.right_stick_y);
            }

            if (gamepad2.right_trigger > 0.05 && (liftPosition + (20 * -gamepad2.right_stick_y)) < robot.LIFT_SCORE_HIGH_BASKET && (liftPosition + (20 * -gamepad2.right_stick_y)) > robot.LIFT_RESET){
                liftPosition += (100 * -gamepad2.right_stick_y);
            }


            if(gamepad2.x){
                mechOps.tightenStrings();
            }

            looptime = getRuntime();
            cycletime = looptime - oldtime;
            oldtime = looptime;

            //Rumble controller for endgame and flash controller light blue
            if(totalRuntime.time() > 90 && totalRuntime.time()<90.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
                gamepad2.rumble(50);
                gamepad2.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 91 && totalRuntime.time()<91.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
                gamepad2.rumble(50);
                gamepad2.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 92 && totalRuntime.time()<92.25){
                gamepad1.rumble(50);
                gamepad1.setLedColor(255,0,0,50);
                gamepad2.rumble(50);
                gamepad2.setLedColor(255,0,0,50);
            } else if(totalRuntime.time() > 93) {
                gamepad2.setLedColor(13, 36, 65, 30000);
                gamepad1.setLedColor(13, 36, 65, 30000);
            }


            telemetry.addData("liftPosition = ", mechOps.liftPosition);
            telemetry.addData("extensionPosition = ", mechOps.extensionPosition);
            telemetry.addData("motor Lift Front Position", robot.motorLiftFront.getCurrentPosition());
            telemetry.addData("motor Lift Back Position", robot.motorLiftBack.getCurrentPosition());
            telemetry.addData("motor Extend Position", robot.extendMotor.getCurrentPosition());
            telemetry.addData("motor Lift Front Current", robot.motorLiftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Lift Back Current", robot.motorLiftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Extend Current", robot.extendMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Time Total", totalRuntime.time());
            telemetry.update();
                /*
                telemetry.addLine("Gamepad 1:");
                telemetry.addLine("Driving is Enabled.");
                telemetry.addLine("RB:          Open/Close Claw");
                telemetry.addLine("A:           Set Elbow Level to Floor");
                telemetry.addLine("Right Stick: Rotate In/Out Claw");
                telemetry.addLine("");
                telemetry.addLine("Gamepad 2:");
                telemetry.addLine("RB/LB: Extend/Contract Slides");
                telemetry.addData("lift variable", extensionPosition);
                telemetry.addData("Lift Target Position",robot.extendMotor.getTargetPosition());
                telemetry.addData("lift current position", robot.extendMotor.getCurrentPosition());
                telemetry.addData("liftMotor Current:",((DcMotorEx) robot.extendMotor).getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Claw Rotated Out: ", clawRotated);
                telemetry.addData("Claw Opened: ", clawOpened);
                telemetry.update();
*/


        }
        // reset the heading value in the HeadingFile to 0
        mechOps.writeToFile(0,"HeadingFile");
    }



}