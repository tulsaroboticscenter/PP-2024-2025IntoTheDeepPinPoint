package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;

import java.util.Locale;

/** @noinspection ALL*/
@TeleOp(name="Worlds Best Teleop", group="Robot")
//@Disabled
public class WorldsBestTeleopFINAL extends LinearOpMode {


    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    private final RRMechOps mechOps = new RRMechOps(robot,opMode);


    public static double NEW_P = 15;
    public static double NEW_I = 1;
    public static double NEW_D = 0.001;
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
        int clawRotateButtonPress = 1;
        int liftButtonPress = 1;
        int clawGrabButtonPress = 1;
        int scoreClawButtonPress = 1;
        ElapsedTime clawRotateButtonPressTime = new ElapsedTime();
        ElapsedTime extensionButtionPressTime = new ElapsedTime();
        ElapsedTime liftButtonPressTime = new ElapsedTime();
        ElapsedTime clawGrabButtonPressTime = new ElapsedTime();
        ElapsedTime scoreClawGrabButtonPressTime = new ElapsedTime();





        robot.init(hardwareMap, true);


        Motor motorLiftBack = new Motor(hardwareMap, "motorLiftR", Motor.GoBILDA.RPM_435 );
        Motor motorLiftFront = new Motor(hardwareMap, "motorLiftF", Motor.GoBILDA.RPM_435);
        Motor motorLiftTop = new Motor(hardwareMap, "motorLiftT", Motor.GoBILDA.RPM_435);

        motorLiftBack.setInverted(false);
        motorLiftTop.setInverted(false);
        motorLiftFront.setInverted(true);
        motorLiftTop.resetEncoder();
        motorLiftFront.resetEncoder();
        motorLiftBack.resetEncoder();
        motorLiftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorLiftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

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
        clawRotateButtonPressTime.reset();
        extensionButtionPressTime.reset();
        liftButtonPressTime.reset();
        clawGrabButtonPressTime.reset();
        scoreClawGrabButtonPressTime.reset();
        mechOps.extForeBarRetract();
        mechOps.scoreForeHold();
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
        ElapsedTime liftRuntime = new ElapsedTime();
        ElapsedTime clawGrabRuntime = new ElapsedTime();
        ElapsedTime scoreClawGrabRuntime = new ElapsedTime();
        ElapsedTime l2ClimbDeployRuntime = new ElapsedTime();



        totalRuntime.reset();
        clawRuntime.reset();
        scoreClawRuntime.reset();
        rotateClawRuntime.reset();
        armExtensionRuntime.reset();
        armClimbRuntime.reset();
        liftRuntime.reset();
        clawGrabRuntime.reset();
        scoreClawGrabRuntime.reset();



        // booleans for keeping track of toggles
        boolean clawRotated = false;
        boolean armRetracted = true;
        boolean armClimb = false;
        boolean isTransferReady = false;
        boolean l2ClimbDeploy = false;


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
                if((clawGrabButtonPress == 1) && (clawGrabButtonPressTime.time() > 0.2)){
                    robot.extGrabServo.setPosition(robot.INTAKE_CLAW_OPEN);

                    clawGrabButtonPress = 2;
                    clawGrabButtonPressTime.reset();

                } else if ((clawGrabButtonPress == 2) && (clawGrabButtonPressTime.time() > 0.2)) {
                    robot.extGrabServo.setPosition(robot.INTAKE_CLAW_CLOSED);

                    clawGrabButtonPress = 1;
                    clawGrabButtonPressTime.reset();

                }
                clawGrabRuntime.reset();
            }


            if (gamepad1.left_bumper && scoreClawRuntime.time() > 0.5) {
                if((scoreClawButtonPress == 1) && (scoreClawGrabButtonPressTime.time() > 0.2)){
                    robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_CLOSED);

                    scoreClawButtonPress = 2;
                    scoreClawGrabButtonPressTime.reset();

                } else if ((scoreClawButtonPress == 2) && (scoreClawGrabButtonPressTime.time() > 0.2)) {
                    robot.scoreGrabServo.setPosition(robot.SCORE_CLAW_OPEN_TELEOP);

                    scoreClawButtonPress = 1;
                    scoreClawGrabButtonPressTime.reset();

                }
                scoreClawGrabRuntime.reset();
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

            if (gamepad1.right_stick_button) {
                if((clawRotateButtonPress == 1) && (clawRotateButtonPressTime.time() > 0.2)){
                    robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);

                    clawRotateButtonPress = 2;
                    clawRotateButtonPressTime.reset();

                } else if ((clawRotateButtonPress == 2) && (clawRotateButtonPressTime.time() > 0.2)){
                    robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);

                    clawRotateButtonPress = 1;
                    clawRotateButtonPressTime.reset();


                }

            }


            if (gamepad1.y) {

                if((liftButtonPress == 1) && (liftButtonPressTime.time() > 0.2)){
                    robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_GRAB);
                    robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_GRAB);

                    liftButtonPress = 2;
                    liftButtonPressTime.reset();

                }else if((liftButtonPress==2)&&(liftButtonPressTime.time() > 0.2)){
                    liftPosition = robot.LIFT_SCORE_HIGH_BASKET;
                    robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE_PART);
                    robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE_PART);
                    scoreClawButtonPress = 2;

                    liftButtonPress = 3;
                    liftButtonPressTime.reset();

                } else if ((liftButtonPress == 3) && (liftButtonPressTime.time() > 0.2)){
                    liftPosition = robot.LIFT_SCORE_HIGH_BASKET_TELEOP;
                    robot.scoreForeLeftServo.setPosition(robot.SCORE_LEFT_FOREBAR_SCORE);
                    robot.scoreForeRightServo.setPosition(robot.SCORE_RIGHT_FOREBAR_SCORE);
                    scoreClawButtonPress = 2;



                    liftButtonPress = 1;
                    liftButtonPressTime.reset();
                }
            }

            if (gamepad1.a) {
                if((extensionButtonPress == 1) && (extensionButtionPressTime.time() > 0.2)) {
                    /* This is the intaking/collecting arm position */
//                    robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                    mechOps.extensionPosition = (int) robot.EXTENSION_OUT_MAX;
                    robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_PREP);
                    robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY_PART);
                    robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY_PART);

                    extensionButtonPress = 2;
                    extensionButtionPressTime.reset();
                } else if((extensionButtonPress == 2) && extensionButtionPressTime.time() > 0.2){
                    robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                    robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
                    robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
//                    mechOps.extensionPosition = (int) robot.EXTENSION_OUT_MAX;
//                    robot.extForeLeftServo.setPosition(robot.INTAKE_LEFT_FOREBAR_DEPLOY);
//                    robot.extForeRightServo.setPosition(robot.INTAKE_RIGHT_FOREBAR_DEPLOY);
                    extensionButtonPress = 3;
                    extensionButtionPressTime.reset();

                }else if((extensionButtonPress == 3) && extensionButtionPressTime.time() > 0.2){
                    robot.extPitchServo.setPosition(robot.INTAKE_CLAW_PITCH_GRAB);
                    mechOps.extForeBarRetractHalf();
                    extensionButtonPress = 1;
                    extensionButtionPressTime.reset();
                }

            } else if (gamepad1.x){
                mechOps.transferSample = true;
                clawGrabButtonPress = 1;
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

//            } else if (gamepad1.y) {
//                liftPosition = robot.LIFT_SCORE_HIGH_BASKET_TELEOP;
//                mechOps.scoreForeSample();

            } else if (gamepad2.a){
                liftPosition = robot.LIFT_RESET_TELEOP;
                mechOps.scoreForeSpecimen();

            } else if (gamepad1.b){
                liftPosition = robot.LIFT_RESET_TELEOP;
                mechOps.scoreForeHold();
                liftButtonPress = 2;
                clawGrabButtonPress = 2;

            }

            if (gamepad1.dpad_right) {
                robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_45);
                liftPosition = robot.LIFT_RESET;
                mechOps.scoreForeSpecimen();


            } else if (gamepad1.dpad_up) {
                liftPosition = robot.LIFT_SPECIMEN_PREP_TELEOP;
                robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_ZERO);

            } else if (gamepad1.dpad_left) {
                mechOps.scoreForeSpecimen();
                liftPosition = robot.LIFT_RESET_TELEOP;

            } else if (gamepad2.dpad_up) {
                liftPosition = robot.LIFT_SPECIMEN_PREP_TELEOP;

            } else if (gamepad1.dpad_down){
                robot.extRotateServo.setPosition(robot.INTAKE_WRIST_ROTATED_NINETY);
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


            } else if (gamepad1.left_stick_button){
                mechOps.extensionPosition = (int) robot.EXTENSION_RESET;
                extensionButtonPress = 1;
                extensionButtionPressTime.reset();
            }


            if (gamepad2.left_trigger > 0.05 && (mechOps.extensionPosition + (40 * -gamepad2.right_stick_y)) > 0 && (mechOps.extensionPosition + (40 * -gamepad2.right_stick_y)) < robot.EXTENSION_DOWN_MAX){
                mechOps.extensionPosition += (20 * -gamepad2.right_stick_y);
            }

            if (gamepad2.right_trigger > 0.05 && (liftPosition + (20 * -gamepad2.right_stick_y)) < robot.LIFT_SCORE_HIGH_BASKET && (liftPosition + (20 * -gamepad2.right_stick_y)) > robot.LIFT_RESET){
                liftPosition += (100 * -gamepad2.right_stick_y);
            }


            if(gamepad2.right_bumper && !l2ClimbDeploy){
                l2ClimbDeployRuntime.reset();
                l2ClimbDeploy = true;
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

            if(gamepad2.left_bumper){
                climb();
                l2ClimbDeploy = false;
            }

            if (l2ClimbDeploy && l2ClimbDeployRuntime.time() < 0.1 && gamepad2.right_bumper) {
                mechOps.l2Up();
            } else if (l2ClimbDeploy && l2ClimbDeployRuntime.time() > 5.75) {
                mechOps.l2Stop();
            }
            if(gamepad2.x){
                liftPosition = robot.LIFT_CLIMB_SECURE;
            }

            motorLiftFront.setTargetPosition((int) liftPosition);
            motorLiftBack.setTargetPosition((int) liftPosition);
            motorLiftTop.setTargetPosition((int) liftPosition);



            if(motorLiftTop.getCurrentPosition() == liftPosition){
                motorLiftTop.set(1);
                motorLiftBack.set(0);
                motorLiftFront.set(0);
            } else {
                motorLiftTop.set(1);
                motorLiftBack.set(1);
                motorLiftFront.set(1);
            }

            telemetry.addData("liftPosition = ", mechOps.liftPosition);
            telemetry.addData("extensionPosition = ", mechOps.extensionPosition);
            telemetry.addData("motor Lift Front Position", robot.motorLiftFront.getCurrentPosition());
            telemetry.addData("motor Lift Back Position", robot.motorLiftBack.getCurrentPosition());
            telemetry.addData("motor Lift Top Position", robot.motorLiftTop.getCurrentPosition());
            telemetry.addData("motor Extend Position", robot.extendMotor.getCurrentPosition());
            telemetry.addData("motor Lift Front Current", robot.motorLiftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Lift Back Current", robot.motorLiftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Lift Top Current", robot.motorLiftTop.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Extend Current", robot.extendMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("score claw position", robot.scoreGrabServo.getPosition());
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


    public void climb(){

        robot.motorLiftFront.setTargetPosition((int)robot.LIFT_CLIMB_SECURE);
        robot.motorLiftBack.setTargetPosition((int)robot.LIFT_CLIMB_SECURE);
        robot.motorLiftTop.setTargetPosition((int)robot.LIFT_CLIMB_SECURE);
        mechOps.l2Down();
        sleep(2000);
        mechOps.l2Stop();
        int liftPosition = (robot.motorLiftBack.getCurrentPosition() + robot.motorLiftFront.getCurrentPosition() + robot.motorLiftTop.getCurrentPosition())/3;
        while(opModeIsActive()){
            if(liftPosition > 20) {
                robot.motorLiftFront.setTargetPosition((int) robot.LIFT_RESET_CLIMB);
                robot.motorLiftBack.setTargetPosition((int) robot.LIFT_RESET_CLIMB);
                robot.motorLiftTop.setTargetPosition((int)robot.LIFT_RESET_CLIMB);
            }

            telemetry.addData("liftPosition = ", mechOps.liftPosition);
            telemetry.addData("motor Lift Front Position", robot.motorLiftFront.getCurrentPosition());
            telemetry.addData("motor Lift Back Position", robot.motorLiftBack.getCurrentPosition());
            telemetry.addData("motor Lift Top Position", robot.motorLiftTop.getCurrentPosition());
            telemetry.addData("motor Lift Front Current", robot.motorLiftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Lift Back Current", robot.motorLiftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Lift Top Current", robot.motorLiftTop.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor Extend Current", robot.extendMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addLine("----------------------------------------");
            //telemetry.addData("Time Total", totalRuntime.time());
            telemetry.update();
        }

    }

    public void setLiftPositionRWE(){
        double motorPower = 1;
        int currentPosition = (robot.motorLiftBack.getCurrentPosition() + robot.motorLiftFront.getCurrentPosition())/2;
        if(currentPosition < liftPosition){
            robot.motorLiftBack.setPower(1);
            robot.motorLiftFront.setPower(1);
        } else if (currentPosition > liftPosition){
            robot.motorLiftFront.setPower(-1);
            robot.motorLiftBack.setPower(-1);
        }

    }


}