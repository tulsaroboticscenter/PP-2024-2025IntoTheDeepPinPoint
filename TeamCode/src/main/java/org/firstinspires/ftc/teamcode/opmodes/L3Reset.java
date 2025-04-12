package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
@TeleOp(name="LEVEL 3 RESET", group="Robot")
//@Disabled
public class L3Reset extends LinearOpMode {


    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    private final RRMechOps mechOps = new RRMechOps(robot,opMode);


    public static double NEW_P = 15;
    public static double NEW_I = 0;
    public static double NEW_D = 0.001;
    public static double NEW_F = 1;


    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double elbowLiftComp = 0;


    /* Variables that are used to set the arm to a specific position */
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
        int extensionButtonPress = 1;
        int clawRotateButtonPress = 1;
        int liftButtonPress = 1;
        ElapsedTime clawRotateButtonPressTime = new ElapsedTime();
        ElapsedTime extensionButtionPressTime = new ElapsedTime();
        ElapsedTime liftButtonPressTime = new ElapsedTime();




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
        clawRotateButtonPressTime.reset();
        extensionButtionPressTime.reset();
        liftButtonPressTime.reset();
        mechOps.l2Down();


        // Initializes ElapsedTimes. One for total runtime of the program and the others set up for toggles.
        ElapsedTime totalRuntime = new ElapsedTime();
        ElapsedTime clawRuntime = new ElapsedTime();
        ElapsedTime scoreClawRuntime = new ElapsedTime();
        ElapsedTime rotateClawRuntime = new ElapsedTime();
        ElapsedTime armExtensionRuntime = new ElapsedTime();
        ElapsedTime armClimbRuntime = new ElapsedTime();
        ElapsedTime twoStageTransferRuntime = new ElapsedTime();
        ElapsedTime liftRuntime = new ElapsedTime();

        totalRuntime.reset();
        clawRuntime.reset();
        scoreClawRuntime.reset();
        rotateClawRuntime.reset();
        armExtensionRuntime.reset();
        armClimbRuntime.reset();
        liftRuntime.reset();



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

            if(gamepad1.a){
                mechOps.l2Down();
            }

            if(gamepad1.b){
                mechOps.l2Stop();
            }

            if (gamepad1.x){
                mechOps.l2Up();
            }
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