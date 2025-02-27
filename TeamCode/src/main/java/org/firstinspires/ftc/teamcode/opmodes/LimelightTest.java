package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;



import java.util.Locale;

@TeleOp(name="LimelightTest", group = "Robot")
public class LimelightTest extends LinearOpMode {

    Limelight3A limelight;
    double currentPipeline;
    double howMuchToExtend;

    public static double higherTyLimit = 10;
    public static double howMuchToDevideDistance = 0.9;
    public static double tuneThisForExtensionLength = 0.0095;
    public static double ARM_INITL = 0.75;
    public static double WRIST_INITL = 0.3;
    public static double tuneThisForExtensionLengthMinus2017 = 0.55;
    public static double tuneThisForExtensionLengthMinus1715 = 0.58;
    public static double tuneThisForExtensionLengthMinus1512 = 0.61;
    public static double tuneThisForExtensionLengthMinus1210 = 0.64;
    public static double tuneThisForExtensionLengthMinus107 = 0.67;
    public static double tuneThisForExtensionLengthMinus75 = 0.7;
    public static double tuneThisForExtensionLengthMinus52 = 0.73;
    public static double tuneThisForExtensionLengthMinus20 = 0.76;
    public static double tuneThisForExtensionLength02 = 0.79;
    public static double tuneThisForExtensionLength25 = 0.82;
    public static double tuneThisForExtensionLength57 = 0.85;
    public static double tuneThisForExtensionLength710 = 0.88;
    public static double tuneThisForExtensionLength1012 = 0.91;
    public static double tuneThisForExtensionLength1215 = 0.94;
    public static double tuneThisForExtensionLength1517 = 0.97;
    public static double tuneThisForExtensionLength1720 = 1;

    public static double EXTENDO_MIN_POSL = 0.45;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        leftExtendoServo = hardwareMap.get(Servo.class, "leftExtendoServo");
//        rightExtendoServo = hardwareMap.get(Servo.class, "rightExtendoServo");
//        armServo = hardwareMap.get(Servo.class, "armServo");
//        wristServo = hardwareMap.get(Servo.class, "wristServo");
//        leftExtendoServo.setPosition(EXTENDO_MIN_POS);
//        rightExtendoServo.setPosition(EXTENDO_MIN_POS);
//        armServo.setPosition(ARM_INIT);
//        wristServo.setPosition(WRIST_INIT_POS);

//        leftExtendoServo.setDirection(Servo.Direction.REVERSE);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                limelight.pipelineSwitch(0); // Pipeline Yellow
                currentPipeline = 0;
            }

            if (gamepad1.dpad_left) {
                limelight.pipelineSwitch(1); // Pipeline Blue
                currentPipeline = 1;
            }

            if (gamepad1.dpad_down) {
                limelight.pipelineSwitch(2); // Pipeline Red
                currentPipeline = 2;
            }

            LLResult result = limelight.getLatestResult();

            if (gamepad1.a) {
                if (result != null) {
                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("Botpose", botpose.toString());
                        telemetry.addData("CurrentPipeline", currentPipeline);

                        if (result.getTy() < -17.5 && result.isValid()){
                            howMuchToExtend = tuneThisForExtensionLengthMinus2017;
                        }
                        else if (result.getTy() < -15 || result.getTy() > -17.5 && result.isValid()){
                            howMuchToExtend = tuneThisForExtensionLengthMinus1715;
                        }

                        else if (result.getTy() < -12.5 || result.getTy() > -15 && result.isValid()){
                            howMuchToExtend = tuneThisForExtensionLengthMinus1512;
                        }
                        else if (result.getTy() < -10 || result.getTy() > -12.5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLengthMinus1210;
                        }
                        else if (result.getTy() < -7.5 || result.getTy() > -10 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLengthMinus107;
                        }
                        else if (result.getTy() < -5 || result.getTy() > -7.5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLengthMinus75;
                        }
                        else if (result.getTy() < -2.5 || result.getTy() > -5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLengthMinus52;
                        }
                        else if (result.getTy() < 0 || result.getTy() > 2.5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLengthMinus20;
                        }
                        else if (result.getTy() > 0 || result.getTy() < 2.5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLength02;
                        }
                        else if (result.getTy() > 2.5 || result.getTy() < 5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLength25;
                        }
                        else if (result.getTy() > 5 || result.getTy() < 7.5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLength57;
                        }
                        else if (result.getTy() > 7.5 || result.getTy() < 10 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLength710;
                        }
                        else if (result.getTy() > 10 || result.getTy() < 12.5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLength1012;
                        }
                        else if (result.getTy() > 12.5 || result.getTy() < 15 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLength1215;
                        }
                        else if (result.getTy() > 15 || result.getTy() < 17.5 && result.isValid()){
                            howMuchToExtend = tuneThisForExtensionLength1517;
                        }
                        else if (result.getTy() > 17.5 && result.isValid()) {
                            howMuchToExtend = tuneThisForExtensionLength1720;
                        }
                    }
                }
            }










            if (gamepad1.b) {
//                leftExtendoServo.setPosition(howMuchToExtend);
//                rightExtendoServo.setPosition(howMuchToExtend);
//                wristServo.setPosition(WRIST_COLLECT_POS);
//                armServo.setPosition(ARM_DROP_SAMPLE_POS);
            }


            if (gamepad1.y) {
//                leftExtendoServo.setPosition(EXTENDO_MIN_POS);
//                rightExtendoServo.setPosition(EXTENDO_MIN_POS);
//                armServo.setPosition(ARM_INITL);
//                wristServo.setPosition(WRIST_INITL);
            }

            telemetry.addData("HowMuchToExtend", howMuchToExtend);
            telemetry.update();
        }
    }

}



