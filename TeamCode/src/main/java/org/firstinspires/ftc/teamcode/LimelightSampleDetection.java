package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Test", group = "sensor")
public class LimelightSampleDetection extends LinearOpMode {

    double limelightMountAngleDegrees = -30;

    double limelightHeightInches = 12.17;

    double YAngle, XAngle;

    double limelightXOffset = 5.2;

    double limelightYOffset = 2.33;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();

        LLResult result = limelight.getLatestResult();

        telemetry.addData(">", "Robot Ready. Press Play");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //Get x and y angles
            if (result != null && result.isValid()) {
                XAngle = result.getTx();
                YAngle = result.getTy();

                telemetry.addData("Target X", XAngle);
                telemetry.addData("Target Y", YAngle);
            } else {
                telemetry.addData("limelight", "No Targets");
            }

            //Calculate angle to target from horizontal
            double angleToTargetDegrees = limelightMountAngleDegrees + YAngle;
            double angleToTargetRadians = angleToTargetDegrees * (3.14159/180);

            //Calculate distance
            double distanceToGoal = (limelightHeightInches / Math.tan(angleToTargetRadians)) + limelightYOffset;
            telemetry.addData("Distance to taget", distanceToGoal);

            //Calculate Side to Side
            double XOffset = (Math.tan(XAngle) * distanceToGoal) - limelightXOffset;
            telemetry.addData("Displacement to the right", XOffset);

            telemetry.update();
        }
   }

}
