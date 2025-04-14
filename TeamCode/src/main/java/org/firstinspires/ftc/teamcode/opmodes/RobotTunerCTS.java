package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Libs.RRMechOps;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

@Config
@TeleOp(name="CTS", group="Robot")
public class RobotTunerCTS extends LinearOpMode {


    private final static HWProfile robot = new HWProfile();
    private final LinearOpMode opMode = this;
    private final RRMechOps mechOps = new RRMechOps(robot,opMode);

    FtcDashboard dashboard;


//    ServoImplEx extForeLeftServo = null;



    @Override
    public void runOpMode() {



        robot.init(hardwareMap, true);
//        extForeLeftServo = hardwareMap.get(ServoImplEx.class, "extForeLeftServo");

        Motor motorLiftBack = new Motor(hardwareMap, "motorLiftR", Motor.GoBILDA.RPM_435 );
        Motor motorLiftFront = new Motor(hardwareMap, "motorLiftF", Motor.GoBILDA.RPM_435);
        Motor motorLiftTop = new Motor(hardwareMap, "motorLiftT", Motor.GoBILDA.RPM_435);

        motorLiftTop.resetEncoder();
        motorLiftFront.resetEncoder();
        motorLiftBack.resetEncoder();

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            telemetry.addData("motor Lift Front Position", motorLiftFront.getCurrentPosition());
            telemetry.addData("motor Lift Back Position", motorLiftBack.getCurrentPosition());
            telemetry.addData("motor Lift Top Position", motorLiftTop.getCurrentPosition());
            telemetry.update();
        }
    }


}