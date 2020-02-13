package org.firstinspires.ftc.teamcode.Team10785;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class VisionTest extends LinearOpMode {


    public Vision visionSystem;
    public String found;


    @Override
    public void runOpMode(){

        visionSystem = new Vision();

        visionSystem.initVuforia(hardwareMap);
        visionSystem.initTfod(hardwareMap);


        telemetry.addData("Stauts","Initialized");
        telemetry.update();

        waitForStart();
        visionSystem.activateTFOD();





        while (opModeIsActive()){

            if (visionSystem.updatePosition())
                found="Target Visible";
            else
                found="Target Not Found";

            visionSystem.updateMineralPostion();



            telemetry.addData("Status","Running");
            telemetry.addData("Found","%s",found);
            telemetry.addData("Visible Target", visionSystem.getmineralPostion());
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    visionSystem.getRobotX(), visionSystem.getRobotY(), visionSystem.getRobotZ());


            telemetry.update();
        }


    }
}
