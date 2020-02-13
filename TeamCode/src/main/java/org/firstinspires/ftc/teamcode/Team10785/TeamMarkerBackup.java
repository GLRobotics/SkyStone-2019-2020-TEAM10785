package org.firstinspires.ftc.teamcode.Team10785;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.KeyStore;
import java.util.TimerTask;
@Disabled
@Autonomous
public class TeamMarkerBackup extends LinearOpMode {

    Robot10785 robot = new Robot10785();

    public Vision visionSystem;
    public String found;
    public int target;
    public int autonomousStep = 1;
    public String storedMineralposition;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = robot.period.seconds();
    public int diagnalLength = 34;
    public int LaW = 24;
    public double startTime = 0;
    public float maxSpeed = 1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        visionSystem = new Vision();

        visionSystem.initVuforia(hardwareMap);
        visionSystem.initTfod(hardwareMap);


        telemetry.addData("Stauts", "Initialized 1");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for a start command...");
            telemetry.update();
        }

        //waitForStart();
        visionSystem.activateTFOD();


        //robot.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        target = 0;

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //commented out TeleOp controls

        while (opModeIsActive()) {


            //robot.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.arm1.setTargetPosition(target);
            //robot.arm2.setTargetPosition(target);
            //robot.arm1.setPower(.50);
            //robot.arm2.setPower(.50);

            visionSystem.updateMineralPostion();

            switch (autonomousStep) {
                case 1:
                    if (robot.moveStep == 3 || robot.moveStep == 0) {
                        robot.movePower = maxSpeed * 0.2f;
                        robot.moveTarget = 4.0f;
                        robot.moveStep = 6;
                        autonomousStep = 2;
                    }break;
            }

            robot.moveUpdate();
            telemetry.addData("Status", "Running");
            telemetry.addData("MoveStep", "%d", robot.moveStep);
            telemetry.addData("Found", "%s", found);
            telemetry.addData("StoredFound", "%s", storedMineralposition);
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    visionSystem.getRobotX(), visionSystem.getRobotY(), visionSystem.getRobotZ());
            //telemetry.addData("armPosition", "%d ", robot.arm1.getCurrentPosition());
            telemetry.addData("robot motor position", "%d", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("AutonomousStep", "%d", autonomousStep);
            telemetry.addData("Lift Position", "%d", robot.lift.getCurrentPosition());

            telemetry.update();

        }
        visionSystem.deactivateTFOD();
    }

}//}
