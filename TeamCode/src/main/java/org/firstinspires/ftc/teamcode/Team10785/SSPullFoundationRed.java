//better autonomous code
package org.firstinspires.ftc.teamcode.Team10785;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.security.KeyStore;
import java.util.TimerTask;

@Autonomous
public class SSPullFoundationRed extends LinearOpMode {

    Robot10785 robot = new Robot10785();

    public String found;
    public int target;
    public int autonomousStep = 1;
    public String storedMineralposition;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = robot.period.seconds();
    public int diagnalLength = 34;
    public int LaW = 24;
    public double startTime = 0;
    public float maxSpeed = .8f;
    public float slowSpeed=0.4f;
    public final int slidemax = 461;
    public final int liftmin = -570;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.setHomePositions();
        robot.startingAngle = Math.PI/2.0f;
        robot.claw.setPosition(0);


        telemetry.addData("Stauts", "Initialized 1");
        telemetry.update();


        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

//
        robot.slide.setTargetPosition(0);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(1);


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("StoredFound", "%s", storedMineralposition);
            telemetry.addData("Lift Position", "%d", robot.lift.getCurrentPosition());

            telemetry.addData("status", "waiting for a start command...");
            telemetry.update();
        }


        while (opModeIsActive()) {


            switch (autonomousStep) {
                case 1: //STRAFE TO LINE UP WITH SIDE OF TRAY
                    robot.movePower = -slowSpeed;//WAS 0.4
                    robot.moveTarget = -54f;
                    robot.strafeHoldAngle(-2.0f);
                    autonomousStep = 102;
                    break;

                case 102://BACKS UP INTO TRAY
                    if (robot.moveComplete()) {
                        robot.hook.setPosition(0.3);
                        robot.hook2.setPosition(0);
                        robot.movePower = -slowSpeed;
                        robot.moveTarget = -10f;
                        robot.moveHoldAngle();
                        autonomousStep = 101;
                    }
                    break;
                case 101://HOOKS DOWN
                    if (robot.moveComplete()) {
                        robot.hook.setPosition(0);
                        robot.hook2.setPosition(0.3);
                        startTime=robot.period.seconds();
                        autonomousStep = 1999;

                    }
                    break;
                case 1999://MOVE FORWARD TOWARDS POST
                    if(robot.period.seconds() - startTime > 1.0){
                        robot.movePower=slowSpeed;//WAS 0.5
                        robot.moveTarget=9f;
                        robot.moveHoldAngle();
                        autonomousStep=2;
                    }
                    break;
                case 2: //ROTATES ROBOT 57 DEGREES
                    if (robot.moveComplete()) {
                        robot.movePower = slowSpeed;
                        robot.moveTarget = -57f;//WAS 0.5
                        robot.turn();
                        autonomousStep = 3;
                    }
                    break;
                case 3: //MOVES BACK INTO BUILD AREA
                    if (robot.moveComplete()) {
                        robot.movePower=-slowSpeed;//WAS 0.6
                        robot.moveTarget=-28.5f;
                        robot.moveHoldAngle();
                        autonomousStep = 33;
                    }
                    break;
                case 33://HOOKS UP
                    if (robot.moveComplete()) {
                        robot.hook.setPosition(0.3);
                        robot.hook2.setPosition(0);
                        startTime = robot.period.seconds();
                        autonomousStep=4;
                    }
                    break;
                case 4: //TURN ON POINT TO CENTERLINE
                    if (robot.period.seconds() - startTime > 1.0) {
                        robot.movePower = slowSpeed;//WAS 0.6
                        robot.moveTarget = 20f;
                        robot.turnOnPoint();
                        autonomousStep = 44;
                    }
                    break;
                case 44://MOVE TO CENTERLINE
                    if (robot.moveComplete()){
                        robot.movePower=maxSpeed;//WAS 0.7
                        robot.moveTarget=23f;
                        robot.moveHoldAngle();
                        autonomousStep=999999;
                    }
                    break;
            }
            robot.moveUpdate();
            telemetry.addData("Status", "Running");
            telemetry.addData("MoveStep", "%d", robot.moveStep);
            telemetry.addData("Found", "%s", found);
            telemetry.addData("StoredFound", "%s", storedMineralposition);
            //telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            telemetry.addData("armPosition", "%d ", robot.lift.getCurrentPosition());
            telemetry.addData("robot motor position", "%d", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("AutonomousStep", "%d", autonomousStep);
            telemetry.addData("Lift Position", "%d", robot.lift.getCurrentPosition());
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.update();

        }
    }
}