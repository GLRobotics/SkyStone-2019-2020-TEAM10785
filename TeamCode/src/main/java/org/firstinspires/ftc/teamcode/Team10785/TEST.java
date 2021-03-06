//better autonomous code
package org.firstinspires.ftc.teamcode.Team10785;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  DISCLAIMER: WE ARE STILL TESTING THIS. THE DIRECTION (Right or Left) COULD BE WRONG, OR IT COULD NOT WORK AT ALL
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * */

@Autonomous
public class TEST extends LinearOpMode {

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
    public float maxSpeed = .4f;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.setHomePositions();


        telemetry.addData("Status", "Initialized 1");
        telemetry.update();


//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(0);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

//        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setTargetPosition(0);
//        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(1);





        //robot.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("StoredFound", "%s", storedMineralposition);
            telemetry.addData("Lift Position", "%d", robot.lift.getCurrentPosition());

            telemetry.addData("status", "waiting for a start command...");
            telemetry.addData("robot motor position", "%d", robot.leftFrontMotor.getCurrentPosition());
            telemetry.update();
        }

        //waitForStart();




        //commented out TeleOp controls

        while (opModeIsActive()) {


            switch (autonomousStep) {

                case 1://STRAFE UNDER BRIDGE
                    //robot.
                    robot.movePower = 0.4f;
                    robot.leftFrontMotor.setPower(robot.movePower);
                    startTime=robot.period.seconds();
                    autonomousStep=2;
                    break;
                case 2:
                    if(robot.period.seconds()-startTime>3){
                        robot.leftFrontMotor.setPower(0);
                        robot.leftBackMotor.setPower(robot.movePower);
                        startTime=robot.period.seconds();
                        autonomousStep=3;
                    }
                    break;
                case 3:
                    if(robot.period.seconds()-startTime>3){
                        robot.leftBackMotor.setPower(0);
                        robot.rightBackMotor.setPower(robot.movePower);
                        startTime=robot.period.seconds();
                        autonomousStep=4;
                    }
                    break;
                case 4:
                    if(robot.period.seconds()-startTime>3){
                        robot.rightBackMotor.setPower(0);
                        robot.rightFrontMotor.setPower(robot.movePower);
                        startTime=robot.period.seconds();
                        autonomousStep=99;
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

}//}
