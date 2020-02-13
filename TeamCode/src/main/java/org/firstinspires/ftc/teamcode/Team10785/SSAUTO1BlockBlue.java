//better autonomous code
package org.firstinspires.ftc.teamcode.Team10785;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;

import java.util.ArrayList;

@Autonomous
public class SSAUTO1BlockBlue extends LinearOpMode {

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
    public float maxSpeed = 0.5f;
    public float slowSpeed=0.4f;
    public final int slidemax = 461;
    public final int liftmin = -570;
    private VuforiaSkyStone vuforiaSkyStone = new VuforiaSkyStone();
    private TfodSkyStone tfodSkyStone = new TfodSkyStone();
    private final float k = -1.0f;//A constant made for ease of transferral

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.setHomePositions();
        robot.startingAngle=0;


        telemetry.addData("Status", "Initialized 1");
        telemetry.update();


//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

//        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setTargetPosition(0);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(1);


        // Sample TFOD Op Mode
        // Initialize Vuforia.
        vuforiaSkyStone.initialize(
                //*Sharp inhale*
                "AbQaekX/////AAABmdZK4VbDrU0cmz7SaHl/whRyPl7Ef/qgl6dy0r02zyIydswhIKnSJslloshmr7SR0dv9mi1bXIP2WrBUXMANqvWEVuEYCXUwPF2bxWiLRuzmmfJXPzusGPfVqJlYz5DPHoh+GXzErifqDn9pND1e8pxs5hCTdAwSAG4DeyMEhRXTuuLKKusNLKyDwoGjLR7ndnRoi5iQxFKRMnkpnY6XawJvQozMzIxP9NzKe3Sgyzr7Q+yh6AJyPOHEHz1Ftx3jvNb9U+n+l2rUlJxxlAwyAKf5/ugGo/T0BDtozghrwVDi6DTgNLzlbBIC4o8ly/UpF0/rPSt+hmMt0f+OA8yDe194IMDBf2VsAXethn52oh/e",
                //AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                true, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                0, // xAngle
                0, // yAngle
                0, // zAngle
                true); // useCompetitionFieldTargetLocations
        // Set min confidence threshold to 0.7
        tfodSkyStone.initialize(vuforiaSkyStone, 0.5F, true, true);
        // Initialize TFOD before waitForStart.
        // Init TFOD here so the object detection labels are visible
        // in the Camera Stream preview window on the Driver Station.
        tfodSkyStone.activate();


        //robot.lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double runningavg = 0;
        boolean skystonefound = false;
        int skystonenotfoundcount = 0;
        String stoneLoc=null;
        robot.claw.setPosition(0);
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("StoredFound", "%s", storedMineralposition);
            telemetry.addData("Lift Position", "%d", robot.lift.getCurrentPosition());


            telemetry.addData("status", "waiting for a start command...");
            if (tfodSkyStone != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfodSkyStone.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    skystonefound = false;
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equalsIgnoreCase("Skystone")) {
                            //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            runningavg = (recognition.getLeft() * 0.2) + (runningavg * 0.8);
                            skystonefound = true;
                            skystonenotfoundcount = 0;
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
                        }
                    }
                }
            }
            if (!skystonefound) {
                skystonenotfoundcount++;
            }
            if (skystonenotfoundcount > 200) {//The Blue side is a little flakey, keep this at 200
                stoneLoc = "Load";//Right?
            } else if (runningavg > 300) {
                stoneLoc = "Center";//Duh
            } else {
                stoneLoc = "Build";//Left?
            }


            telemetry.addData("Skystone location: ",stoneLoc);
            telemetry.addData("  Running avg: ",  "%f",runningavg);
            telemetry.addData("Skystone not found", "%d", skystonenotfoundcount);
            telemetry.update();
        }
                //waitForStart();

/* TO MOVE SLIDE
        robot.slide.setTargetPosition(461);
        if (Math.abs(robot.slide.getCurrentPosition()-461)<100) {
            autonomousStep = 2; }
        break;
*/
/* TO MOVE FORWARD
robot.movePower = maxSpeed;
                    robot.moveTarget = 36f;
                    robot.move();
                    autonomousStep=3;
                    break;
 */

/* TO STAFE
if (robot.moveStep == 3) {
                        robot.movePower = -0.4f;
                        robot.moveTarget = -24.0f;
                        robot.strafe();
                        autonomousStep = 4; }

 */
/* TO MOVE LIFT
robot.lift.setTargetPosition(-250);
                    while(startTime - robot.period.seconds() > -5) {
                        if (Math.abs(robot.lift.getCurrentPosition() + 250) < 100) {
                            autonomousStep = 6;
                        }
                        break;
                    }
                    break;
 */
/* TO MOVE CLAW
if(robot.claw.getPosition() > .6) {
                        robot.claw.setPosition(.3);
                        autonomousStep = 7;
                    }
 */

                //commented out TeleOp controls

        while (opModeIsActive()) {


            switch (autonomousStep) {
                case 1:// LIFT LEVEL 1/Open claw/Move forward
                    robot.lift.setTargetPosition(481);
                    robot.claw.setPosition(1);
                    robot.movePower = slowSpeed;
                    robot.moveTarget = 20f;
                    robot.moveHoldAngle();
                    autonomousStep = 101;
                    break;

                case 101://STRAFES TO LINE UP WITH CORRECT SKYSTONE
                    if(robot.moveStep==3) {
                        if (stoneLoc.equals("Build")) {
                            robot.movePower = -slowSpeed*k;
                            robot.moveTarget = -6.0f*k;
                            robot.strafeHoldAngle(0);
                        }
                        else if (stoneLoc.equals("Center")) {
                            robot.movePower = slowSpeed*k;
                            robot.moveTarget = 1.5f*k;
                            robot.strafeHoldAngle(0);
                        }
                        else {
                            robot.movePower = slowSpeed*k;
                            robot.moveTarget = 10.5f*k;
                            robot.strafeHoldAngle(0);
                        }
                        autonomousStep = 102;
                    }
                    break;
                case 102://MOVES FORWARD TO 1ST BLOCK
                    if(robot.moveStep==3){
                        robot.movePower = slowSpeed;
                        robot.moveTarget = 10f;
                        robot.moveHoldAngle(0);
                        autonomousStep=2;
                    }
                    break;
                case 2:
                    if (Math.abs(robot.lift.getCurrentPosition() - 481) < 10 && robot.moveStep == 3) {
                        autonomousStep = 4;
                    }
                    break;
                //}


                case 3: // DRIVE FORWARD
                    break;

                case 4: // LIFT TO ZERO
                    robot.lift.setTargetPosition(0);
                    if (Math.abs(robot.lift.getCurrentPosition()) < 10) {
                        autonomousStep = 5;
                    }
                    break;


                case 5: // CLOSE CLAW
                    robot.claw.setPosition(0.4);//old value 0.5
                    robot.slide.setTargetPosition(100);//max is 127
                    autonomousStep = 51;
                    startTime = robot.period.seconds();
                    break;
                case 51://BRINGS LIFT UP TO 60
                    if (robot.period.seconds() - startTime > 1) {
                        robot.lift.setTargetPosition(60);
                        autonomousStep=6;
                    }
                    break;

                case 6: // MOVES BACKWARDS AFTER GRABBING BLOCK
                    if (Math.abs(robot.lift.getCurrentPosition()-60) < 5) {
                        robot.movePower = -maxSpeed;
                        robot.moveTarget = -12f;//was -20
                        robot.moveHoldAngle(0);
                        autonomousStep = 7;
                    }
                    break;
                        /*case 61://
                           if (robot.moveStep==3) {
                               robot.movePower = 0.4f;
                               robot.moveTarget = 0f;
                               robot.turn();
                               autonomousStep=7;
                           }
                            break;*/

                case 7: // STRAFE UNDER 14 INCH BAR TO SAME POSITION
                    if (robot.moveStep == 3) {
                        robot.movePower = -maxSpeed*k;
                        if (stoneLoc.equals("Build")) {
                            robot.moveTarget = (-56.0f+6.0f-30.0f)*k;//Extra 30 is for tray
                            robot.strafeHoldAngle(0);
                        } else if (stoneLoc.equals("Center")) {
                            robot.moveTarget =(-56.0f-2.0f-30.0f)*k;
                            robot.strafeHoldAngle(0);
                        } else {
                            robot.moveTarget = (-56.0f-10.5f-30.0f)*k;
                            robot.strafeHoldAngle(0);
                        }
                        autonomousStep = 9;
                    }
                    break;

                        /*case 8: // MOVE SLIDE OUT
                            if (robot.moveStep == 3) {
                                robot.slide.setTargetPosition(127);
                                /*robot.movePower=0.4f;
                                robot.moveTarget=0f;
                                robot.turn();
                                autonomousStep = 9;
                            }
                            break;
                    */
                case 9:// OPEN CLAW/RELEASE BLOCK
                    if  (robot.moveComplete()) {
                        robot.claw.setPosition(1);
                        startTime=robot.period.seconds();
                        autonomousStep = 10;
                    }
                    break;

                case 10: // STRAFE BACK TO SECOND BLOCK AND LINES UP
                    robot.lift.setTargetPosition(0);
                    robot.movePower=maxSpeed*k;
                    if (stoneLoc.equals("Build")) {
                        robot.moveTarget = (80.0f+30.0f)*k;//Extra 30 is for tray
                        autonomousStep=11;
                    } else if (stoneLoc.equals("Center")) {
                        robot.moveTarget = (87.0f+30.0f)*k;
                        autonomousStep=11;
                    } else {
                        robot.moveTarget = (85.0f+30.0f)*k;
                        autonomousStep=111;
                    }
                    //robot.moveTarget=15.0f;THIS WAS MOVE TO CENTERLINE
                    robot.strafeHoldAngle(0);
                    startTime=robot.period.seconds();
                    break;
                case 111://BUILD 2ND BLOCK
                    if (robot.period.seconds()-startTime>3.0) {
                        robot.claw.setPosition(1);
                        robot.slide.setTargetPosition(75);
                    }
                    if (robot.moveComplete()) {
                        robot.movePower=slowSpeed;
                        robot.moveTarget=8.0f;
                        robot.moveHoldAngle(0);
                        autonomousStep = 222;
                    }
                    break;
                case 11: // BUILD/CENTER MOVE TO BLOCK
                    if (robot.period.seconds()-startTime>3.0) {
                        robot.lift.setTargetPosition(481);
                        robot.claw.setPosition(1);
                        robot.slide.setTargetPosition(0);
                    }
                    if (robot.moveComplete()) {
                        robot.movePower=slowSpeed;
                        robot.moveTarget=10f;
                        robot.moveHoldAngle(0);
                        autonomousStep = 12;
                    }
                    break;

                case 222: // STRAFE TO LOAD BLOCK
                    if (robot.moveComplete()){
                        robot.movePower= slowSpeed*k;
                        robot.moveTarget= 6.0f*k;
                        robot.strafeHoldAngle(0);
                        autonomousStep = 13;
                    }
                    break;

                case 12: // Reset lift to ZERO
                    if ((Math.abs(robot.lift.getCurrentPosition()-481) < 10)&&robot.moveComplete()) {
                        robot.lift.setTargetPosition(0);
                        autonomousStep=13;
                    }
                    break;
                case 13: // CHECKS IF LIFT IS DOWN/CLAW CLOSES/SLIDE OUT
                    if (robot.lift.getCurrentPosition()<10&&robot.moveComplete()){
                        robot.claw.setPosition(0.4);//old value 0.5
                        robot.slide.setTargetPosition(100);//max is 127
                        autonomousStep = 14;
                        startTime = robot.period.seconds();
                    }
                    break;
                case 14: // MOVE BACKWARDS SECOND TIME
                    if (robot.period.seconds()-startTime>1){
                        robot.lift.setTargetPosition(60);
                        robot.movePower=-maxSpeed;
                        if (stoneLoc.equals("Build")){
                            robot.moveTarget = -9f;
                        }
                        else {
                            robot.moveTarget = -10f;
                        }
                        robot.moveHoldAngle(0);
                        autonomousStep=15;
                    }
                    break;
                case 15://STRAFE UNDER 14 INCH BAR PT 2
                    if(robot.moveComplete()) {
                        robot.movePower = -maxSpeed*k;
                        if (stoneLoc.equals("Build")) {
                            robot.moveTarget = (-81.0f-30.0f)*k;
                            autonomousStep = 16;
                        } else if (stoneLoc.equals("Center")) {
                            robot.moveTarget = (-90.0f-30.0f)*k;
                            autonomousStep = 16;
                        }
                        else {
                            robot.moveTarget = (-101.0f-30.0f)*k;
                            autonomousStep = 16;
                        }
                        robot.strafeHoldAngle(0);
                    }
                    break;
                case 16://OPEN CLAW/STRAFE TO PARK AT CENTERLINE
                    if (robot.moveComplete()) {
                        robot.claw.setPosition(1);
                        robot.moveTarget=(23.0f+30.0f)*k;
                        robot.movePower=maxSpeed*k;
                        robot.strafeHoldAngle(0);
                        startTime=robot.period.seconds();
                        autonomousStep=17;
                    }
                    break;
                case 17://CLOSE CLAW
                    if (robot.period.seconds()-startTime>1.0){
                        robot.claw.setPosition(0);
                        autonomousStep=999;
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
