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
    public class MoveProfileTest extends LinearOpMode {

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
        public float maxSpeed = 0.8f;
        public float slowSpeed=0.4f;
        public final int slidemax = 461;
        public final int liftmin = -570;
        private VuforiaSkyStone vuforiaSkyStone = new VuforiaSkyStone();
        private TfodSkyStone tfodSkyStone = new TfodSkyStone();
        private final float k = 1.0f;
        @Override
        public void runOpMode() {
            robot.init(hardwareMap);
            robot.setHomePositions();
            robot.startingAngle=0;


            telemetry.addData("Stauts", "Initialized 1");
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
                    "AbQaekX/////AAABmdZK4VbDrU0cmz7SaHl/whRyPl7Ef/qgl6dy0r02zyIydswhIKnSJslloshmr7SR0dv9mi1bXIP2WrBUXMANqvWEVuEYCXUwPF2bxWiLRuzmmfJXPzusGPfVqJlYz5DPHoh+GXzErifqDn9pND1e8pxs5hCTdAwSAG4DeyMEhRXTuuLKKusNLKyDwoGjLR7ndnRoi5iQxFKRMnkpnY6XawJvQozMzIxP9NzKe3Sgyzr7Q+yh6AJyPOHEHz1Ftx3jvNb9U+n+l2rUlJxxlAwyAKf5/ugGo/T0BDtozghrwVDi6DTgNLzlbBIC4o8ly/UpF0/rPSt+hmMt0f+OA8yDe194IMDBf2VsAXethn52oh/e",
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
                            if(recognition.getLabel().equalsIgnoreCase("Skystone")) {
                                //                          telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                if (recognition.getLeft()>50) {
                                    runningavg = (recognition.getLeft() * 0.2) + (runningavg * 0.8);
                                    skystonefound = true;
                                    skystonenotfoundcount = 0;
                                }
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
                            }
                        }

                    }
                }
                //CHECKS RUNNING AVERAGE AND SETS STONE LOCATION TO A STRING
                if(!skystonefound) {
                    skystonenotfoundcount ++;
                }
                if (skystonenotfoundcount>100){
                    stoneLoc="Build";
                }
                else if (runningavg>421){
                    stoneLoc="Build";//Right?
                }
                else if(runningavg<255){
                    stoneLoc="Load";//Left?
                }
                else{
                    stoneLoc="Center";//Duh
                }

                telemetry.addData("Skystone location: ",stoneLoc);
                telemetry.addData("  Running avg: ",  "%f",runningavg);
                telemetry.addData("Skystone not found", "%d", skystonenotfoundcount);
                telemetry.update();
            }
            while (opModeIsActive()) {


                switch (autonomousStep) {
                    case 1:
                        robot.moveProfileStrafe(1.5f, 1.0f, 71.0f);
                        autonomousStep = 2;
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

