package org.firstinspires.ftc.teamcode.Team10785;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

@Config
@TeleOp
public class Team10785TeleOp extends LinearOpMode {

    Robot10785 robot = new Robot10785();

    public Vision visionSystem;
    public String found;
    public int target;
    public int slidetarget;
    public int clawtarget;
    public int liftCount;
    public int startingLiftPos=0;
    private double startTimeTele = 0;
    public final int slideout = 411;
    public final int slidein = 0;
    public boolean awaspressed = false;
    private float initRobotAngle;
    boolean xwaspressed = false;
    public float savedRobotAngle;
    public final int liftceil = 4250;
    public final int liftfloor = -466;
    public float tape1Target =0;
    public float tape2Target =0;

    private float totalCumulative, cumulativeError,correctionFactor;
    private FtcDashboard dashboard;
    private PIDFController turnPIDController;
    private PIDCoefficients turnPIDCoefficients=new PIDCoefficients();
    public double kP=0.03;
    public double kI=0.0018;
    public double kD=0.003;
    private String catName;
    private CustomVariable catVar;
    private static final String PID_VAR_NAME = "TeleOp Turn PID";
    private boolean driveModePID=true;

    private void addPidVariable() {
        catName = getClass().getSimpleName();
        catVar = (CustomVariable) dashboard.getConfigRoot().getVariable(catName);
        if (catVar == null) {
            // this should never happen...
            catVar = new CustomVariable();
            dashboard.getConfigRoot().putVariable(catName, catVar);

            RobotLog.w("Unable to find top-level category %s", catName);
        }

        CustomVariable pidVar = new CustomVariable();
        pidVar.putVariable("kP", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return kP;
            }

            @Override
            public void set(Double value) {
                kP=value;
                turnPIDCoefficients.kP=kP;
                turnPIDCoefficients.kI=kI;
                turnPIDCoefficients.kD=kD;
                turnPIDController=new PIDFController(turnPIDCoefficients,0,0,0);

            }
        }));
        pidVar.putVariable("kI", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return kI;
            }

            @Override
            public void set(Double value) {
                kI=value;
                turnPIDCoefficients.kP=kP;
                turnPIDCoefficients.kI=kI;
                turnPIDCoefficients.kD=kD;
                turnPIDController=new PIDFController(turnPIDCoefficients,0,0,0);

            }
        }));
        pidVar.putVariable("kD", new BasicVariable<>(new ValueProvider<Double>() {
            @Override
            public Double get() {
                return kD;
            }

            @Override
            public void set(Double value) {
                kD=value;
                turnPIDCoefficients.kP=kP;
                turnPIDCoefficients.kI=kI;
                turnPIDCoefficients.kD=kD;
                turnPIDController=new PIDFController(turnPIDCoefficients,0,0,0);

            }
        }));

        catVar.putVariable(PID_VAR_NAME, pidVar);
        dashboard.updateConfig();
    }

    private void removePidVariable() {
        if (catVar.size() > 1) {
            catVar.removeVariable(PID_VAR_NAME);
        } else {
            dashboard.getConfigRoot().removeVariable(catName);
        }
        dashboard.updateConfig();
    }

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        turnPIDCoefficients.kP=kP;
        turnPIDCoefficients.kI=kI;
        turnPIDCoefficients.kD=kD;
        turnPIDController=new PIDFController(turnPIDCoefficients,0,0,0);
        turnPIDController.setOutputBounds(-1,1);
        telemetry.addData("Stauts","Initialized 1");
        telemetry.update();
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        addPidVariable();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Lift Position", "%d ", robot.lift.getCurrentPosition());
            telemetry.addData("Slide", "%d ", robot.slide.getCurrentPosition());
            telemetry.addData("Rotation", "%f ", robot.getRobotAngle());
            telemetry.update();
        }

        startingLiftPos=robot.lift.getCurrentPosition();
        target=startingLiftPos;
//        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(0);
//        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

//        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setTargetPosition(0);
//        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(1);

        target = startingLiftPos;
        liftCount = 0;
        savedRobotAngle=robot.getRobotAngle();

        robot.tape1.setTargetPosition(0);
        robot.tape2.setTargetPosition(0);
        robot.tape1.setPower(1);
        robot.tape2.setPower(1);

        while(opModeIsActive()) {
            // Convert joysticks to desired motion.
            if (gamepad1.dpad_left){
                tape1Target+=2;
                if (tape1Target>120)
                    tape1Target=120;
            }
            else if (gamepad1.dpad_right) {
                tape1Target -= 2;
                if (tape1Target<-10)
                    tape1Target=-10;
            }
            robot.tape1.setTargetPosition((int) (tape1Target*robot.countsPerInchTape) );


            if (gamepad1.b){
                tape2Target+=2;
                if (tape2Target>120)
                    tape2Target=120;
            }
            else if (gamepad1.x) {
                tape2Target -= 2;
                if (tape2Target<-10)
                    tape2Target=-10;
            }
            robot.tape2.setTargetPosition((int) (tape2Target*robot.countsPerInchTape) );


            double robotRad = robot.getRobotAngleRad()+robot.startingAngle;

            double x = ((gamepad1.left_stick_x * Math.cos(robotRad))-(gamepad1.left_stick_y * Math.sin(robotRad)));
            double y = ((gamepad1.left_stick_y * Math.cos(robotRad))+(gamepad1.left_stick_x * Math.sin(robotRad)));

            if (Math.abs(y)>Math.abs(x)){
                driveModePID=false;
            }
            else {
                driveModePID=true;
                savedRobotAngle=robot.getRobotAngle();
            }
            // POV Control
            Mecanum.Motion motion;
            if (Math.abs(gamepad1.right_stick_x)>0.05){
                motion = Mecanum.joystickToMotion(x, y, gamepad1.right_stick_x, gamepad1.right_stick_y);
                savedRobotAngle=robot.getRobotAngle();
                correctionFactor=0;
                cumulativeError=0;
                totalCumulative=0;
            }
            else if ((Math.abs(gamepad1.left_stick_x)>0.05)||Math.abs(gamepad1.left_stick_y)>0.05){
                turnPIDController.setTargetPosition(savedRobotAngle);
                /*correctionFactor=-0.023f*(savedRobotAngle-robot.getRobotAngle());
                cumulativeError+=(0.010f)*correctionFactor;
                totalCumulative=cumulativeError+correctionFactor;
                */
                if (driveModePID) {
                    motion = Mecanum.joystickToMotion(x, y, -turnPIDController.update(robot.getRobotAngle()), 0);
                }
                else{
                    motion = Mecanum.joystickToMotion(x, y,0, 0);
                }
            }
            else{
                motion= Mecanum.joystickToMotion(0,0,0,0);
            }
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Desired Angle: ", savedRobotAngle);
            packet.put("Actual Angle: ", robot.getRobotAngle());

            dashboard.sendTelemetryPacket(packet);

            /*
            // Non Adjusted Motion
            Mecanum.Motion motion = Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y);
*/


            // Convert desired motion to wheel powers, with power clamping.

            motion.vTheta = motion.vTheta * 0.8;
            Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
            wheels = Mecanum.Wheels.scaleWheelPower();
           // wheels=Mecanum.Wheels.scaleWheelPowerTwo();
            robot.setWheelSpeeds(wheels.frontLeft, wheels.backLeft, wheels.frontRight, wheels.backRight);


            /*/
             * The paramater "target" is the target that we want to move the lift to
             * When we hold down the button, we increase the target by 1, but also tell the robot to
             * move to the position, creating motion along the lift. This is more consistent than
             * robot.lift.setpower(int power); because it creates continuous motion along the lift.
             * Using robot.lift.setpower(int power); causes the lift to move in multiple tiny
             * increments opposed to one big one, which takes a LONG time compared to using the
             * "target" way of doing this.
            /*/

            //lift
            {
                if(gamepad2.left_stick_y < -0.2) {
                    target += 10;
                } else if (gamepad2.left_stick_y > 0.2) {
                    target -= 10;

                }

                if (target > liftceil) {
                    target = liftceil;
                }
                if (target < liftfloor) {
                    target = liftfloor;
                }


                robot.lift.setTargetPosition(target);

            }//end lift

            //slide
            {
                if (gamepad2.dpad_left) {
                    slidetarget = 127;
                }

                if (gamepad2.dpad_right) {
                    slidetarget = 0;
                }

                /*if (gamepad2.left_stick_x > 0.2) {
                    slidetarget += 1;
                } else if (gamepad2.left_stick_x < -0.2) {
                    slidetarget -= 1;

                }*/

                if (slidetarget > slideout) {
                    slidetarget = slideout; //is zero out or in?
                }
                if (slidetarget < slidein) {
                    slidetarget = slidein;
                }
                robot.slide.setTargetPosition(slidetarget);
            }//end slide

            //claw
            {
                if (gamepad2.dpad_up) {//UP = OPEN
                    robot.claw.setPosition(1);
                }
                if (gamepad2.dpad_down) {//DOWN = CLOSED
                    robot.claw.setPosition(0.4);
                    slidetarget = 127;
                }
            }//end claw

            //buttons
            {
                if (gamepad2.x) {//in then down
                    xwaspressed = true;
                }
                    if(xwaspressed){
                    if(robot.slide.getCurrentPosition()<=20) {
                        target = 0; // lvl 1
                    }
                    if(robot.lift.getCurrentPosition()>=20) {
                        slidetarget = 0;
                    }
                    if(robot.lift.getCurrentPosition() <=20 && robot.slide.getCurrentPosition() <=20){
                        xwaspressed = false;
                    }
                    }

                    if (gamepad2.left_trigger > .5) {
                        target = 60;
                    }
                if (gamepad2.a) {
                    awaspressed = true;
                    target = 281; //lvl 2
                }
                    if(awaspressed){

                    }
                if (gamepad2.b) {
                    target = 803; // lvl 3

                }
                if (gamepad2.y) {
                    target = 1284;//level 4

                }
                if (gamepad2.right_stick_y > .9) {
                    target = 1702; //zero = out
                    slidetarget = 127;
                }

                if (slidetarget > slideout) {
                    slidetarget = slideout;
                }
                if (slidetarget < slidein) {
                    slidetarget = slidein;
                }


                robot.slide.setTargetPosition(slidetarget);
            }//end buttons

            //pin release
            {
                if (gamepad2.right_trigger > .5) {
                    robot.pin.setPosition(0);
                }
                if (gamepad2.left_trigger > .5) {
                    robot.pin.setPosition(1);
                }
            }// end of pin release

            { //hook

                if (gamepad1.right_trigger > .5) {
                    robot.hook.setPosition(.3);  // close hook 1
                    robot.hook2.setPosition(0); } // close hook 2

                if (gamepad1.left_trigger > .5) {
                    robot.hook.setPosition(0);  //open hook 1
                    robot.hook2.setPosition(.3); } //open hook  2

            } // end of hook



            robot.lift.setTargetPosition(target);


            telemetry.addData("Status","Running");
            telemetry.addData("MoveStep","%d",robot.moveStep);
            telemetry.addData("Starting Arm Postition", "%d", startingLiftPos);
            telemetry.addData("Found","%s",found);
            telemetry.addData("Velocity: ","%f",robot.getVelocity());
            telemetry.addData("Velocity Error: ","%f",robot.getVelocityError());
            telemetry.addData("Left Joystick Command: ","%f",wheels.frontLeft);
            telemetry.addData("Left Jostick Value: ","%f",gamepad1.left_stick_y);
            telemetry.addData("Lift Position", "%d ", robot.lift.getCurrentPosition());
            telemetry.addData("Slide", "%d ", robot.slide.getCurrentPosition());
            telemetry.addData("robot motor position", "%d", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("MCMD", "%.1f %.1f", motion.vD, motion.vTheta);
            telemetry.addData("Rotation", "%f ", robot.getRobotAngle());
            telemetry.addData("PDIF Stuff:", "%s",robot.slide.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION).toString());
            telemetry.addData("Front Left Power", "%f ", wheels.frontLeft);
            telemetry.addData("Front Right Power", "%f",wheels.frontRight);
            telemetry.addData("Back Right Power", "%f",wheels.backRight);
            telemetry.addData("Back Left Power", "%f",wheels.backLeft);



            telemetry.update();

        }//end while(OpmodeIsActive);
        removePidVariable();
    }//end public void runopmode();

}//end class
