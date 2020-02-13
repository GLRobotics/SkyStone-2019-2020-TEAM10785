package org.firstinspires.ftc.teamcode.Team10785;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.sun.tools.javac.tree.DCTree;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

public class Robot10785 {
    //public CRServo collector2;
    //public CRServo extend2;
    //public CRServo extend1;
    //public DcMotor lift;
    //public DcMotor lift2;
    public Servo claw; //stacker part: Holds the blocks
    public Servo pin;
    public Servo hook;
    public Servo hook2;
    public DcMotorEx lift; //stacker part: Raises the claw
    public DcMotorEx slide; //stacker part: Moves the lift out
    public DcMotorEx tape1;
    public DcMotorEx tape2;
    public double countsPerInchTape = 288/13.2;
    //public Servo teammarker;
    public DcMotorEx leftFrontMotor; //Wheel 1
    public DcMotorEx rightFrontMotor; //Wheel 2
    public DcMotorEx leftBackMotor; //Wheel 3
    public DcMotorEx rightBackMotor; //Wheel 4
    public int moveStep = 0;
    public float movePower = 0;
    public float moveTarget = 0;
    public float countPerInch = 45.0f;//WAS 172
    public float inchesPer360 = 79.0116f;
    public float initRobotAngle;
    public float cumulativeError;
    public float totalCumulative;
    public float correctionFactor;
    public PIDFCoefficients veloPIDF=new PIDFCoefficients();
    public PIDFCoefficients slidePID=new PIDFCoefficients();
    public PIDFCoefficients liftPIDF=new PIDFCoefficients();
    private FtcDashboard dashboard;
    private float accelTime;
    private float vMax;
    private float distance;
    private float startTime;
    private float t;
    private float t2;


    public static BNO055IMU imu;
    public static double startingAngle=0;
    public double runningavg;

    public Orientation angles;
    //public Acceleration gravity;


    /* local OpMode members. */
    HardwareMap hwMap =  null;
    public ElapsedTime period = new ElapsedTime();


    public void Robot10785() {
    }

    //Chances are this is all you're here for:\\
    public double getVelocity(){
        return leftBackMotor.getVelocity();
        //Should we return the average velocity from all four?
    }
    public double getVelocityError(){
        return leftBackMotor.getVelocity()-(leftBackMotor.getPower()*3100);
        //Should we return the average velocity from all four?
    }

    public void turn() {
        moveStep = 10;
    }
    public void turn(float degrees) {
        moveTarget = degrees;
        moveStep = 10;
    }
    public void move(){
        moveStep = 1;
    }
    public void move(float dist){
        moveTarget = dist;
        moveStep = 1;
    }
    public void moveProfile(float aTime, float v, float d){
        accelTime = aTime;
        vMax = v;
        distance = d;
        startTime = (float) period.seconds();
        t2 = (distance/(2400/45) -(accelTime*vMax))/vMax;
        moveStep = 40;
    }
    public void moveHoldAngle(){
        initRobotAngle=getRobotAngle();
        moveStep=101;
    }
    public void moveHoldAngle(float angle1){
        initRobotAngle=angle1;
        moveStep=101;
    }
    public void strafe(){
        moveStep = 6;
    }
    public void strafeHoldAngle(){
        moveStep=66;
        initRobotAngle=getRobotAngle();
    }
    public void strafeHoldAngle(float angle1){
        moveStep=66;
        initRobotAngle=angle1;
    }
    public void moveProfileStrafe(float aTime, float v, float d){
        accelTime = aTime;
        vMax = v;
        distance = d;
        startTime = (float) period.seconds();
        t2 = (distance/(1800/45) -(accelTime*vMax))/vMax;
        moveStep = 41;
    }
    public void strafeToSkystone(){
        moveStep=14;
    }

    public void turnOnPoint(){
        moveStep=13;
    }
    public boolean moveComplete(){
        if((moveStep==3)||(moveStep==0)){
            return true;
        }
        else return false;
    }
    //end of all you're here for

    public void setrobotradius(double rad){
        inchesPer360 = (float)(rad * 2 * Math.PI);
    }
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //EVERY SERVO OR MOTOR MUST HAVE AN ENTRY HERE:
        {
            //Expansion hub 1
            {
                lift = hwMap.get(DcMotorEx.class, "lift"); //motor 0
                slide = hwMap.get(DcMotorEx.class, "slide"); //motor 1
                tape1 = hwMap.get(DcMotorEx.class, "tape1");
                tape2 = hwMap.get(DcMotorEx.class, "tape2");

                claw = hwMap.get(Servo.class, "claw"); //servo
                hook = hwMap.get(Servo.class, "hook"); // hook for foundation
                hook2 = hwMap.get(Servo.class, "hook2"); // hook2 for foundation
                pin = hwMap.get(Servo.class, "pin"); // pin servo 0
            }//ends expansion hub 1

            //expansion hub 2
            {
                //teammarker = hwMap.get(Servo.class, "teammarker"); //
                leftFrontMotor = hwMap.get(DcMotorEx.class, "lfWheel");
                leftBackMotor = hwMap.get(DcMotorEx.class, "lrWheel");
                rightFrontMotor = hwMap.get(DcMotorEx.class, "rfWheel");
                rightBackMotor = hwMap.get(DcMotorEx.class, "rrWheel");
            }//ends expansion hub 2
        }//ends outer grouping
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setTargetPosition(slide.getCurrentPosition());
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tape1.setDirection(DcMotor.Direction.REVERSE);
        tape2.setDirection(DcMotor.Direction.FORWARD);
        tape1.setTargetPosition(slide.getCurrentPosition());
        tape1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tape1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tape2.setTargetPosition(slide.getCurrentPosition());
        tape2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tape2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Must Stop and Reset before using Run With Encoder
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        veloPIDF.p=5;//WAS 10
        veloPIDF.d=0;
        veloPIDF.i=3;//WAS 6
        veloPIDF.f=2;
        veloPIDF.algorithm=MotorControlAlgorithm.PIDF;
        leftFrontMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,veloPIDF);
        leftBackMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,veloPIDF);
        rightBackMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,veloPIDF);
        rightFrontMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,veloPIDF);

        slidePID.p=15;//was 5
        slidePID.i=0;
        slidePID.d=0;
        slidePID.f=0;
        slidePID.algorithm=MotorControlAlgorithm.PIDF;
        slide.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION,slidePID);

        liftPIDF.p=8;//was 5
        liftPIDF.i=0;
        liftPIDF.d=0;
        liftPIDF.f=0;
        liftPIDF.algorithm=MotorControlAlgorithm.PIDF;
        lift.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION,slidePID);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);



    }

    public void setWheelSpeeds(double LF, double LB, double RF, double RB){
        this.leftFrontMotor.setPower(LF);
        this.leftFrontMotor.setVelocity(LF*3100);

        this.rightFrontMotor.setPower(RF);
        this.rightFrontMotor.setVelocity(RF*3100);

        this.leftBackMotor.setPower(LB);
        this.leftBackMotor.setVelocity(LB*3100);

        this.rightBackMotor.setPower(RB);
        this.rightBackMotor.setVelocity(RB*3100);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Power Left Front: ", LF);
        packet.put("Power Right Front : ", RF);
        packet.put("Power Left Rear: ", LB);
        packet.put("Power Right Rear: ", RB);

       // dashboard.sendTelemetryPacket(packet);
    }

    public void setHomePositions(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }



    public float getRobotAngle() {
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle);
    }

    public float getRobotAngleRad() {
        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }



    public void moveUpdate () {
        switch (moveStep) {
            case 0:
                break; //End everything

                //MOVE THE ROBOT
            case 1:
                leftFrontMotor.setPower(movePower);
                leftBackMotor.setPower(movePower);
                rightFrontMotor.setPower(movePower);
                rightBackMotor.setPower(movePower);
                moveStep = 11; //this and the break statement can be removed to no effect
                break;
            case 101:
                leftFrontMotor.setPower(movePower);
                leftBackMotor.setPower(movePower);
                rightFrontMotor.setPower(movePower);
                rightBackMotor.setPower(movePower);

                moveStep = 167;
                break;
            case 167:
                correctionFactor=-0.012f*(initRobotAngle-getRobotAngle());
                cumulativeError+=(0.009f)*correctionFactor;
                totalCumulative=cumulativeError+correctionFactor;
                leftFrontMotor.setPower(movePower+totalCumulative);
                leftBackMotor.setPower(movePower+totalCumulative);
                rightFrontMotor.setPower(movePower-totalCumulative);
                rightBackMotor.setPower(movePower-totalCumulative);
                if ((Math.abs(moveTarget*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(rightFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(leftBackMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(rightBackMotor.getCurrentPosition()) <= 0))
                    moveStep=11;
                break;
            case 11:
                if (Math.abs(moveTarget*countPerInch * .1) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0) {
                    leftFrontMotor.setPower(movePower);
                    leftBackMotor.setPower(movePower);
                    rightFrontMotor.setPower(movePower);
                    rightBackMotor.setPower(movePower);
                    moveStep = 12; //this and the break statement can be removed to no effect
                }
                break;
            case 12:
                if (Math.abs(moveTarget*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0) {
                    leftFrontMotor.setPower(movePower);
                    leftBackMotor.setPower(movePower);
                    rightFrontMotor.setPower(movePower);
                    rightBackMotor.setPower(movePower);
                    moveStep = 2;
                }
                break;
            case 2:
                if (Math.abs(moveTarget*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveStep = 3;
                }
                break;

            //END OF MOVING THE ROBOT

            case 3:
                moveStep = 0;
                break;

                //DEPRECIATED TURNING ALGORITHM
             case 4:
                leftFrontMotor.setPower(movePower);
                leftBackMotor.setPower(movePower);
                rightFrontMotor.setPower(-movePower);
                rightBackMotor.setPower(-movePower);
                moveStep = 5;
                break;
            case 5:
                if (Math.abs(moveTarget*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveStep = 3;
                }
                break;
            //END OF DEPRECIATED TURNING ALGORITHM


            //STRAFE
            case 6:
			  leftFrontMotor.setPower(-movePower);
                leftBackMotor.setPower(movePower);
                rightFrontMotor.setPower(-movePower);
                rightBackMotor.setPower(movePower);
                moveStep = 9;
                break;
            //END OF STRAFE
           
           case 66://STRAFE WITH ANGLE CORRECTION
             cumulativeError=0;
                leftFrontMotor.setPower(-movePower);
                leftBackMotor.setPower(movePower);
                rightFrontMotor.setPower(-movePower);
                rightBackMotor.setPower(movePower);
                moveStep = 67;
                break;
            case 67://STRAFE ANGLE CORRECTION CALCULATIONS
                correctionFactor=-0.024f*(initRobotAngle-getRobotAngle());
                cumulativeError+=(0.009f)*correctionFactor;
                totalCumulative=cumulativeError+correctionFactor;
                leftFrontMotor.setPower(-movePower+totalCumulative);
                leftBackMotor.setPower(movePower+totalCumulative);
                rightFrontMotor.setPower(-movePower-totalCumulative);
                rightBackMotor.setPower(movePower-totalCumulative);
                if ((Math.abs(moveTarget*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(rightFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(leftBackMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(rightBackMotor.getCurrentPosition()) <= 0))
                    moveStep=9;
                break;
                

            case 7:
                if (Math.abs(moveTarget*countPerInch * .1) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0) {
                    leftFrontMotor.setPower(movePower);
                    leftBackMotor.setPower(movePower);
                    rightFrontMotor.setPower(movePower);
                    rightBackMotor.setPower(movePower);
                    moveStep = 12;
                }
                break;
            case 8:
                if (Math.abs(moveTarget*countPerInch * .9) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0) {
                    leftFrontMotor.setPower(movePower * .2);
                    leftBackMotor.setPower(movePower * .2);
                    rightFrontMotor.setPower(movePower * .2);
                    rightBackMotor.setPower(movePower * .2);
                    moveStep = 2;
                }
                break;
            case 9://CHEKCS IF MOVETARGET IS ACQUIRED AND STOPS ROBOT 
                if ((Math.abs(moveTarget*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(rightFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(leftBackMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(moveTarget*countPerInch) - Math.abs(rightBackMotor.getCurrentPosition()) <= 0)) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveStep = 3;

                }
                break;

            case 10: //TURNS THE ROBOT
                if(moveTarget-getRobotAngle() < 0){
                    leftFrontMotor.setPower(movePower);
                    leftBackMotor.setPower(movePower);
                    rightFrontMotor.setPower(-movePower);
                    rightBackMotor.setPower(-movePower);
                    moveStep = 20;
                }
                else{
                    leftFrontMotor.setPower(-movePower);
                    leftBackMotor.setPower(-movePower);
                    rightFrontMotor.setPower(movePower);
                    rightBackMotor.setPower(movePower);
                    moveStep = 21;
                }
                break;

            case 20://ENDS TURN
                if (moveTarget - getRobotAngle()  >= -2) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveStep = 3;
                }
                break;

            case 21://ENDS TURN OTHER DIRECTION
                if (moveTarget - getRobotAngle() <= 2) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveStep = 3;
                }
                break;
            case 13://TURN ON POINT (TURNS ABOUT RIGHT BACK WHEEL)
                if(moveTarget-getRobotAngle() < 0){
                    leftFrontMotor.setPower(movePower);
                    leftBackMotor.setPower(movePower);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    moveStep = 201;
                }
                else{
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(movePower);
                    rightBackMotor.setPower(movePower);
                    moveStep = 202;
                }
                break;

            case 201:
                if (moveTarget - getRobotAngle()  >= -2) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveStep = 3;
                }
                break;

            case 202:
                if (moveTarget - getRobotAngle() <= 2) {
                    leftFrontMotor.setPower(0);
                    leftBackMotor.setPower(0);
                    rightFrontMotor.setPower(0);
                    rightBackMotor.setPower(0);
                    leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    moveStep = 3;
                }
                break;
            case 14://STRAFE TO SKYSTONE
                //342 left is lower, right is higher
                //right is negative
                if(runningavg>342){
                    movePower=movePower;

                }
                else{
                    movePower=movePower*(-1.0f);
                }
                moveStep=665;
                break;
            case 665://STRAFE WITH ANGLE CORRECTION FOR SKYSTONE
                initRobotAngle=getRobotAngle();
                cumulativeError=0;
                leftFrontMotor.setPower(-movePower);
                leftBackMotor.setPower(movePower);
                rightFrontMotor.setPower(-movePower);
                rightBackMotor.setPower(movePower);
                moveStep = 675;
                break;
            case 675: //IMPLEMENTATION OF ANGLE CORRECTION FOR SKYSTONE
                correctionFactor=-0.012f*(initRobotAngle-getRobotAngle());
                cumulativeError+=(0.009f)*correctionFactor;
                totalCumulative=cumulativeError+correctionFactor;
                leftFrontMotor.setPower(-movePower+totalCumulative);
                leftBackMotor.setPower(movePower+totalCumulative);
                rightFrontMotor.setPower(-movePower-totalCumulative);
                rightBackMotor.setPower(movePower-totalCumulative);
                if (Math.abs(runningavg-342)<=10){
                moveStep=95;
            }
            break;
            case 95:
                leftFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightBackMotor.setPower(0);
                leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveStep = 3;
                break;

            //END OF TURNING THE ROBOT
            /*case 6:
                arm1.setPower(movePower);
                arm2.setPower(movePower);
                moveStep = 7;
                break;
            case 7:
                arm1.setPower(0);
                arm2.setPower(0);
                arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveStep = 3;
                break;

                */

            case 40: //Move profile
                correctionFactor=-0.012f*(initRobotAngle-getRobotAngle());
                cumulativeError+=(0.009f)*correctionFactor;
                totalCumulative=cumulativeError+correctionFactor;
                t = (float)period.seconds() - startTime;
                if(t < accelTime){
                    movePower = (vMax/accelTime)*t;
                }
                else if(t < (accelTime+t2)){
                    movePower = vMax;
                }
                else if(t<((accelTime*2)+t2)){
                    movePower = vMax-(vMax/accelTime)*(t - (accelTime+t2));
                }
                else{
                    if(vMax > 0){
                        movePower = 0.1f;
                    }
                    else{
                        movePower = -0.1f;
                    }
                }
                leftFrontMotor.setPower(movePower+totalCumulative);
                leftBackMotor.setPower(movePower+totalCumulative);
                rightFrontMotor.setPower(movePower-totalCumulative);
                rightBackMotor.setPower(movePower-totalCumulative);
                if ((Math.abs(distance*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(distance*countPerInch) - Math.abs(rightFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(distance*countPerInch) - Math.abs(leftBackMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(distance*countPerInch) - Math.abs(rightBackMotor.getCurrentPosition()) <= 0)) {
                    moveStep = 2;
                }
                break;

            case 41: //Move profile strafe
                correctionFactor=-0.024f*(initRobotAngle-getRobotAngle());
                cumulativeError+=(0.009f)*correctionFactor;
                totalCumulative=cumulativeError+correctionFactor;
                t = (float)period.seconds() - startTime;
                if(t < accelTime){
                    movePower = (vMax/accelTime)*t;
                }
                else if(t < (accelTime+t2)){
                    movePower = vMax;
                }
                else if(t<((accelTime*2)+t2)){
                    movePower = vMax-(vMax/accelTime)*(t - (accelTime+t2));
                }
                else{
                    if(vMax > 0){
                        movePower = 0.1f;
                    }
                    else{
                        movePower = -0.1f;
                    }
                }
                leftFrontMotor.setPower(-movePower+totalCumulative);
                leftBackMotor.setPower(movePower+totalCumulative);
                rightFrontMotor.setPower(-movePower-totalCumulative);
                rightBackMotor.setPower(movePower-totalCumulative);
                if ((Math.abs(distance*countPerInch) - Math.abs(leftFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(distance*countPerInch) - Math.abs(rightFrontMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(distance*countPerInch) - Math.abs(leftBackMotor.getCurrentPosition()) <= 0)
                        &&(Math.abs(distance*countPerInch) - Math.abs(rightBackMotor.getCurrentPosition()) <= 0)) {
                    moveStep = 2;
                }
                break;


            default:
                moveStep = 0;
                break;
        }
}

}
