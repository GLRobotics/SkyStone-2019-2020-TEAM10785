package org.firstinspires.ftc.teamcode.Team10785;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Team10785TeleOpPOVdisabled extends LinearOpMode {

    Robot10785 robot = new Robot10785();

    public Vision visionSystem;
    public String found;
    public int target;
    public int slidetarget;
    //public int clawtarget;
    public int liftCount;
    public int startingLiftPos=0;

    public final int slideout = 411;
    public final int slidein = 0;

    public final int liftceil = 4250;
    public final int liftfloor = -466;
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        telemetry.addData("Stauts","Initialized 1");
        telemetry.update();

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

        while(opModeIsActive()) {

            // Convert joysticks to desired motion.
/*            double robotRad = robot.getRobotAngleRad();
            double x = (gamepad1.left_stick_x * Math.cos(robotRad))-(gamepad1.left_stick_y * Math.sin(robotRad));
            double y = (gamepad1.left_stick_y * Math.cos(robotRad))+(gamepad1.left_stick_x * Math.sin(robotRad));

            // POV Control

            Mecanum.Motion motion = Mecanum.joystickToMotion(
                    x, y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y);
*/
            // Non Adjusted Motion
            Mecanum.Motion motion = Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y);



            // Convert desired motion to wheel powers, with power clamping.
            motion.vTheta = motion.vTheta * 0.8;
            Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
            robot.leftFrontMotor.setPower(wheels.frontLeft);
            robot.rightFrontMotor.setPower(wheels.frontRight);
            robot.leftBackMotor.setPower(wheels.backLeft);
            robot.rightBackMotor.setPower(wheels.backRight);


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
                    robot.claw.setPosition(0.5);
                }
            }//end claw

            //buttons
            {
                if (gamepad2.a) {
                    target = 281; //lvl 803
                }

                if (gamepad2.b) {
                    target = 803; // lvl 3
                }

                if (gamepad2.x) {
                    target = 0; // lvl 1
                }
                if (gamepad2.y) {
                    target = 1284; //level 4
                }

                if (gamepad2.right_stick_y > .9) {
                    target = 1702; //zero out
                }
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

            { // start of hook

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

            telemetry.addData("Lift Position", "%d ", robot.lift.getCurrentPosition());
            telemetry.addData("Slide", "%d ", robot.slide.getCurrentPosition());
            telemetry.addData("robot motor position", "%d", robot.leftFrontMotor.getCurrentPosition());
            telemetry.addData("MCMD", "%.1f %.1f", motion.vD, motion.vTheta);
            telemetry.addData("Rotation", "%f ", robot.getRobotAngle());
            telemetry.update();

        }//end while(OpmodeIsActive);
    }//end public void runopmode();

}//end class
