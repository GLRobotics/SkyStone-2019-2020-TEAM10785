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
public class SSAUTOResets extends LinearOpMode {

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
    public final int slidemax = 461;
    public final int liftmin = -570;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.setHomePositions();
        robot.claw.setPosition(0);
        robot.startingAngle=0;

        waitForStart();

        while(opModeIsActive()) {

        }

        }
    }
