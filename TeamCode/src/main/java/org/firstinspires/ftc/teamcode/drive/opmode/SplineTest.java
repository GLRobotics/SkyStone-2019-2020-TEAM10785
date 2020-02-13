package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        MecanumConstraints constraints=new MecanumConstraints(DriveConstants.BASE_CONSTRAINTS,DriveConstants.TRACK_WIDTH,8.5);
        waitForStart();

        if (isStopRequested()) return;
        Trajectory trajectory=new TrajectoryBuilder(new Pose2d(0,0,0),constraints).lineTo(new Vector2d(0,72),new ConstantInterpolator(0)).build();
        drive.followTrajectorySync(trajectory);

        sleep(2000);
        Trajectory tragectoryReverse=new TrajectoryBuilder(new Pose2d(0,72,0),constraints).lineTo(new Vector2d(0,0),new ConstantInterpolator(0)).build();

        drive.followTrajectorySync(tragectoryReverse);
    }
}
