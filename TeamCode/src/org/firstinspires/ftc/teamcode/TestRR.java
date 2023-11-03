package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

/*
 * Op mode for testing Roadrunner.
 */
@Config
@Autonomous(group = "drive")
public class TestRR extends LinearOpMode {

    SampleMecanumDrive drive = null;
    Servo servo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // Get a SampleMecanumDrive object
        drive = new SampleMecanumDrive(hardwareMap);

        //Get a Servo object from the hardware map
        servo = hardwareMap.get(Servo.class, "back_servo");

        // Specify the starting pose of the robot on the field. ESSENTIAl.
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Build a Trajectory Sequence
        // Segments of the sequence go above the ".build()" method call.
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(72, 0), Math.toRadians(0))
                .build();

        waitForStart();

        drive.followTrajectorySequence(trajSeq);
    }
}
