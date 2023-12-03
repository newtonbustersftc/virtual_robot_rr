package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Arrays;

//almost worked version for high pole first and then low pole and possible ground junction
public class AutonomousTaskBuilder {
    RobotProfile robotProfile;    RobotHardware robotHardware;
    ArrayList<RobotControl> taskList = new ArrayList<>();
    SampleMecanumDrive drive;
    String delayString;
    String startPosMode;
    String parking;
    Pose2d startingPose;
    Pose2d lastLocation;
    String teamPropPos;  //= RobotCVProcessor.TEAM_PROP_POS.CENTER;
    TrajectorySequence dropBoard_traj_a=null, dropBoard_traj_b=null, dropBoard_traj_c=null;

    boolean isFarSideLeft =false;
    TrajectorySequence  team_prop_pos_traj=null,dropBoard_traj=null,lastTrajectory=null;
    boolean isRed;
    boolean isFar;

    public AutonomousTaskBuilder(RobotHardware robotHardware, RobotProfile robotProfile,
                                 String teamPropPos, Pose2d startingPose) {
        this.robotHardware = robotHardware;
        this.robotProfile = robotProfile;
        drive = robotHardware.getMecanumDrive();
        this.teamPropPos = teamPropPos;
        this.startingPose = startingPose;
    }

    public ArrayList<RobotControl> buildTaskList() {
        try {
            delayString = "0";
            startPosMode = "BLUE_RIGHT";
            Logger.logFile(AutonomousOptions.START_POS_MODES_PREF + " - " + startPosMode);
            Logger.logFile(AutonomousOptions.START_DELAY_PREF + " - " + delayString);
            if(startPosMode.startsWith("RED")){
                isRed = true;
            }
            isFar = (startPosMode.equals("BLUE_RIGHT") || startPosMode.equals("RED_LEFT"));
        } catch (Exception e) {
            RobotLog.e("SharedPref exception " + e);
            this.delayString = "0";
        }
        Logger.logFile("Done with init in autonomous");

        RobotProfile.AutonParam param = robotProfile.autonParam;
        TrajectoryVelocityConstraint velFast = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), robotProfile.hardwareSpec.trackWidth);
        TrajectoryAccelerationConstraint accelFast = getAccelerationConstraint(param.fastAcceleration);
        TrajectoryVelocityConstraint velConstraint = getVelocityConstraint(param.normVelocity, Math.toRadians(param.normAngVelo), 3);
        TrajectoryAccelerationConstraint accelConstraint = getAccelerationConstraint(param.normAcceleration);

        if (!delayString.equals("0")) {
            taskList.add(new RobotSleep(Integer.parseInt(delayString) * 1000));
        }
        Pose2d propPose = robotProfile.getProfilePose("TEAM_PROP_POS_" + teamPropPos + "_" +startPosMode);
        Pose2d dropPose = robotProfile.getProfilePose("DROPBOARD_APRILTAG_" + (isRed?"RED":"BLUE") + "_" + teamPropPos);
        Pose2d parkingPose = robotProfile.getProfilePose("PARKING_" + startPosMode);

        team_prop_pos_traj = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .splineTo(propPose.vec(), propPose.getHeading() + Math.PI)
                .build();

        taskList.add(new SplineMoveTask(drive, team_prop_pos_traj));
        taskList.add(new RobotSleep(2000, "DropSpike"));
        if (!isFar) {
            dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                    .splineTo(dropPose.vec(), dropPose.getHeading())
                    .build();
            taskList.add(new SplineMoveTask(drive, dropBoard_traj));
        }
        else {
            Pose2d wp1 = robotProfile.getProfilePose("WAY_POINT1_" + teamPropPos + "_" +startPosMode);
            Pose2d wp2 = robotProfile.getProfilePose("WAY_POINT2_" + teamPropPos + "_" +startPosMode);
            Pose2d wp3 = robotProfile.getProfilePose("WAY_POINT3_" + teamPropPos + "_" +startPosMode);
            TrajectorySequenceBuilder bldr = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
                    .splineTo(wp1.vec(), wp1.getHeading())
                    .splineTo(wp2.vec(), wp2.getHeading());
            if (wp3!=null) {
                bldr.splineTo(wp3.vec(), wp3.getHeading())    ;
            }
            dropBoard_traj = bldr.build();
//            dropBoard_traj = drive.trajectorySequenceBuilder(team_prop_pos_traj.end())
//                    .splineTo(wp1.vec(), wp1.getHeading())
//                    .splineTo(wp2.vec(), wp2.getHeading())
//                    .build();
            taskList.add(new SplineMoveTask(drive, dropBoard_traj));
        }

        return taskList;
    }

    Pose2d getProfilePose(String name) {
        RobotProfile.AutoPose ap = robotProfile.poses.get(name);
        return new Pose2d(ap.x, ap.y, Math.toRadians(ap.heading));
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
