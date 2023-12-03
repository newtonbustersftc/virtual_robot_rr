package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;

public class RobotHardware {
    public double extensionPos;
    HardwareMap hardwareMap;
    DcMotorEx rrMotor, rlMotor, frMotor, flMotor;
    DcMotorEx intakeMotor, hangerMotor;
    private DcMotorEx[] liftMotors;        // make it private so we can prevent mistakes by lift down while arm is retracted in
    //private
    Servo dronePivotServo, droneReleaseServo, intakeServo1, intakeServo2,
            gripperServo, gripperInOutServo, gripperRotateServo, droppingServo;

    NormalizedColorSensor coneSensor;

    LynxModule expansionHub1;
    LynxModule expansionHub2;
    SampleMecanumDrive mecanumDrive;
    BNO055IMU imu;
    DistanceSensor distanceSensorLeft, distanceSensorRight;
    double gyroOffset;
    boolean gripOpen = false;

    DecimalFormat nf2 = new DecimalFormat("#.##");
    RobotProfile profile;
    enum IntakeMode { ON, REVERSE, OFF }
    IntakeMode intakeMode;
    AprilTagDetection aprilTag;
    TrajectorySequence aprilTagTrajectory;
    Pose2d lastLocation;

    public void init(HardwareMap hardwareMap, RobotProfile profile) {
        Logger.logFile("RobotHardware init()");
        this.hardwareMap = hardwareMap;
        this.profile = profile;

        //expansionHub1 = hardwareMap.get(LynxModule.class, "Control Hub");
        rrMotor = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        rlMotor = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        frMotor = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        flMotor = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        /*
        dronePivotServo = hardwareMap.servo.get("dronePivotServo");
        droneReleaseServo = hardwareMap.servo.get("droneReleaseServo");
        droppingServo = hardwareMap.servo.get("droppingServo");

        expansionHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        intakeServo1 = hardwareMap.servo.get("intakeServo1");
        intakeServo2 = hardwareMap.servo.get("intakeServo2");
        gripperServo = hardwareMap.servo.get("gripperServo");
        gripperInOutServo = hardwareMap.servo.get("gripperInOutServo");
        gripperRotateServo = hardwareMap.servo.get("gripperRotateServo");

        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotor = hardwareMap.get(DcMotorEx.class,"hangerMotor");
        hangerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotors = new DcMotorEx[2];
        liftMotors[0] = hardwareMap.get(DcMotorEx.class,"liftMotorLeft");
        liftMotors[1] = hardwareMap.get(DcMotorEx.class,"liftMotorRight");
        liftMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER); //??
        liftMotors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //??
        liftMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        //two distance sensors
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // Use manual cache mode for most efficiency, but each program
        // needs to call clearBulkCache() in the while loop
        expansionHub1.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //expansionHub1.clearBulkCache();
        expansionHub2.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        */
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        //resetDriveAndEncoders();
        intakeMode = IntakeMode.OFF;
    }

    public List<DcMotorEx> getDriveMotors() {
        return Arrays.asList(flMotor, rlMotor, rrMotor, frMotor);
    }

    public void initGyro() {
        if (profile.hardwareSpec.useControlHubImu) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
        }
    }

    public void intakeDropSpike(int step) {
        if (step==0) {
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setTargetPosition(0);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(0.5); //0.15
            Logger.logFile("intakeDropSpike step=0");
        }
        else {
            intakeMotor.setTargetPosition((profile.hardwareSpec.intakeDropSpikeStep*step));
            Logger.logFile("intakeDropSpike step = "+ step + ", targetPosition = "+profile.hardwareSpec.intakeDropSpikeStep*step);
        }
    }

    public void resetIntakeMotor() {
        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDriveAndEncoders() {
        flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rlMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
//        liftMotors[1].setDirection(DcMotorSimple.Direction.FORWARD);
//        expansionHub1.clearBulkCache();
//        expansionHub2.clearBulkCache();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public SampleMecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public Localizer getLocalizer() {
        return mecanumDrive.getLocalizer();
    }

    public void startIntake() {
        intakeMode = IntakeMode.ON;
        setLiftPosition(profile.hardwareSpec.liftIntakePos);
        grabberOpen();
        grabberIn();
        intakeMotor.setPower(profile.hardwareSpec.intakePower);
        intakeServo1.setPosition(profile.hardwareSpec.intakeServo1In);
        intakeServo2.setPosition(profile.hardwareSpec.intakeServo2In);
    }
    public void reverseIntake() {
        intakeMode = IntakeMode.REVERSE;
        intakeMotor.setPower(profile.hardwareSpec.intakeReversePower);
        intakeServo1.setPosition(profile.hardwareSpec.intakeServo1Out);
        intakeServo2.setPosition(profile.hardwareSpec.intakeServo2Out);
    }

    public void stopIntake() {
        intakeMode = IntakeMode.OFF;
        setLiftPosition(0);
        intakeMotor.setPower(0);
        intakeServo1.setPosition(profile.hardwareSpec.intakeServo1Stop);
        intakeServo2.setPosition(profile.hardwareSpec.intakeServo2Stop);
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }
    public double getDistanceSensorLeft(){
        return distanceSensorLeft.getDistance(DistanceUnit.INCH);
    }

    public double getDistanceSensorRight(){
        return distanceSensorRight.getDistance(DistanceUnit.INCH);
    }

    public int getLiftPosition() {
        return liftMotors[1].getCurrentPosition();
    }

    public int getLiftTargetPosition() {
        return liftMotors[1].getTargetPosition();
    }

    public String getLiftMotorPos() {
        return "Lift12: " + liftMotors[0].getCurrentPosition() + "," + liftMotors[1].getCurrentPosition() ;
    }

    public double getLiftVelocity() {
        return liftMotors[1].getVelocity();
    }

    public void setLiftPosition(int newLiftPos) {
        int currPos = liftMotors[0].getCurrentPosition();
        setLiftPosition(newLiftPos, (currPos<newLiftPos)?profile.hardwareSpec.liftPowerUp:
                profile.hardwareSpec.liftPowerDown);
    }

    public void setLiftPosition(int newLiftPos, double power) {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setTargetPosition(newLiftPos);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(power);
        }
    }

    public void setLiftPower(double power) {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(power);
        }
    }

    public int getTargetLiftPosition() {
        return liftMotors[1].getTargetPosition();
    }

    public boolean isLiftMoving() {
        return Math.abs(liftMotors[1].getVelocity())>5;
    }

    public void mecanumDrive2(double power, double angle, double rotation){
        //10/28/2019, Will, Ian Athena implemented and tested the drive method
        double robotAngle = Math.PI / 2 + angle - Math.PI / 4;
        double frontLeft = power * Math.cos(robotAngle) + rotation;
        double frontRight = power * Math.sin(robotAngle) - rotation;
        double rearLeft = power * Math.sin(robotAngle) + rotation;
        double rearRight = power * Math.cos(robotAngle) - rotation;


        double biggest = 0;
        if (Math.abs(frontRight) > biggest){
            biggest = Math.abs(frontRight);
        }
        if (Math.abs(rearLeft) > biggest){
            biggest = Math.abs(rearLeft);
        }
        if (Math.abs(rearRight) > biggest){
            biggest = Math.abs(rearRight);
        }
        if (Math.abs(frontLeft) > biggest){
            biggest = Math.abs(frontLeft);
        }

        power = Math.max(power, Math.abs(rotation));
        frontLeft = frontLeft/biggest*power;
        frontRight = frontRight/biggest*power;
        rearLeft = rearLeft/biggest*power;
        rearRight = rearRight/biggest*power;

        setMotorPower(frontLeft, frontRight, rearLeft, rearRight);        // TO FIX!!!
    }

    public void setMotorPower(double flPower, double frPower, double rlPower, double rrPower) {
        try {
            flMotor.setPower(flPower);
            frMotor.setPower(frPower);
            rlMotor.setPower(rlPower);
            rrMotor.setPower(rrPower);
        }
        catch (Exception ex) {
            Logger.logFile("setMotorPower exception: " + ex);
            ex.printStackTrace();
        }
    }

    public void setMotorStopBrake(boolean brake) {
        flMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        frMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rlMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
        rrMotor.setZeroPowerBehavior(brake?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void enableManualCaching(boolean enable) {

    }

    public void clearBulkCache() {
//        expansionHub1.clearBulkCache();
//        expansionHub2.clearBulkCache();
    }

    public void stopAll() {
        setMotorPower(0, 0, 0, 0);
//        for(DcMotorEx liftMotor:liftMotors) {
//            liftMotor.setPower(0);
//        }

    }

    public enum EncoderType {LEFT, RIGHT, HORIZONTAL}

    public double getGyroHeading() {
        if (profile.hardwareSpec.useControlHubImu) {
            double firstAngle = imu.getAngularOrientation().firstAngle;
            return firstAngle;
        }
        return 0;
    }

    public double getGyroVelocity() {
        return 0;
    }

    public void resetImu() {
    }



    RobotProfile getRobotProfile() {
        return profile;
    }

    public int getFLMotorEncoderCnt(){
        return this.flMotor.getCurrentPosition();
    }

    public int getFRMotorEncoderCnt(){
        return this.frMotor.getCurrentPosition();
    }

    public int getRlMotorEncoderCnt(){
        return this.rlMotor.getCurrentPosition();
    }

    public int getRRMotorEncoderCnt(){
        return this.rrMotor.getCurrentPosition();
    }

    public void resetLiftPos() {
        for(DcMotorEx liftMotor : liftMotors) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void grabberOpen() {
        gripOpen = true;
        gripperServo.setPosition(profile.hardwareSpec.grabberOpenPos);
    }

    public void grabberClose() {
        grabberClose(true);
    }
    public void grabberClose(boolean isOne) {
        gripOpen = false;
        gripperServo.setPosition(isOne?profile.hardwareSpec.gripperServo1Pixel:profile.hardwareSpec.gripperServo2Pixel);
    }

    public boolean isGripOpen() {
        return gripOpen;
    }

    public void grabberUp() {
        gripperRotateServo.setPosition(profile.hardwareSpec.gripperRotateServoUp);
    }

    public boolean isGrabberUp() {
        return getLiftPosition() > profile.hardwareSpec.liftOutMin-100;
    }

    public void grabberLeft() {
        gripperRotateServo.setPosition(profile.hardwareSpec.gripperRotateServoLeft);
    }
    public void grabberRight() {
        gripperRotateServo.setPosition(profile.hardwareSpec.gripperRotateServoRight);
    }

    public void grabberIn() {
        gripperInOutServo.setPosition(profile.hardwareSpec.gripperInOutServoIn);
    }

    public void grabberOut() {
        gripperInOutServo.setPosition(profile.hardwareSpec.gripperInOutServoOut);
    }

    public void grabberPreDrop() {
        gripperInOutServo.setPosition(profile.hardwareSpec.gripperInOutServoDrop);
    }

    public void droneShootPosition() {
        dronePivotServo.setPosition(profile.hardwareSpec.droneServoShootPos);
    }

    public void droneInitPosition() {
        dronePivotServo.setPosition(profile.hardwareSpec.droneServoInitPos);
    }

    public void droneLoadPosition() {
        dronePivotServo.setPosition(profile.hardwareSpec.droneServoLoadPos);
    }

    public void droneRelease() {
        droneReleaseServo.setPosition(profile.hardwareSpec.droneHookOpenPos);
    }
    public void droneHook() {
        droneReleaseServo.setPosition(profile.hardwareSpec.droneHookClosePos);
    }

    public void hang(double power) {
        hangerMotor.setPower(power);
    }

    public void initSetupNoAuto(OpMode opmod) {
        /*
        grabberOpen();
        resetLiftPos();
        setLiftPosition(profile.hardwareSpec.liftOutMin);
        long t = System.currentTimeMillis();
        while (!isLiftTouched() && (System.currentTimeMillis()-t)<3000) {
            try {
                Thread.sleep(100);
            }
            catch (Exception ex) {
            }
        }
        resetLiftPos();
        resetLiftPos();
         */
    }

    public void initSetup(LinearOpMode opmode) {
        resetLiftPos();
    }

    DcMotorEx[] getLiftMotors() {
        return liftMotors;
    }

    public void initDroppingStick(){
        this.droppingServo.setPosition(0);
    }

    public void releaseDroppingStick(){
        this.droppingServo.setPosition(0.65);
    }

    public double getDroppingServoNumber(){
        return this.droppingServo.getPosition();
    }

    public void storeDesiredAprilTag(AprilTagDetection desiredTag){
        this.aprilTag = desiredTag;
        Logger.logFile("Tag id="+this.aprilTag.id);
        Logger.logFile("range="+this.aprilTag.ftcPose.range);
        Logger.logFile("yaw="+this.aprilTag.ftcPose.yaw);
        Logger.logFile("x="+this.aprilTag.ftcPose.x);
        Logger.logFile("y="+this.aprilTag.ftcPose.y);
        Logger.logFile("z="+this.aprilTag.ftcPose.z);
        Logger.flushToFile();
    }

    public AprilTagDetection getDesiredAprilTag(){
        return this.aprilTag;
    }

    public void setAprilTagTrajectory(TrajectorySequence aprilTagTraj){
        this.aprilTagTrajectory =aprilTagTraj;
    }

    public TrajectorySequence getAprilTagTrajectory(){
        Logger.logFile("robothardware aprilTagTrajectory is "+ this.aprilTagTrajectory);

        return this.aprilTagTrajectory;
    }
}