package org.firstinspires.ftc.teamcode.testing;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class GuardMode extends OpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    double xyParameter;
    double headingParameter;
    double lockThreshold;
    Pose2d lockLocation;
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        xyParameter= 1;
        headingParameter= 1;
        lockThreshold=0;
        lockLocation= new Pose2d(0,0,0);
        telemetry.addData("init", "completed");
    }

    @Override
    public void loop(){

        if((Math.abs(drive.getPoseEstimate().getX() - lockLocation.getX()) >= lockThreshold
                ||
                Math.abs(drive.getPoseEstimate().getY() - lockLocation.getY()) >= lockThreshold)){
            lockTo(lockLocation);
        }

        drive.update();
    }

    public void lockTo(Pose2d targetPos) {
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());
        double heading = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());

        drive.setWeightedDrivePower(new Pose2d(xy.times(xyParameter), heading* headingParameter));
    }}