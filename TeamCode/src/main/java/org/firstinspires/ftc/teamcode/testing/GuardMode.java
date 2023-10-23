package org.firstinspires.ftc.teamcode.testing;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class GuardMode extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    double xyParameter = 1;
    double headingParameter = 1;
    double lockThreshold = 0;
    Pose2d lockLocation = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            if((Math.abs(drive.getPoseEstimate().getX() - lockLocation.getX()) >= lockThreshold || Math.abs(drive.getPoseEstimate().getY() - lockLocation.getY()) >= lockThreshold)){
                lockTo(lockLocation);
            }
            drive.update();
        }
    }

    public void lockTo(Pose2d targetPos) {
        Pose2d currPos = drive.getPoseEstimate();
        Pose2d difference = targetPos.minus(currPos);
        Vector2d xy = difference.vec().rotated(-currPos.getHeading());
        double heading = Angle.normDelta(targetPos.getHeading()) - Angle.normDelta(currPos.getHeading());

        drive.setWeightedDrivePower(new Pose2d(xy.times(xyParameter), heading* headingParameter));
    }}