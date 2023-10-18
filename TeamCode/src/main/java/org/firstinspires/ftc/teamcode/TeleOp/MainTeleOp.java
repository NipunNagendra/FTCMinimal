package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MainTeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {

    Movement move;
    RevColorSensorV3 cs;
    DistanceSensor ds;
    String moveType = "robot";
    double leftY;
    double leftX;
    double rightX;
    int red;
    int green;
    int blue;

    double[] motorPower = {0, 0, 0, 0};

    public void init() {
        move = new Movement(hardwareMap);
        cs = hardwareMap.get(RevColorSensorV3.class, "cs");
        ds = hardwareMap.get(DistanceSensor.class, "ds");

        telemetry.addData("init", "completed");
        telemetry.update();
    }

    @Override
    public void loop()
    {

        red = cs.red();
        green = cs.green();
        blue = cs.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red,green,blue,hsv);
        float hue = hsv[0];
        String color;

        if(hue>=190 && hue<=300){
            color="purple";
        }
        else if(hue>=105 && hue<=150 ){
            color="green";
        }
        else if(hue>=16 && hue<=100){
            color="yellow";
        }
        else{
            color="weird color";
        }

        telemetry.addData("Red: ",cs.red());
        telemetry.addData("Green: ",cs.green());
        telemetry.addData("Blue: ",cs.blue());
        telemetry.addData("Hue", hue);
        telemetry.addLine(color);
        telemetry.addData("Distance(Cm)", ds.getDistance(DistanceUnit.CM));
        telemetry.update();


        if (Math.abs(gamepad1.left_stick_y)  > 0.1 ||
                Math.abs(gamepad1.left_stick_x)  > 0.1 ||
                Math.abs(gamepad1.right_stick_x) > 0.1) {

            leftY = gamepad1.left_stick_y;
            leftX = -1*(gamepad1.left_stick_x);
            rightX = (-gamepad1.right_stick_x);

            motorPower = move.holonomicDrive(leftX, leftY, rightX);
        }
        else {
            motorPower = move.holonomicDrive(0, 0, 0);
        }

        move.setPowers(motorPower[0], motorPower[1], motorPower[2], motorPower[3]);
    }

}
