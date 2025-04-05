package org.firstinspires.ftc.teamcode.teleop.subsytems.intake.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.BigWheelIntake;
@TeleOp
public class ColorSensorTest extends LinearOpMode {
    ColorSensor colorSensor;
    RevColorSensorV3 hardwareColorSensor;
    CRServoImplEx leftRoller;
    CRServoImplEx rightRoller;
    BigWheelIntake intake;
    enum Alliance{RED,BLUE}
    Alliance alliance = Alliance.RED;
    double intakePower = 0;
    boolean exitingWrongColor = false;
    boolean detectingColor = false;
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareColorSensor = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        leftRoller = hardwareMap.get(CRServoImplEx.class, "leftRoller");
        rightRoller = hardwareMap.get(CRServoImplEx.class, "rightRoller");
        leftRoller.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensor = new ColorSensor(hardwareColorSensor);
        intake = new BigWheelIntake(leftRoller,rightRoller);

        while (opModeInInit()){
            if (gamepad1.a){
                alliance = Alliance.BLUE;
            }
            if (gamepad1.b){
                alliance = Alliance.RED;
            }
        }
        waitForStart();

        while (opModeIsActive()){
            colorSensor.updateHSVandDistance();
            colorSensor.updateDetectColor();

            detectingColor = colorSensor.detectingBlue() || colorSensor.detectingRed() || colorSensor.detectingYellow();

            if (colorSensor.detectingBlue()){
                if (alliance == Alliance.BLUE){
                    telemetry.addLine("PICKED UP ALLIANCE COLOR, READY TO RETRACT SLIDES");
                    intakePower = 0;
                } else {
                    telemetry.addLine("EJECT");
                    exitingWrongColor = true;
                    intakePower = -1;
                }
            } else if (colorSensor.detectingRed()){
                if (alliance == Alliance.BLUE){
                    telemetry.addLine("EJECT");
                    exitingWrongColor = true;
                    intakePower = -1;
                } else {
                    telemetry.addLine("PICKED UP ALLIANCE COLOR, READY TO RETRACT SLIDES");
                    intakePower = 0;
                }
            } else if (colorSensor.detectingYellow()){
                telemetry.addLine("PICKED UP YELLOW, READY TO RETRACT SLIDES");
                intakePower = 0;
            }

            if (exitingWrongColor && !detectingColor){
                exitingWrongColor = false;
                intakePower = 0;
            }

            if (gamepad1.a){
                intakePower = 0;
            }
            if (gamepad1.b){
                intakePower = 1;
            }
            if (gamepad1.x){
                intakePower = -1;
            }

            if (intakePower == 0){
                intake.stop();
            } else if (intakePower == 1){
                intake.intake();
            } else {
                intake.outtake();
            }
            telemetry.addData("H", colorSensor.getH());
            telemetry.addData("Detecting Blue", colorSensor.getBlue());
            telemetry.update();

        }
    }
}
