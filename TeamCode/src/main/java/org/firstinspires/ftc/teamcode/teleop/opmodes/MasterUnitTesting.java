package org.firstinspires.ftc.teamcode.teleop.opmodes;

import static org.firstinspires.ftc.teamcode.auto.roadrunner.PinpointDrive.PARAMS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.modules.driverControl.DriverControls;
import org.firstinspires.ftc.teamcode.teleop.robot.RobotConstants;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.teleop.subsytems.IMU.IMUforREV;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.test.DriveTrainTest;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.teleop.subsytems.elbow.test.ElbowTest;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.BigWheelIntake;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.test.ColorSensorTest;
import org.firstinspires.ftc.teamcode.teleop.subsytems.slide.test.SlideMaxVelocityFinder;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.test.WristTest;

import java.util.ArrayList;
@TeleOp(group="Testing")
@Disabled
public class MasterUnitTesting extends LinearOpMode {
    private class Tuple{
        public Tuple(String label, LinearOpMode opMode){
            this.label = label;
            this.opMode = opMode;
        }
        String label;
        LinearOpMode opMode;
    }
     //enum mode {DRIVETRAIN, ELBOW, SLIDES, INTAKE}
    int indexNumber = 0;
    public static double x = 39.7, y = 65, heading = -180;
     public GoBildaPinpointDriverRR pinpoint;
     ArrayList<Tuple> opModes = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        opModes.add(new Tuple("elbow test", new ElbowTest()));
        opModes.add(new Tuple("wrist test",new WristTest()));
        opModes.add(new Tuple("drivetrain test", new DriveTrainTest()));
        opModes.add(new Tuple("color sensor test", new ColorSensorTest()));
        opModes.add(new Tuple ("slide maximum velocity finder",new SlideMaxVelocityFinder()));

        while (opModeInInit()){
            if (gamepad1.dpad_up){
                indexNumber++;
            }
            if (gamepad1.dpad_down){
                indexNumber--;
            }
            if (indexNumber > opModes.size() - 1){
                indexNumber = 0;
            }
            if (indexNumber < 0){
                indexNumber = opModes.size() - 1;
            }
            telemetry.addLine("Press the d-pad to cycle between units to test. In any unit, hold down Gamepad 1's x to see instructions.");
            telemetry.addLine("Current Selected Test Code: " + opModes.get(indexNumber).label);
            telemetry.update();
            sleep(500);
            if (gamepad1.x) {
                break;
            }
        }
        //waitForStart();



        opModes.get(indexNumber).opMode.start();
    }
}
//pitch didn't go up on third one