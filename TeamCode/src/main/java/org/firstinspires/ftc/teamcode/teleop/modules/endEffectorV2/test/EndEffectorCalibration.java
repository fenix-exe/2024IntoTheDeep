package org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
@Config
@TeleOp
//@Disabled
public class EndEffectorCalibration extends LinearOpMode {
    Servo pitchServo;
    Servo rollServo;
    Servo clawServo;
    EndEffectorV2 endEffector;
    Wrist wrist;
    Claw claw;
    double pitchPosition;
    double rollPosition;
    public static double pitchPos;
    public static double rollPos;
    public static boolean closeClaw = false;

    @Override
    public void runOpMode() throws InterruptedException {
        pitchServo = hardwareMap.get(Servo.class, "pitch");
        rollServo = hardwareMap.get(Servo.class, "roll");
        clawServo = hardwareMap.get(Servo.class, "claw");
        rollServo.setDirection(Servo.Direction.REVERSE);
        wrist = new Wrist(pitchServo);
        claw = new Claw(clawServo);
        endEffector = new EndEffectorV2(wrist, claw);

        waitForStart();

        while (opModeIsActive()){
            pitchPosition = pitchServo.getPosition();
            rollPosition = rollServo.getPosition();

            wrist.presetPositionPitch(pitchPos);

            if (closeClaw){
                claw.closeClaw();
            } else {
                claw.openClaw();
            }

            telemetry.addData("Pitch position", pitchPosition);
            telemetry.addData("Roll position", rollPosition);
            telemetry.addData("Pitch Angle", wrist.getPitchAngle());

            telemetry.update();
        }
    }
}

