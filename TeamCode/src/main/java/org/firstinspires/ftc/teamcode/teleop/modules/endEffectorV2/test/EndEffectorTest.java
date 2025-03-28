package org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.teleop.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.teleop.subsytems.wrist.Wrist;
@Disabled
public class EndEffectorTest extends LinearOpMode {
    Servo pitchServo;
    Servo rollServo;
    Servo clawServo;
    EndEffectorV2 endEffector;
    Wrist wrist;
    Claw claw;
    double pitchPosition;
    double rollPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        pitchServo = hardwareMap.get(Servo.class, "pitch");
        rollServo = hardwareMap.get(Servo.class, "roll");
        clawServo = hardwareMap.get(Servo.class, "claw");
        pitchServo.setDirection(Servo.Direction.REVERSE);
        rollServo.setDirection(Servo.Direction.REVERSE);
        wrist = new Wrist(pitchServo);
        claw = new Claw(clawServo);
        endEffector = new EndEffectorV2(wrist, claw);

        waitForStart();

        while (opModeIsActive()){
            pitchPosition = pitchServo.getPosition();
            rollPosition = rollServo.getPosition();

            if (gamepad1.dpad_down) {
                endEffector.manualPitch(-1, pitchPosition);
            }
            if (gamepad1.dpad_up) {
                endEffector.manualPitch(1, pitchPosition);
            }
            if (gamepad1.a) {
                endEffector.presetPitch(0);
            }
            if (gamepad1.left_bumper) {
                endEffector.openClaw();
            }
            if (gamepad1.right_bumper){
                endEffector.closeClaw();
            }
            telemetry.addData("Pitch position", pitchPosition);
            telemetry.addData("Roll position", rollPosition);
            telemetry.update();
        }
    }
}
