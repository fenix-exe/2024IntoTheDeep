package org.firstinspires.ftc.teamcode.modules.endEffectorV2.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.arm.Arm;
import org.firstinspires.ftc.teamcode.modules.endEffectorV2.EndEffectorV2;
import org.firstinspires.ftc.teamcode.subsytems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsytems.elbow.Elbow;
import org.firstinspires.ftc.teamcode.subsytems.elbow.PIDControl;
import org.firstinspires.ftc.teamcode.subsytems.slide.Slide;
import org.firstinspires.ftc.teamcode.subsytems.wrist.Wrist;

@TeleOp
public class EndEffectorSlideTest extends LinearOpMode {
    Servo pitchServo;
    Servo rollServo;
    Servo clawServo;
    EndEffectorV2 endEffector;
    Wrist wrist;
    Claw claw;
    double pitchPosition;
    double rollPosition;
    Arm arm;
    Slide slide;
    Elbow elbow;
    DcMotorEx elbowMotor;
    DcMotorEx slideMotor;
    RevTouchSensor limitSwitch;


    @Override
    public void runOpMode() throws InterruptedException {
        pitchServo = hardwareMap.get(Servo.class, "pitch");
        rollServo = hardwareMap.get(Servo.class, "roll");
        clawServo = hardwareMap.get(Servo.class, "claw");
        wrist = new Wrist(pitchServo, rollServo);
        claw = new Claw(clawServo);
        endEffector = new EndEffectorV2(wrist, claw);
        endEffector.goToPresetPosition(0.5,0.5);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        elbowMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(RevTouchSensor.class, "limit switch");

        //pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide= new Slide(slideMotor);
        elbow = new Elbow(elbowMotor, limitSwitch, new PIDControl(new PIDController(0.019, 0.006, 0.00022), 0,24.22), 2300);
        arm = new Arm(slide, elbow);

        slideMotor.setTargetPosition(0);
        elbowMotor.setTargetPosition(0);


        waitForStart();

        while (opModeIsActive()){
            pitchPosition = pitchServo.getPosition();
            rollPosition = rollServo.getPosition();

            if (gamepad1.dpad_down) {
                endEffector.manualPitch(-0.001, pitchPosition);
            }
            if (gamepad1.dpad_up) {
                endEffector.manualPitch(0.001, pitchPosition);
            }
            if (gamepad1.dpad_left) {
                endEffector.manualRoll(-0.001, rollPosition);
            }
            if (gamepad1.dpad_right) {
                endEffector.manualRoll(0.001, rollPosition);
            }
            if (gamepad1.a) {
                endEffector.goToPresetPosition(0,0);
            }
            if (gamepad1.left_bumper) {
                endEffector.openClaw();
            }
            if (gamepad1.right_bumper){
                endEffector.closeClaw();
            }
            if (gamepad1.left_stick_y > 0.5 || gamepad1.left_stick_y < -0.5){
                arm.moveSlide(gamepad1.left_stick_y, false);
            } else {
                arm.holdSlide();
            }

            telemetry.addData("Pitch position", pitchPosition);
            telemetry.addData("Roll position", rollPosition);
            telemetry.update();
        }
    }
}
