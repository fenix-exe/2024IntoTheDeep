package org.firstinspires.ftc.teamcode.auto.subsystems.elbow;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ElbowMessage;
import org.firstinspires.ftc.teamcode.common.CommonElbow;

import androidx.annotation.NonNull;


public class Elbow extends CommonElbow {
    public DcMotorEx elbowMotor;
    public RevTouchSensor limitSwitch;
    int elbowPosition;
    public int topPosition;
    double encoderRes = 145.1;
    double gearRatio = 28;
    double degreesInTicks = (encoderRes*gearRatio)/360; //calculation: ((tick per revolution) * (gear ratio)) / 360
    private final DownsampledWriter elbowWriter;

    public Elbow(DcMotorEx elbow, RevTouchSensor limitSwitch, int topPosition){
        super(elbow, limitSwitch, topPosition);
        this.elbowMotor = elbow;
        elbowWriter = new DownsampledWriter("ELBOW INFO", 50_000_000);
    }


    public class elbowControl implements Action {
        private final double target;
        private final double speed;

        elbowControl(double targetPos, double speed) {
            this.target = targetPos;
            this.speed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setTargetAngleAndSpeed(target, speed);
            elbowWriter.write(new ElbowMessage(getElbowAngle(), target, elbowMotor.getCurrent(CurrentUnit.MILLIAMPS)));
            if (target-0.5 < getElbowAngle() && getElbowAngle() < target+0.5) {
                elbowMotor.setPower(0);
                return false;
            } else {
                return true;
            }
        }


    }
    public Action elbowControl(double targetPos, double speed) {
        return new elbowControl(targetPos, speed);
    }



}
