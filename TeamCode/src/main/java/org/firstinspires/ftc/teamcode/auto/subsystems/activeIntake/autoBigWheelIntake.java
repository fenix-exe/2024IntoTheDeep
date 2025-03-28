package org.firstinspires.ftc.teamcode.auto.subsystems.activeIntake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ActiveIntakeMessage;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.BigWheelIntake;

import androidx.annotation.NonNull;

public class autoBigWheelIntake extends BigWheelIntake {

    private final DownsampledWriter bigWheelWriter;

    public autoBigWheelIntake(CRServoImplEx leftRoller, CRServoImplEx rightRoller) {
        super(leftRoller, rightRoller);
        super.leftRoller = leftRoller;
        super.rightRoller = rightRoller;
        bigWheelWriter = new DownsampledWriter("BIG WHEEL INTAKE INFO", 50_000_000);
    }

    public void setPower(double power) {
        leftRoller.setPower(power);
        rightRoller.setPower(power);
    }

    public class  bigWheelIntakePower implements Action {
        private final double power;

        bigWheelIntakePower(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPower(power);
            bigWheelWriter.write(new ActiveIntakeMessage(power));
            return false;
        }
    }

    public Action bigWheelIntakePower(double power) {
        return new bigWheelIntakePower(power);
    }
}
