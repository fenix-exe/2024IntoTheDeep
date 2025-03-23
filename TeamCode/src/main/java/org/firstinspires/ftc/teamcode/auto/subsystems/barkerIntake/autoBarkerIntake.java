package org.firstinspires.ftc.teamcode.auto.subsystems.barkerIntake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ActiveIntakeMessage;
import org.firstinspires.ftc.teamcode.common.PinchRollerIntake;

import androidx.annotation.NonNull;

public class autoBarkerIntake extends PinchRollerIntake {

    private final DownsampledWriter barkerWriter;

    public autoBarkerIntake(CRServoImplEx intake) {
        super(intake);
        barkerWriter = new DownsampledWriter("BARKER INTAKE INFO", 50_000_000);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public class barkerIntakePower implements Action {
        private final double power;

        public barkerIntakePower(double power) {
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPower(power);
            barkerWriter.write(new ActiveIntakeMessage(power));
            return false;
        }
    }

    public Action barkerIntakePower(double power) {
        return new barkerIntakePower(power);
    }

}
