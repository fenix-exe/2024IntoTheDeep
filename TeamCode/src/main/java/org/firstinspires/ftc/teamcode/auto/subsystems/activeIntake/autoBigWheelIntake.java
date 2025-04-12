package org.firstinspires.ftc.teamcode.auto.subsystems.activeIntake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.auto.roadrunner.messages.ActiveIntakeMessage;
import org.firstinspires.ftc.teamcode.teleop.subsytems.colorSensor.ColorSensor;
import org.firstinspires.ftc.teamcode.teleop.subsytems.intake.BigWheelIntake;

import androidx.annotation.NonNull;

public class autoBigWheelIntake extends BigWheelIntake {

    private final DownsampledWriter bigWheelWriter;
    private ColorSensor color;

    public autoBigWheelIntake(CRServoImplEx leftRoller, CRServoImplEx rightRoller) {
        super(leftRoller, rightRoller);
        super.leftRoller = leftRoller;
        super.rightRoller = rightRoller;
        bigWheelWriter = new DownsampledWriter("BIG WHEEL INTAKE INFO", 50_000_000);
    }
    public autoBigWheelIntake(CRServoImplEx leftRoller, CRServoImplEx rightRoller, ColorSensor color) {
        super(leftRoller, rightRoller);
        super.leftRoller = leftRoller;
        super.rightRoller = rightRoller;
        this.color = color;
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

    public class colorIntake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
           color.updateHSVandDistance();
           double distance = color.getDistance();
           if (distance < 32.5) {
               setPower(0);
               return false;
           } else {
               setPower(1);
               return true;
           }

        }
    }

    public Action colorIntake() {
        return new colorIntake();
    }
}
