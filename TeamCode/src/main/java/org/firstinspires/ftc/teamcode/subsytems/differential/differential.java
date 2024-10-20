package org.firstinspires.ftc.teamcode.subsytems.differential;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class differential {
    Servo left;
    Servo right;

    float leftRange = 270;
    float rightRange = 270;
    public static float pitchError = -13;
    public static float rollError = -18;

    public differential(Servo left, Servo right){
        this.left = left;
        this.right = right;
    }

    private double setAPosition(double pitch, double roll) {
        double aPos;
        aPos = (1 / leftRange) * (pitch + roll / 2) + 0.5;
        return aPos;
    }

    private double setBPosition(double pitch, double roll) {
        double bPos;
        bPos = (1 / rightRange) * (pitch - roll / 2) + 0.5;
        return bPos;
    }

    public void setDifferentialPosition(double pitch, double roll){
        left.setPosition(setAPosition(pitch+pitchError, roll+rollError));
        right.setPosition(setBPosition(pitch+pitchError, roll+rollError));
    }

    

}
