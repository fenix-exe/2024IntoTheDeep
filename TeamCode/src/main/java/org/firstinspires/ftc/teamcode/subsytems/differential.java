package org.firstinspires.ftc.teamcode.subsytems;

import com.qualcomm.robotcore.hardware.Servo;

public class differential {
    Servo left;
    Servo right;

    float leftRange = 270;
    float rightRange = 270;
    float pitchError = 7;
    float rollError = 0;

    public differential(Servo left, Servo right){
        this.left = left;
        this.right = right;
    }
    

}
