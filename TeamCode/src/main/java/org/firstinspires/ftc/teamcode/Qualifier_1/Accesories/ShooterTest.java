package org.firstinspires.ftc.teamcode.Qualifier_1.Accesories;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * <h1> Shooter test program</h1>
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
@Autonomous(name= "Shooter Test")
public class ShooterTest extends LinearOpMode{
    Shooter robot=new Shooter();

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        robot.initChassis(this);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        robot.shootHighGoal(200000);
    }



}
