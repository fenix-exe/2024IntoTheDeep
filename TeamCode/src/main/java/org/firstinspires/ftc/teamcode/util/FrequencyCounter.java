package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FrequencyCounter {

    long counter = 0;
    ElapsedTime timer;

    double min_frequency = Double.MAX_VALUE;
    double max_frequency = 0;

    double ave_frequency = 0;

    double n_count = 0;

    public void count() {
        if(timer == null) {
            timer  = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        if (counter > 10000  || timer.milliseconds() > 10000){
            double frequency = counter/timer.seconds();
            min_frequency = Math.min(min_frequency, frequency);
            max_frequency = Math.max(max_frequency, frequency);
            ave_frequency = (ave_frequency*n_count + frequency ) / (n_count + 1);
            n_count++;
            // reset the counter and timer
            counter = 0;
            timer.reset();
        }else {
            counter++;
        }
    }
    public double getAveFrequency(){
        return ave_frequency;
    }

}
