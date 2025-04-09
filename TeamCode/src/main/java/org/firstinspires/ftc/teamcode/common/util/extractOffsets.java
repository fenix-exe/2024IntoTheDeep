package org.firstinspires.ftc.teamcode.common.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class extractOffsets {

    public double getPitchOffset() {
        return pitchOffset;
    }

    public void setPitchOffset(double pitchOffset) {
        this.pitchOffset = pitchOffset;
    }

    private double pitchOffset;

    public void offsetGetter(String filename) throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(filename));
        String line;
        ArrayList<Double> offsets = new ArrayList<>();
        br.readLine();
        while ((line = br.readLine()) != null) {
            String[] values = line.split(",");
            for (String value : values) {
                offsets.add(Double.parseDouble(value));
            }
        }
        br.close();
        setPitchOffset(offsets.get(0));

    }

}
