package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.util.ArrayList;

public class extractAuto {
    ArrayList<PositionInSpace> autoPath = new ArrayList<>();

    public class PositionInSpace {
        public int x_value;
        public int y_value;
        public double angle;
        public int elbow_phi;
        public int linear_slide;
        public double pitch;
        public double roll;
        public double claw;
        public double wait;

        PositionInSpace(int x_value, int y_value, double angle, int elbow_phi, int linear_slide, double pitch, double roll, double claw, double wait) {
            this.x_value = x_value;
            this.y_value = y_value;
            this.angle = angle;
            this.elbow_phi = elbow_phi;
            this.linear_slide = linear_slide;
            this.pitch = pitch;
            this.roll = roll;
            this.claw = claw;
            this.wait = wait;

        }
    }

    public ArrayList<PositionInSpace> SetUpListOfThings(Telemetry telemetry, String filename) throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(filename));
        String line;
        br.readLine();
        while ((line = br.readLine()) != null) {
            String[] values = line.split(",");
            try {
                int x_value = Integer.parseInt(values[0].trim());
                int y_value = Integer.parseInt(values[1].trim());
                double angle = Double.parseDouble(values[2].trim());
                int elbow_phi = Integer.parseInt(values[3].trim());
                int linear_slide = Integer.parseInt(values[4].trim());
                double pitch = Double.parseDouble(values[5].trim());
                double roll = Double.parseDouble(values[6].trim());
                double claw = Double.parseDouble(values[7].trim());
                double wait = Double.parseDouble(values[8].trim());
                autoPath.add(new PositionInSpace(x_value, y_value, angle, elbow_phi, linear_slide, pitch, roll, claw, wait));
            } catch (NumberFormatException e) {
                telemetry.addData("Error", "Invalid number format in line: " + line);
                telemetry.update();
            } finally {
                telemetry.addData("The file listed has been read and parsed successfully. ", filename);
                telemetry.update();
            }
        }
        return autoPath;
    }

    public int getXFromList(PositionInSpace position) {
        return position.x_value;
    }

    public int getYFromList(PositionInSpace position) {
        return position.y_value;
    }

    public double getAngleFromList(PositionInSpace position) {
        return Math.toRadians(position.angle);
    }

    public int getElbowPhiFromList(PositionInSpace position) {
        return position.elbow_phi;
    }

    public int getLinearSlideFromList(PositionInSpace position) {
        return position.linear_slide;
    }

    public double getPitchFromList(PositionInSpace position) {
        return position.pitch;
    }

    public double getRollFromList(PositionInSpace position) {
        return position.roll;
    }

    public double getClawFromList(PositionInSpace position) {
        return position.claw;
    }

    public double getWaitFromList(PositionInSpace position) {
        return position.wait;
    }

}