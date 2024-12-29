package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.util.ArrayList;

public class extractAuto {
    ArrayList<PositionInSpace> autoPath = new ArrayList<>();

    public class PositionInSpace {
        public double x_value;
        public double y_value;
        public double angle;
        public double elbow_phi;
        public double elbow_speed;
        public double linear_slide;
        public double pitch;
        public double roll;
        public double claw;
        public double wait;

        PositionInSpace(double x_value, double y_value, double angle, double elbow_phi, double elbow_speed, double linear_slide, double pitch, double roll, double claw, double wait) {
            this.x_value = x_value;
            this.y_value = y_value;
            this.angle = angle;
            this.elbow_phi = elbow_phi;
            this.elbow_speed = elbow_speed;
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
                double x_value = Double.parseDouble(values[0].trim());
                double y_value = Double.parseDouble(values[1].trim());
                double angle = Double.parseDouble(values[2].trim());
                double elbow_phi = Double.parseDouble(values[3].trim());
                double elbow_speed = Double.parseDouble(values[4].trim());
                double linear_slide = Double.parseDouble(values[5].trim());
                double pitch = Double.parseDouble(values[6].trim());
                double roll = Double.parseDouble(values[7].trim());
                double claw = Double.parseDouble(values[8].trim());
                double wait = Double.parseDouble(values[9].trim());
                autoPath.add(new PositionInSpace(x_value, y_value, angle, elbow_phi, elbow_speed,linear_slide, pitch, roll, claw, wait));
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

    public double getXFromList(PositionInSpace position) {
        return position.x_value;
    }

    public double getYFromList(PositionInSpace position) {
        return position.y_value;
    }

    public double getAngleFromList(PositionInSpace position) {
        return Math.toRadians(position.angle);
    }

    public double getElbowPhiFromList(PositionInSpace position) {
        return position.elbow_phi;
    }

    public double getLinearSlideFromList(PositionInSpace position) {
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

    public double getElbowSpeedFromList(PositionInSpace position) {
        return position.elbow_speed;
    }


}