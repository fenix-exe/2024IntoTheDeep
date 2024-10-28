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
        public double elbow_phi;
        public int linear_slide;
        public double wrist_psi;
        public double wrist_rho;
        public int intake;

        PositionInSpace(int x_value, int y_value, double angle, double elbow_phi, int linear_slide, double wrist_psi, double wrist_rho, int intake) {
            this.x_value = x_value;
            this.y_value = y_value;
            this.angle = angle;
            this.elbow_phi = elbow_phi;
            this.linear_slide = linear_slide;
            this.wrist_psi = wrist_psi;
            this.wrist_rho = wrist_rho;
            this.intake = intake;

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
                double elbow_phi = Double.parseDouble(values[3].trim());
                int linear_slide = Integer.parseInt(values[4].trim());
                double wrist_psi = Double.parseDouble(values[5].trim());
                double wrist_rho = Double.parseDouble(values[6].trim());
                int intake = Integer.parseInt(values[7].trim());
                autoPath.add(new PositionInSpace(x_value, y_value, angle, elbow_phi, linear_slide, wrist_psi, wrist_rho, intake));
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

    public double getElbowPhiFromList(PositionInSpace position) {
        return position.elbow_phi;
    }

    public int getLinearSlideFromList(PositionInSpace position) {
        return position.linear_slide;
    }

    public double getWristPsiFromList(PositionInSpace position) {
        return position.wrist_psi;
    }

    public double getWristRhoFromList(PositionInSpace position) {
        return position.wrist_rho;
    }

    public int getIntakeFromList(PositionInSpace position) {
        return position.intake;
    }

}