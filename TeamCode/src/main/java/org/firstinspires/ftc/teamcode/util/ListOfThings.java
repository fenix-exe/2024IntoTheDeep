package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class ListOfThings {
    ArrayList<PositionInSpace> listOfLists = new ArrayList<>();

    public class PositionInSpace {
        public int x_value;
        public int y_value;
        public double angle;

        PositionInSpace(int x_value, int y_value, double angle) {
            this.x_value = x_value;
            this.y_value = y_value;
            this.angle = angle;
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
                listOfLists.add(new PositionInSpace(x_value, y_value, angle));
            } catch (NumberFormatException e) {
                telemetry.addData("Error", "Invalid number format in line: " + line);
                telemetry.update();
            }
        }
        return listOfLists;
    }

    public int getXFromList(PositionInSpace position) {
        return position.x_value;
    }

    public int getYFromList(PositionInSpace position) {
        return position.y_value;
    }

    public double getAngleFromList(PositionInSpace position) {
        return position.angle;
    }
}