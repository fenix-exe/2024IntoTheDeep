package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedInputStream;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class writeAuto {
    String filename;
    public writeAuto(String filename) {
        this.filename=filename;
    }

    public void writeToFile(double x, double y, double heading, double elbow, double slide, double pitch, double roll, double claw) {
        FileWriter fw;

        try {
            fw = new FileWriter(filename,true);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        BufferedWriter bw= new BufferedWriter(fw);
        PrintWriter pw = new PrintWriter(bw);

        pw.println(x+","+y+","+heading+","+elbow+","+slide+","+pitch+","+roll+","+claw);
        pw.flush();
        pw.close();
    }
}
