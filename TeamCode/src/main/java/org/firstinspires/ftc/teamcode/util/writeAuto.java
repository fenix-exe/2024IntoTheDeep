package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedInputStream;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;

public class writeAuto {
    String name;
    String filename;
    String storeLoc = "/sdcard/Download/autoLogger/";
    Date date = new Date();

    public writeAuto(String name) {
        this.name=name;
        filename = storeLoc + name + " - " + date.toString() + ".csv";
        File file = new File("filename");
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
