package org.firstinspires.ftc.teamcode.auto.util;

import java.io.BufferedInputStream;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;

public class writeAuto {

    /*
    * These methods are used to write various helpful items from auto to a file
     */

    String name;
    String filename;
    String storeLoc = "/sdcard/Download/autoLogger/";
    Date date = new Date();

    public writeAuto(String name) {
        this.name=name;
        filename = storeLoc + name + " - " + date.toString() + ".csv";
        File file = new File("filename");
    }


    //this method  write a robot position to a file. unused
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

    //this method writers how long an auto sequence took to a file
    public void timer(double time) {
        FileWriter fw;

        try {
            fw = new FileWriter(filename,true);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        BufferedWriter bw= new BufferedWriter(fw);
        PrintWriter pw = new PrintWriter(bw);

        pw.println(time);
        pw.flush();
        pw.close();
    }

    //this methods writes an arbitrary string to a file.
    public void string(double string) {
        FileWriter fw;

        try {
            fw = new FileWriter(filename,true);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        BufferedWriter bw= new BufferedWriter(fw);
        PrintWriter pw = new PrintWriter(bw);

        pw.println(string);
        pw.flush();
        pw.close();
    }
}
