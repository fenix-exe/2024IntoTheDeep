package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import androidx.annotation.NonNull;

public class writeAuto {
    String fileName;
    public writeAuto(String filename) {
        this.fileName = filename;
    }

    public void writeToFile(int elbow_phi, int linear_slide, double wrist_psi, double wrist_rho) {
        // write to file
        FileWriter fw;
        try {
            fw = new FileWriter(fileName, false);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        BufferedWriter bw = new BufferedWriter(fw);
        PrintWriter pw = new PrintWriter(bw);

        pw.println(elbow_phi+","+ linear_slide+","+wrist_psi+","+ wrist_rho);
        pw.flush();
        pw.close();
    }

    public class savePosition implements Action {
        int elbow_phi;
        int linearSlide;
        double wrist_psi;
        double wrist_rho;

        public savePosition(int elbow, int linear, double wristp, double wristr) {
            this.elbow_phi = elbow;
            this.linearSlide = linear;
            this.wrist_psi = wristp;
            this.wrist_rho = wristr;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            writeToFile(elbow_phi,linearSlide,wrist_psi,wrist_rho);
            return false;
        }
    }

    public Action savePosition(int elbow, int linear, double wristp, double wristr) {
        return new savePosition(elbow,linear,wristp, wristr);
    }
}

