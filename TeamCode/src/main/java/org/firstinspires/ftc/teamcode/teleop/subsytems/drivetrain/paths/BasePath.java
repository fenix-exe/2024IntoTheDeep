package org.firstinspires.ftc.teamcode.teleop.subsytems.drivetrain.paths;

import androidx.annotation.NonNull;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;

public abstract class BasePath {

     class PointsAsStrings {
        String X;
        String Y;
        public PointsAsStrings(String x, String y){
            X = x;
            Y = y;
        }
    }
    public ArrayList<PointsAsStrings> controlPoints  = new ArrayList<>();
    public PointsAsStrings endPointAsString;
    public String interpolationType;
    public double interpolationParam1;
    public double interpolationParam2;
    protected BasePath(){
    }
    public abstract PathChain getPathChain(Pose currentPose);
    public boolean closeToDestination(Pose currentPose){
        //point inside circle centered at endPoint and radius 10
        Point endPoint = convertToPoint(endPointAsString,currentPose);
        return (Math.pow((currentPose.getX() - endPoint.getX()),2) + Math.pow((currentPose.getY() - endPoint.getY()),2)) < 100;
    }
    public void parse(BasePath path, String[] csvValues){
        switch(csvValues[1]){
            case "ControlPoint":
                path.controlPoints.add(new PointsAsStrings(csvValues[2], csvValues[3]));
                break;
            case "EndPoint":
                path.endPointAsString = new PointsAsStrings(csvValues[2], csvValues[3]);
                break;
            case "InterpolationType":
                path.interpolationType = csvValues[2];
                break;
            case "InterpolationParam1":
                path.interpolationParam1 = Double.parseDouble(csvValues[2]);
                break;
            case "InterpolationParam2":
                path.interpolationParam2 = Double.parseDouble(csvValues[2]);
                break;
        }
    }

    public Point convertToPoint(PointsAsStrings pointString, Pose currentPose) {
        double xCoord;
        double yCoord;
        if (pointString.X.equals("currentX")){
            xCoord = currentPose.getX();
        } else {
            xCoord = Double.parseDouble(pointString.X);
        }
        if (pointString.Y.equals("currentY")){
            yCoord = currentPose.getY();
        }else {
            yCoord = Double.parseDouble(pointString.Y);
        }
        return new Point(xCoord, yCoord, Point.CARTESIAN);
    }

    public void clear() {
        controlPoints.clear();
    }

    @NonNull
    public String toString(){
        StringBuilder strBuilder = new StringBuilder();
        for(int i=0; i< controlPoints.size(); i++) {
            strBuilder.append("("+controlPoints.get(i).X+","+controlPoints.get(i).Y+") ");
        }
        strBuilder.append("("+endPointAsString.X+","+endPointAsString.Y+")");
        strBuilder.append("["+interpolationParam1+","+interpolationParam2+"]");
        return strBuilder.toString();
    }
}
