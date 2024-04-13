package jp.jaxa.iss.kibo.rpc.defaultapk;

import java.util.ArrayList;
import java.util.List;
import java.util.Stack;
import java.util.Vector;

import gov.nasa.arc.astrobee.types.Point;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

public class PathRunner {
    public static void RunWithAStar(Point startPoint, Poin endPoint){
        //TODO: Finish the A* Algorithm by code.
        List<Point> closed_points = new ArrayList<>();
        List<Point> open_points = new ArrayList<>();

        open_points.add(startPoint);

        while(!open_points.isEmpty()){

        }

    }

    private double distance(Point a, Point b){  // Using the Euclidean distance
        double result =
                sqrt(
                        pow(a.getX()-b.getX(),2) +
                                pow(a.getY()-b.getY(),2) + pow(a.getZ()-b.getZ(),2)
                );

        return result;
    }
}
