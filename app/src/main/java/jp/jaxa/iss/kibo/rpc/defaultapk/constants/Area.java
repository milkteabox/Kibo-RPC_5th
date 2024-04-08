package jp.jaxa.iss.kibo.rpc.defaultapk.constants;

import gov.nasa.arc.astrobee.types.Point;

public class Area {
    public final Point minPoint;
    public final Point maxPoint;

    public Area(Point minPoint, Point maxPoint) {
        this.minPoint = minPoint;
        this.maxPoint = maxPoint;
    }
}
