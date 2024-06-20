package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.untils.PointWithQuaternion;

class Constants {

    private static final double distanceToArea = 0.775;

    static final double targetZ_area23 = 3.76093 + distanceToArea;
    static final double targetY_area1 = -10.58 + distanceToArea;
    static final double targetX_area4 = 9.866984 + distanceToArea;


    static final PointWithQuaternion astronautPointwithQuaternion =
            new PointWithQuaternion(new Point(11.143, -6.6707, 4.9654), new Quaternion(0f, 0f, 0.707f, 0.707f));

    static final PointWithQuaternion scanPath_1 = new PointWithQuaternion(new Point(10.95, -10,5.195), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion scanPath_1_2 = new PointWithQuaternion(new Point(11.45, -9.7,5.1475), new Quaternion(0f, 0f, 0.707f, 0.707f));
    static final PointWithQuaternion scanPath_2 = new PointWithQuaternion(new Point(11.1,-9.4,5.1), new Quaternion(0f,0.707f,0f,0.707f));
    static final PointWithQuaternion scanPath_3 = new PointWithQuaternion(new Point(11.0,-8.2,4.45), new Quaternion(0f,0.707f,0f,0.707f));
    static final PointWithQuaternion scanPath_4 = new PointWithQuaternion(new Point(10.5,-7.5,4.5), new Quaternion(0f,0f,-1f,0f));
    static final PointWithQuaternion scanPath_5 = new PointWithQuaternion(new Point(11.143, -6.6707, 4.9654), new Quaternion(0f,0f,-1f,0f));

    static final PointWithQuaternion targetPQ_area4 = new PointWithQuaternion(new Point(targetX_area4, -6.8525, 4.945), new Quaternion(0f,0f,-1f,0f));
    static final PointWithQuaternion targetPQ_area3 = new PointWithQuaternion(new Point(10.925, -7.925, targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion targetPQ_area2 = new PointWithQuaternion(new Point(10.925, -8.875, targetZ_area23), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));
    static final PointWithQuaternion targetPQ_area1 = new PointWithQuaternion(new Point(10.95, targetY_area1, 5.195), new Quaternion(0f, 0f, -0.707f, 0.707f));
    static final PointWithQuaternion targetPQ_pathPoint2 = new PointWithQuaternion(new Point(11.1, -7.0, 5.2), new Quaternion(0.5f, 0.5f, -0.5f, 0.5f));

    static final PointWithQuaternion targetPQ_pathPoint1_1 = new PointWithQuaternion(new Point(10.5,-7.5,4.5),new Quaternion(0f, 0f, -0.707f, 0.707f));
    static final PointWithQuaternion targetPQ_pathPoint1_2 = new PointWithQuaternion(new Point(11.0,-8.2,4.45), new Quaternion(0f, 0f, -0.707f, 0.707f));
    static final PointWithQuaternion targetPQ_pathPoint1_3 = new PointWithQuaternion(new Point(11.1,-9.4,5.1), new Quaternion(0f, 0f, -0.707f, 0.707f));
}
