package jp.jaxa.iss.kibo.rpc.defaultapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.untils.Area;
import jp.jaxa.iss.kibo.rpc.defaultapk.untils.PointWithQuaternion;

public class Constants {

    public static final PointWithQuaternion astronautPointwithQuaternion =
            new PointWithQuaternion(new Point(11.143, -6.6707, 4.9654), new Quaternion(0f, 0f, 0.707f, 0.707f));

    public static final PointWithQuaternion scanPath_1 = new PointWithQuaternion(new Point(10.95, -10,5.195), new Quaternion(0f, 0f, 0.707f, 0.707f));
    public static final PointWithQuaternion scanPath_2 = new PointWithQuaternion(new Point(11.1,-9.4,5.1), new Quaternion(0f,0.707f,0f,0.707f));
    public static final PointWithQuaternion scanPath_3 = new PointWithQuaternion(new Point(11.0,-8.2,4.45), new Quaternion(0f,0.707f,0f,0.707f));
    public static final PointWithQuaternion scanPath_4 = new PointWithQuaternion(new Point(10.5,-7.5,4.5), new Quaternion(0f,0f,-1f,0f));
    public static final PointWithQuaternion scanPath_5 = new PointWithQuaternion(new Point(11.143, -6.6707, 4.9654), new Quaternion(0f,0f,-1f,0f));
}
