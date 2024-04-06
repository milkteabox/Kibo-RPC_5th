package jp.jaxa.iss.kibo.rpc.defaultapk.constants;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Constants {

    public static final Area area1 =
            new Area(new Point(10.42, -10.58, 4.82), new Point(11.48, -10.58, 5.57));

    public static final Area area2 =
            new Area(new Point(10.3, -9.25, 3.76203), new Point(11.55, -8.5, 3.76203));

    public static final Area area3 =
            new Area(new Point(10.3, -8.4, 3.76093), new Point(11.55, -7.45, 3.76093));

    public static final Area area4 =
            new Area(new Point(9.866984, 4.32, 4.32), new Point(9.866984, -6.365, 5.57));



    public static final Area koz1_1 =
            new Area(new Point(10.87, -9.5, 4.27), new Point(11.6, -9.45, 4.97));
    public static final Area koz1_2 =
            new Area(new Point(10.25, -9.5, 4.97), new Point(10.87, -9.45, 5.62));

    public static final Area koz2_1 =
            new Area(new Point(10.87, -8.5, 4.97), new Point(11.6, -8.45, 5.62));
    public static final Area koz2_2 =
            new Area(new Point(10.25, -8.5, 4.27), new Point(10.7, -8.45, 4.97));

    public static final Area koz3_1 =
            new Area(new Point(10.87, -7.40, 4.27), new Point(11.6, -7.35, 4.97));
    public static final Area koz3_2 =
            new Area(new Point(10.25, -7.40, 4.97), new Point(10.87, -7.35, 5.62));



    public static final Area kiz1 =
            new Area(new Point(10.3, -10.2, 4.32), new Point(11.55, -6.0, 5.57));
    public static final Area kiz2 =
            new Area(new Point(9.5, -10.5, 4.02), new Point(10.5, -9.6, 4.8));


    public static final PointWithQuaternion astronautPointwithQuaternion =
            new PointWithQuaternion(new Point(11.143, -6.6707, 4.9654), new Quaternion(1, 0, 0, 0));
}
