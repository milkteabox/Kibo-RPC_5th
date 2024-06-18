package jp.jaxa.iss.kibo.rpc.defaultapk;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.defaultapk.untils.PointWithQuaternion;

import static jp.jaxa.iss.kibo.rpc.defaultapk.Constants.astronautPointwithQuaternion;

public class PathMap {
    List<PointWithQuaternion> scanPath = new ArrayList<>();

    public PathMap(){
        setUPPathMap();
    }

    private void setUPPathMap() {
        scanPath.add(0, new PointWithQuaternion(new Point(10.95, -10,5.195), new Quaternion(0f, 0f, 0.707f, 0.707f)));
        scanPath.add(1, new PointWithQuaternion(new Point(11.1,-9.4,5.1), new Quaternion(0f,0f,0f,1f)));
        scanPath.add(2, new PointWithQuaternion(new Point(11.0,-8.2,4.45), new Quaternion(0f,0f,0f,1f)));
        scanPath.add(3, new PointWithQuaternion(new Point(10.5,-7.5,4.5), new Quaternion(-0.5f,0.5f,-0.5f,0.5f)));
        scanPath.add(4, astronautPointwithQuaternion);
    }
}
