package jp.jaxa.iss.kibo.rpc.defaultapk.untils;

import android.util.Pair;

import gov.nasa.arc.astrobee.types.Point;

public class VisionData {
    public final Point arucoPoint;
    public final Pair<String, Integer> itemData;

    public VisionData(Point arucoPoint, Pair<String, Integer> itemData) {
        this.arucoPoint = arucoPoint;
        this.itemData = itemData;
    }
}
