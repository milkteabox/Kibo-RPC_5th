package jp.jaxa.iss.kibo.rpc.defaultapk.untils;

import android.util.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gov.nasa.arc.astrobee.types.Point;
import jp.jaxa.iss.kibo.rpc.api.areas.AreaInfo;

public class AreasData {
    private Map<Integer, List<VisionData>> areaVisionDataMap = new HashMap<>();

    public AreasData() {
    }

    public void putVisionData(Integer areaNum, VisionData visionData) {
        List<VisionData> visionDataList = areaVisionDataMap.get(areaNum);
        if (visionDataList != null) {
            visionDataList.add(visionData);
        } else {
            List<VisionData> newVisionDataList = new ArrayList<>();
            newVisionDataList.add(visionData);
            areaVisionDataMap.put(areaNum, newVisionDataList);
        }
    }

    public AreaInfo[] getFinalAreaData() {
        AreaInfo[] areaInfos = new AreaInfo[5];
        for(int i = 1; i <= 4; i++){
            AreaInfo areaInfo = new AreaInfo();
            Pair<String, Integer> itemData = getMaxFreqItemData(i);

            areaInfo.setNumber(i);
            areaInfo.setItemName(itemData.first, itemData.second);

            areaInfos[i] = areaInfo;
        }
        return areaInfos;
    }

    private Pair<String, Integer> getMaxFreqItemData(Integer AreaNum) {
        List<VisionData> visionDataList = areaVisionDataMap.get(AreaNum);
        Map<String, Integer> itemNameCountMap = new HashMap<>();
        for (VisionData data : visionDataList) {
            String itemName = data.itemData.first;
            itemNameCountMap.put(itemName, itemNameCountMap.getOrDefault(itemName, 0) + 1);
        }

        String maxFreqItemName = null;
        int maxFreq = Integer.MIN_VALUE;
        for (Map.Entry<String, Integer> entry : itemNameCountMap.entrySet()) {
            if (entry.getValue() > maxFreq) {
                maxFreq = entry.getValue();
                maxFreqItemName = entry.getKey();
            }
        }

        Map<Integer, Integer> itemNumCountMap = new HashMap<>();
        for (VisionData data : visionDataList) {
            if (data.itemData.first.equals(maxFreqItemName)) {
                Integer itemNum = data.itemData.second;
                itemNumCountMap.put(itemNum, itemNumCountMap.getOrDefault(itemNum, 0) + 1);
            }
        }

        Integer maxFreqItemNum = null;
        maxFreq = Integer.MIN_VALUE;
        for (Map.Entry<Integer, Integer> entry : itemNumCountMap.entrySet()) {
            if (entry.getValue() > maxFreq) {
                maxFreq = entry.getValue();
                maxFreqItemNum = entry.getKey();
            }
        }

        return new Pair<>(maxFreqItemName, maxFreq);
    }

    public Point getAveragePoint(List<VisionData> visionDataList) {
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;

        for (VisionData d : visionDataList){
            sumX = d.arucoPoint.getX();
            sumY = d.arucoPoint.getY();
            sumZ = d.arucoPoint.getZ();
        }

        int l = visionDataList.size();
        return new Point(sumX/l, sumY/l, sumZ/l);
    }
}

