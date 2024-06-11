package jp.jaxa.iss.kibo.rpc.defaultapk.untils;

import android.util.Log;
import android.util.Pair;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AreasItemData {
    private Map<Integer, List<Pair<String, Integer>>> areaVisionDataMap = areaVisionDataMap = new HashMap<>();;

    public AreasItemData() {
    }

    public void putVisionData(Integer areaNum, Pair<String, Integer> visionData) {
        List<Pair<String, Integer>> visionDataList = areaVisionDataMap.get(areaNum);
        if (visionDataList != null) {
            visionDataList.add(visionData);
        } else {
            List<Pair<String, Integer>> newVisionDataList = new ArrayList<>();
            newVisionDataList.add(visionData);
            areaVisionDataMap.put(areaNum, newVisionDataList);
        }
    }

    public Pair<String, Integer> getFinalAreaData(Integer areaNum) {
        Log.i("PairTEST",areaVisionDataMap.toString());
        Pair<String, Integer> itemData = getMaxFreqItemData(areaNum);

        return itemData;
    }

    private Pair<String, Integer> getMaxFreqItemData(Integer areaNum) {
        List<Pair<String, Integer>> visionDataList = areaVisionDataMap.get(areaNum);

        Map<String, Integer> frequencyMap = new HashMap<>();
        for (Pair<String, Integer> pair : visionDataList) {
            frequencyMap.put(pair.first, frequencyMap.getOrDefault(pair.first, 0) + 1);
        }

        String mostFrequentString = null;
        int maxFrequency = 0;
        for (Map.Entry<String, Integer> entry : frequencyMap.entrySet()) {
            if (entry.getValue() > maxFrequency) {
                maxFrequency = entry.getValue();
                mostFrequentString = entry.getKey();
            }
        }

        List<Pair<String, Integer>> filteredPairs = new ArrayList<>();
        for (Pair<String, Integer> pair : visionDataList) {
            if (pair.first.equals(mostFrequentString)) {
                filteredPairs.add(pair);
            }
        }

        Map<Integer, Integer> integerFrequencyMap = new HashMap<>();
        for (Pair<String, Integer> pair : filteredPairs) {
            integerFrequencyMap.put(pair.second, integerFrequencyMap.getOrDefault(pair.second, 0) + 1);
        }

        int mostFrequentInteger = 0;
        int maxIntegerFrequency = 0;
        for (Map.Entry<Integer, Integer> entry : integerFrequencyMap.entrySet()) {
            if (entry.getValue() > maxIntegerFrequency) {
                maxIntegerFrequency = entry.getValue();
                mostFrequentInteger = entry.getKey();
            }
        }

        return new Pair<>(mostFrequentString, mostFrequentInteger);
    }
}

