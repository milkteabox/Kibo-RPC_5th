package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.RectF;
import android.util.Log;
import android.util.Pair;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

class TFliteDetector {
    private ObjectDetector objectDetector;

    TFliteDetector(Context context){

        ObjectDetector.ObjectDetectorOptions options = ObjectDetector.ObjectDetectorOptions
                .builder()
                .setScoreThreshold(0.415f)
                .setMaxResults(10)
                .build();

        try{
            objectDetector = ObjectDetector
                    .createFromFileAndOptions(context, "detect.tflite", options);
        } catch (IOException e) {
            Log.i("TFLite", "Failed to load model: " + e.getMessage());
        } catch (Exception e) {
            Log.i("TFLite", "An unexpected error occurred: " + e.getMessage());
        }
    }

    Pair<String, Integer> DetectFromMat(Mat mat){
        Bitmap bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, bitmap);

        TensorImage image = TensorImage.fromBitmap(bitmap);
        List<Detection> results = objectDetector.detect(image);
        if(results.isEmpty()){return null;}

        // Overlapping filtering using IoU
        float iouThreshold = 0.675f;
        List<Detection> filteredResults = new ArrayList<>();

        for (int i = 0; i < results.size(); i++) {
            Detection resultA = results.get(i);
            boolean keep = true;
            for (int j = 0; j < filteredResults.size(); j++) {
                Detection resultB = filteredResults.get(j);
                if (calculateIoU(resultA, resultB) > iouThreshold) {
                    keep = false;
                    break;
                }
            }
            if (keep) {
                filteredResults.add(resultA);
            }
        }

        float highestScore = -1;
        String highestScoreLabel = null;
        for (Detection result : filteredResults) {
            Category category = result.getCategories().get(0);
            String label = category.getLabel();
            float score = category.getScore();
            if (score > highestScore) {
                highestScore = score;
                highestScoreLabel = label;
            }
        }

        int highestScoreLabelCount = 0;
        if (highestScoreLabel != null){
            for (Detection result : filteredResults) {
                Category category = result.getCategories().get(0);
                if (category.getLabel().equals(highestScoreLabel)) {
                    highestScoreLabelCount++;
                }
            }
        }

        assert highestScoreLabel != null;
        highestScoreLabel = highestScoreLabel.replace("\r", "");
        return new Pair<>(highestScoreLabel, highestScoreLabelCount);
    }

    private float calculateIoU(Detection a, Detection b) {
        RectF boxA = a.getBoundingBox();
        RectF boxB = b.getBoundingBox();

        float xA = Math.max(boxA.left, boxB.left);
        float yA = Math.max(boxA.top, boxB.top);
        float xB = Math.min(boxA.right, boxB.right);
        float yB = Math.min(boxA.bottom, boxB.bottom);

        float interArea = Math.max(0, xB - xA) * Math.max(0, yB - yA);
        float boxAArea = (boxA.right - boxA.left) * (boxA.bottom - boxA.top);
        float boxBArea = (boxB.right - boxB.left) * (boxB.bottom - boxB.top);

        return interArea / (boxAArea + boxBArea - interArea);
    }
}

