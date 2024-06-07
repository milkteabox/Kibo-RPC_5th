package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;
import android.util.Pair;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.tensorflow.lite.support.image.TensorImage;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.detector.Detection;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;

import java.io.IOException;
import java.util.List;

public class TFliteDetector {
    private ObjectDetector objectDetector;

    public TFliteDetector(Context context){

        ObjectDetector.ObjectDetectorOptions options = ObjectDetector.ObjectDetectorOptions
                .builder()
                .setScoreThreshold(0.3f)
                .setMaxResults(10)
                .build();

        try{
            objectDetector = ObjectDetector
                    .createFromFileAndOptions(context, "a.tflite", options);
        } catch (IOException e) {
            Log.i("TFLite", "Failed to load model: " + e.getMessage());
        } catch (Exception e) {
            Log.i("TFLite", "An unexpected error occurred: " + e.getMessage());
        }

    }

    public Pair<String, Integer> DetectFromMat(Mat mat){
        Bitmap bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, bitmap);

        TensorImage image = TensorImage.fromBitmap(bitmap);
        List<Detection> results = objectDetector.detect(image);
        if(results.isEmpty()){return null;}

        float highestScore = -1;
        String highestScoreLabel = null;
        for (Detection result : results) {
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
            for (Detection result : results) {
                Category category = result.getCategories().get(0);
                if (category.getLabel().equals(highestScoreLabel)) {
                    highestScoreLabelCount++;
                }
            }
        }

        return new Pair<>(highestScoreLabel,highestScoreLabelCount);
    }
}

