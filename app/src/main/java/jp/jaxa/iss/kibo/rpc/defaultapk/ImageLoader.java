package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class ImageLoader {

    public static Mat loadImage(Context context, String imgName) {
        int resID = context.getResources().getIdentifier(imgName, "drawable", context.getPackageName());
        Bitmap bitmap = BitmapFactory.decodeResource(context.getResources(), resID);
        Mat img = new Mat();
        Utils.bitmapToMat(bitmap, img);

        return img;
    }
}

