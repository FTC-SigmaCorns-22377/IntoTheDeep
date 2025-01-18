package sigmacorns.common.subsystems.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;

import java.util.ArrayList;
import java.util.List;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

public class ImageProcessing {
    // inputs: hsv image, masking color
    // outputs: masked image
    public static Mat mask(Mat hsvImage, String color) {
        Mat hsv = hsvImage;

        Scalar yLower = new Scalar(20, 100, 125);
        Scalar yUpper = new Scalar(40, 255, 255);
        Scalar bLower = new Scalar(100, 100, 100);  // Lower bound for blue (Hue around 100-120)
        Scalar bUpper = new Scalar(140, 255, 255);  // Upper bound for blue (Hue around 140)

        Scalar rLower1 = new Scalar(0, 100, 100);  // Lower bound of red in HSV (near 0 for dark red)
        Scalar rUpper1 = new Scalar(10, 255, 255); // Upper bound for this range

        // Upper red (Hue close to 180 for bright red)
        Scalar rLower2 = new Scalar(170, 100, 100);  // Lower bound for this range (near 180)
        Scalar rUpper2 = new Scalar(180, 255, 255);  // Upper bound for this range

        Mat mask = new Mat();
        if (color.equals("red")) {
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, rLower1, rUpper1, mask1);
            Core.inRange(hsv, rLower2, rUpper2, mask2);
            Core.bitwise_or(mask1, mask2, mask);
        }
        if (color.equals("yellow")) {
            Core.inRange(hsv, yLower, yUpper, mask);
        }
        if (color.equals("blue")) {
            Core.inRange(hsv, bLower, bUpper, mask);
        }

        return mask;
    }

    // inputs: masked image
    // outputs: list of rectangles
    public static List<List<RotatedRect>> createBoundingRectangles(Mat hsv) {
        Mat maskRed = processMask(mask(hsv, "red"));
        Mat maskYellow = processMask(mask(hsv, "yellow"));
        Mat maskBlue = processMask(mask(hsv, "blue"));

        List<RotatedRect> redRectArr = new ArrayList<>();
        List<RotatedRect> yellowRectArr = new ArrayList<>();
        List<RotatedRect> blueRectArr = new ArrayList<>();

        List<MatOfPoint> redContours = new ArrayList<>();
        List<MatOfPoint> blueContours = new ArrayList<>();
        List<MatOfPoint> yellowContours = new ArrayList<>();
        Mat redHierarchy = new Mat();
        Mat blueHierarchy = new Mat();
        Mat yellowHierarchy = new Mat();

        Imgproc.findContours(maskRed, redContours, redHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskBlue, blueContours, blueHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskYellow, yellowContours, yellowHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint cnt : redContours) {
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            RotatedRect redRect = Imgproc.minAreaRect(cnt2f);
            redRectArr.add(redRect);
        }

        for (MatOfPoint cnt : yellowContours) {
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            RotatedRect yellowRect = Imgproc.minAreaRect(cnt2f);
            yellowRectArr.add(yellowRect);
        }

        for (MatOfPoint cnt : blueContours) {
            MatOfPoint2f cnt2f = new MatOfPoint2f(cnt.toArray());
            RotatedRect blueRect = Imgproc.minAreaRect(cnt2f);
            blueRectArr.add(blueRect);
        }

        List<List<RotatedRect>> rectArr = new ArrayList<>();
        rectArr.add(redRectArr);
        rectArr.add(yellowRectArr);
        rectArr.add(blueRectArr);

        return rectArr;
    }

    // inputs: list of rectangles
    // outputs: rectangle closest to the center of the frame (or to claw, once i figure out the dimensions)
    public static RotatedRect[] closestRects(List<List<RotatedRect>> rects, int width, int height) {
        double centerWidth = width / 2.0;
        double centerHeight = height / 2.0;

        double minRedDistance = Double.MAX_VALUE;
        double minYellowDistance = Double.MAX_VALUE;
        double minBlueDistance = Double.MAX_VALUE;

        RotatedRect closestRedRect = null;
        RotatedRect closestYellowRect = null;
        RotatedRect closestBlueRect = null;

        for (RotatedRect redRect : rects.get(0)) {
            double distance = Math.sqrt(Math.pow(redRect.center.x - centerWidth, 2) +
                    Math.pow(redRect.center.y - centerHeight, 2));
            if (distance < minRedDistance) {
                minRedDistance = distance;
                closestRedRect = redRect;
            }
        }

        for (RotatedRect yellowRect : rects.get(0)) {
            double distance = Math.sqrt(Math.pow(yellowRect.center.x - centerWidth, 2) +
                    Math.pow(yellowRect.center.y - centerHeight, 2));
            if (distance < minYellowDistance) {
                minYellowDistance = distance;
                closestYellowRect = yellowRect;
            }
        }

        for (RotatedRect blueRect : rects.get(0)) {
            double distance = Math.sqrt(Math.pow(blueRect.center.x - centerWidth, 2) +
                    Math.pow(blueRect.center.y - centerHeight, 2));
            if (distance < minBlueDistance) {
                minBlueDistance = distance;
                closestBlueRect = blueRect;
            }
        }

        return new RotatedRect[]{closestRedRect, closestYellowRect, closestBlueRect};
    }

    public static double rectAngle(RotatedRect rect){
        if (rect == null)
        {
            return 0.0;
        }

        double angle = rect.angle;

        if (rect.size.width < rect.size.height)
        {
            return angle;
        }

        return angle - 90;
    }

    public static Mat applyDilation(Mat mask, double size)
    {
        double dilationSize = size;

        Mat element = Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT,
                new Size(2 * dilationSize + 1, 2 * dilationSize + 1),
                new Point(dilationSize, dilationSize));

        Mat dilatedMask = new Mat();

        Imgproc.dilate(mask, dilatedMask, element);

        return dilatedMask;
    }

    public static Mat applyErosion(Mat mask, double size)
    {
        double erosionSize = size;

        Mat element = Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT,
                new Size(2 * erosionSize + 1, 2 * erosionSize + 1),
                new Point(erosionSize, erosionSize));

        Mat erodedMask = new Mat();

        Imgproc.erode(mask, erodedMask, element);

        return erodedMask;
    }

    // Placeholder for processMask method
    public static Mat processMask(Mat mask) {
        // Implement the processing of the mask as needed
        return applyDilation(applyErosion(applyDilation(applyErosion(mask, 4), 6), 4), 2); //she dilate on my eroded dilated eroded mask :fire: :fire:
    }

    public static void main(String[] args) {
        // Load OpenCV native library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Open video capture (1 for secondary camera, adjust as needed)
        VideoCapture cap = new VideoCapture(1);
        if (!cap.isOpened()) {
            System.out.println("Error: Cannot open camera");
            return;
        }

        // Set up Swing components for display
        JFrame frame = new JFrame("OpenCV Example");
        JLabel imageLabel = new JLabel();
        frame.getContentPane().add(imageLabel, BorderLayout.CENTER);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(800, 600);
        frame.setVisible(true);

        Mat frameMat = new Mat();

        while (true) {
            // Capture a frame
            if (!cap.read(frameMat)) {
                System.out.println("Error: Cannot read frame");
                break;
            }

            // Convert frame to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(frameMat, hsv, Imgproc.COLOR_BGR2HSV);

            // Display masks and rectangles
            Mat maskRed = mask(hsv, "red");
            Mat maskYellow = mask(hsv, "yellow");
            Mat maskBlue = mask(hsv, "blue");

            Mat rectanglesFrame = drawBoundingRectangles(frameMat, createBoundingRectangles(hsv));

            // Show the rectangles frame in the Swing window
            imageLabel.setIcon(new ImageIcon(matToBufferedImage(rectanglesFrame)));
            frame.repaint();

            // Print rectAngle of the closest rectangle
            System.out.println(rectAngle(closestRects(createBoundingRectangles(hsv)).get(0)));

            // Break the loop if 'q' is pressed
            if ((char) System.in.read() == 'q') {
                break;
            }
        }

        // Release resources
        cap.release();
            frame.dispose();
        }

        // Convert Mat to BufferedImage for display
        private static BufferedImage matToBufferedImage(Mat mat) {
            int type = (mat.channels() > 1) ? BufferedImage.TYPE_3BYTE_BGR : BufferedImage.TYPE_BYTE_GRAY;
            BufferedImage image = new BufferedImage(mat.cols(), mat.rows(), type);
            mat.get(0, 0, ((DataBufferByte) image.getRaster().getDataBuffer()).getData());
            return image;
        }
}