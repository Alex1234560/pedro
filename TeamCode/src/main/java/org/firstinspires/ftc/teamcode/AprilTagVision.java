package org.firstinspires.ftc.teamcode; // Make sure this matches your team's package name

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This class encapsulates all the logic for initializing and using the AprilTag processor.
 * It simplifies OpModes by hiding the complex setup of the VisionPortal and providing
 * easy-to-use methods for accessing detection data.
 */






public class AprilTagVision {
    public static int myExposure = 8;//20;
    public static int myGain = 100;//100;//150;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Optional: A variable to hold the most recent detection to avoid calling getDetections() multiple times.
    private AprilTagDetection latestDetection = null;



    /**
     * The constructor for the AprilTagVision class.
     * @param hardwareMap The hardware map from the OpMode, used to find the webcam.
     * @param webcamName The name of the webcam in your robot's configuration file.
     */
    public AprilTagVision(HardwareMap hardwareMap, String webcamName) {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                // Add any custom settings here, like tag family, units, etc.
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera from the hardware map.
        builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));

        // Optional: Set a custom camera resolution.
        // builder.setCameraResolution(new Size(640, 480));

        // Set and enable the processor.
        builder.addProcessor(aprilTag);




        // Build the Vision Portal.
        visionPortal = builder.build();


    }



    /**
     * This method should be called in every loop of your OpMode to update the detections.
     * It finds the first visible tag and stores it.
     * @return The first AprilTagDetection found, or null if no tags are visible.
     */
    public AprilTagDetection update() {

        setManualExposure(myExposure,myGain);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if (currentDetections.size() > 0) {
            // If tags are found, save the first one to our variable.
            latestDetection = currentDetections.get(0);
            return latestDetection;
        }

        // If no tags are found, clear the variable.
        latestDetection = null;

        return null;
    }

    /**
     * Checks if a tag is currently visible.
     * @return true if a tag was found in the last update(), false otherwise.
     */
    public boolean isTagVisible() {
        return latestDetection != null;
    }

    /**
     * Gets the range (distance) to the last detected tag.
     * @return The range in inches, or a default value (like -1) if no tag is visible.
     */
//    public double getRange() {
//        if (isTagVisible()) {
//            return latestDetection.ftcPose.range;
//        }
//        return -1; // Return an invalid value if no tag is seen
//    }
    public double getRange() {
        // Check if a tag is detected AND if its pose data is available
        if (latestDetection != null && latestDetection.ftcPose != null) {
            return latestDetection.ftcPose.range;
        }
        return -1; // Return an invalid/safe value if no tag or pose data is seen
    }

    /**
     * Gets the bearing (left/right angle) to the last detected tag.
     * @return The bearing in degrees, or 0 if no tag is visible.
     */
    public double getBearing() {
        // Check if the detection exists AND if the pose calculation is complete.
        if (latestDetection != null && latestDetection.ftcPose != null) {
            return latestDetection.ftcPose.bearing;
        }
        // Return a safe, invalid value if no tag or no pose data is available.
        return 999;
    }
    /**
     * Gets the yaw (rotational error) of the last detected tag.
     * @return The yaw in degrees, or 0 if no tag is visible.
     */
    public double getYaw() {
        if (isTagVisible()) {
            return latestDetection.ftcPose.yaw;
        }
        return 0;
    }

    // In AprilTagVision.java

// ... (after the getYaw() method) ...

    /**
     * Gets the lateral offset (left/right distance) to the last detected tag.
     * @return The X offset in inches. Positive is to the right of the camera. Returns 0 if no tag is visible.
     */
    public double getX() {
        if (isTagVisible()) {
            return latestDetection.ftcPose.x;
        }
        return 0;
    }

    /**
     * Gets the forward offset (straight-line distance) to the last detected tag.
     * @return The Y offset in inches. Returns 0 if no tag is visible.
     */
    public double getY() {
        if (isTagVisible()) {
            return latestDetection.ftcPose.y;
        }
        return 0;
    }

// ... (before the getID() method) ...


    /**
     * Gets the ID of the last detected tag.
     * @return The tag ID, or -1 if no tag is visible.
     */
    public int getID() {
        if (isTagVisible()) {
            return latestDetection.id;
        }
        return -1;
    }

    /**
     * Closes the vision portal to save resources. Call this at the end of your OpMode.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();


        }
    }
    /**
     * Sets the camera's manual exposure and gain in one call.
     * @param exposureMS The exposure time in milliseconds.
     * @param gain The camera gain.
     */
    public void setManualExposure(int exposureMS, int gain) {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return; // Exit the method if the camera is not ready.
        }
        // Stop if the Vision Portal isn't working.
        if (visionPortal == null) {
            return;
        }

        // Get the camera controls for exposure and gain
        org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl exposureControl = visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.class);
        org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl gainControl = visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl.class);

        // Tell the camera to use manual exposure
        exposureControl.setMode(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.Mode.Manual);

        // Set the exposure and gain values
        exposureControl.setExposure((long)exposureMS, java.util.concurrent.TimeUnit.MILLISECONDS);
        gainControl.setGain(gain);
    }

}

