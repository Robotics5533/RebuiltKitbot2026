package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class MathUtil {
    /**
     * Creates a bounding box pair representing opposite corners of a rectangle
     * 
     * @param center Center position of the obstacle
     * @param width  Width of the obstacle (X dimension)
     * @param depth  Depth of the obstacle (Y dimension)
     * @return Pair of Translation2d representing opposite corners
     */
    public static Pair<Translation2d, Translation2d> createBoundingBox(
            Translation2d center, double width, double depth) {

        Translation2d corner1 = new Translation2d(
                center.getX() - width / 2.0,
                center.getY() - depth / 2.0);

        Translation2d corner2 = new Translation2d(
                center.getX() + width / 2.0,
                center.getY() + depth / 2.0);

        return new Pair<>(corner1, corner2);
    }
}
