package frc.robot.Subsystems.Vision.PhotonVision;

import frc.robot.Subsystems.Vision.PhotonVision.ObjectCoordinates.Component;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ObjectRectangle {

  public enum CornerLocation {
    TopLeft,
    TopRight,
    BottomRight,
    BottomLeft
  }

  List<ObjectCoordinates> corners;

  public ObjectRectangle(ObjectCoordinates... coordinates) {
    corners = Arrays.asList(coordinates);
    corners = this.sortCorners();
  }

  public ObjectRectangle(double... coords) {
    corners = new ArrayList<>();
    int len = (coords.length < 8) ? coords.length : 8;
    for (int i = 0; i < len; i += 2) {
      corners.add(ObjectCoordinates.of(coords[i], coords[i + 1]));
    }
    corners = this.sortCorners();
  }

  // returns corners sorted in a clockwise fashion
  private List<ObjectCoordinates> sortCorners() {
    if (corners.size() != 4) return corners;

    final double[] center = {0, 0};
    for (ObjectCoordinates p : corners) {
      center[0] += p.getXCoord();
      center[1] += p.getYCoord();
    }
    double centerX = center[0] / corners.size();
    double centerY = center[1] / corners.size();

    List<ObjectCoordinates> sorted = new ArrayList<>(corners);
    sorted.sort(
        (p1, p2) -> {
          double angle1 = Math.atan2(p1.getYCoord() - centerY, p1.getXCoord() - centerX);
          double angle2 = Math.atan2(p2.getYCoord() - centerY, p2.getXCoord() - centerX);
          return Double.compare(angle1, angle2);
        });

    double crossProduct = cross(sorted.get(0), sorted.get(1), sorted.get(2));
    if (crossProduct > 0) { // positive means its counterclockwise
      Collections.reverse(sorted);
    }

    int startIndex = 0;
    for (int i = 1; i < sorted.size(); i++) {
      if (sorted.get(i).getYCoord() < sorted.get(startIndex).getYCoord()
          || (sorted.get(i).getYCoord() == sorted.get(startIndex).getYCoord()
              && sorted.get(i).getXCoord() < sorted.get(startIndex).getXCoord())) {
        startIndex = i;
      }
    }
    Collections.rotate(sorted, -startIndex);

    return sorted;
  }

  private double cross(ObjectCoordinates a, ObjectCoordinates b, ObjectCoordinates c) {
    double abx = b.getXCoord() - a.getXCoord();
    double aby = b.getYCoord() - a.getYCoord();
    double acx = c.getXCoord() - a.getXCoord();
    double acy = c.getYCoord() - a.getYCoord();
    return abx * acy - aby * acx;
  }

  public ObjectRectangle fromNumberArray(double[] arrayOfCoords) {
    return new ObjectRectangle(arrayOfCoords);
  }

  public double[] toNumberArray() {
    double[] arr = new double[corners.size() * 2];
    for (int i = 0; i < corners.size() * 2; i += 2) {
      arr[i / 2] = corners.get(i).getXCoord();
      arr[(i / 2) + 1] = corners.get(i).getYCoord();
    }
    return arr;
  }

  public ObjectCoordinates getCorner(CornerLocation location) {
    switch (location) {
      case TopLeft:
        return corners.get(0);
      case TopRight:
        return corners.get(1);
      case BottomLeft:
        return corners.get(2);
      case BottomRight:
        return corners.get(3);
    }
    return null;
  }

  public double getObjectPixelHeight() {
    return (this.getCorner(CornerLocation.TopLeft)
                .compareDistanceToCoordinates(this.getCorner(CornerLocation.BottomLeft))
            + this.getCorner(CornerLocation.TopRight)
                .compareDistanceToCoordinates(this.getCorner(CornerLocation.BottomRight)))
        / 2;
  }

  public double getObjectPixelWidth() {
    return (this.getCorner(CornerLocation.TopLeft)
                .compareDistanceToCoordinates(this.getCorner(CornerLocation.TopRight))
            + this.getCorner(CornerLocation.BottomLeft)
                .compareDistanceToCoordinates(this.getCorner(CornerLocation.BottomRight)))
        / 2;
  }

  public List<ObjectCoordinates> getCornerList() {
    return this.corners;
  }

  public double compareCorners(CornerLocation firstCorner, CornerLocation secondCorner) {
    return this.getCorner(firstCorner).compareDistanceToCoordinates(this.getCorner(secondCorner));
  }

  public double compareCorners(
      CornerLocation firstCorner, CornerLocation secondCorner, Component component) {
    return this.getCorner(firstCorner)
        .compareDistanceToComponent(this.getCorner(secondCorner), component);
  }
}
