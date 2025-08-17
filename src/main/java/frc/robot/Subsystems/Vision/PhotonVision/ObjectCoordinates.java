package frc.robot.Subsystems.Vision.PhotonVision;

public class ObjectCoordinates {

  private double x;
  private double y;

  public enum Component {
    X,
    Y
  }

  public ObjectCoordinates(double x1, double y1) {
    this.x = x1;
    this.y = y1;
  }

  public static ObjectCoordinates of(double x1, double y1) {
    return new ObjectCoordinates(x1, y1);
  }

  public double getXCoord() {
    return x;
  }

  public double getYCoord() {
    return y;
  }

  public double getCoordComponent(Component component) {
    switch (component) {
      case X:
        return this.x;
      case Y:
        return this.y;
    }
    return 0;
  }

  public void setXCoord(double x1) {
    this.x = x1;
  }

  public void setYCoord(double y1) {
    this.y = y1;
  }

  public double compareDistanceToCoordinates(ObjectCoordinates objCoords) {
    return Math.sqrt(
        Math.pow(objCoords.getXCoord() - this.x, 2) + Math.pow(objCoords.getYCoord() - this.y, 2));
  }

  public double compareDistanceToComponent(
      ObjectCoordinates objCoords, Component componentToCompare) {
    switch (componentToCompare) {
      case X:
        return Math.abs(objCoords.getXCoord() - this.x);
      case Y:
        return Math.abs(objCoords.getYCoord() - this.y);
      default:
        return 0;
    }
  }
}
