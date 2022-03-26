package frc.robot.utils;

public class Ball {
    private int id;
    private String color;
    private double centerX, centerY, radius, angle;
    public Ball(int id, String color, double centerX, double centerY, double radius, double angle) {
        this.id = id;
        this.color = color;
        this.centerX = centerX;
        this.centerY = centerY;
        this.radius = radius;
        this.angle = angle;
    }
    public String toString() {
        return String.format("ID:%s, COLOR:%s, CENTERX:%s, CENTERY:%s, RADIUS:%s", this.id, this.color, this.centerX, this.centerY, this.radius);
    }
    public int getId() {return id;}
    public String getColor() {return color;}
    public double getCenterX() {return centerX;}
    public double getCenterY() {return centerY;}
    public double getRadius() {return radius;}
    public double getAngle() {return angle;}
}
