package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightConfigs extends SubsystemBase{
    
    public NetworkTable Limelight(){
        return NetworkTableInstance.getDefault().getTable("limelight");
    }
    public boolean getHasTarget(){
        return Limelight().getEntry("tv").getDouble(0)==1;
    }
    public double getTagId(){
        return Limelight().getEntry("tid").getDouble(-1);
    }
    public double getTx(){
        return Limelight().getEntry("tx").getDouble(0.0);
    }
    public double getTy(){
        return Limelight().getEntry("ty").getDouble(0.0);
    }
    public double getTa(){
        return Limelight().getEntry("ta").getDouble(0.0);
    }
    public boolean setLedMode(int mode){
        return Limelight().getEntry("ledMode").setNumber(mode);
    }
    public double estimateDistance(){
        double tag_height = 1.0;
        double limelight_height = 0.5;
        double limelight_angle = 45;
        double y = getTy();

        if(!getHasTarget() || Double.isNaN(y)){
            return -1;
        }
        double angle_to_radians = Math.toRadians(limelight_angle + y);
        double height_difference = tag_height - limelight_height;
        double distance = height_difference/Math.tan(angle_to_radians);
return distance;
    }
    public double getTl(){
        return Limelight().getEntry("tl").getDouble(0.0);
    }
}