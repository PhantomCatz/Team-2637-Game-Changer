package frc.Autonomous;
public class CatzAutonomous
{
    public static void setAutonomouspath(double distance)
    {
        double errorThreshold = 0; //TBD
        double redPathADistance = 67.0;
        double bluePathADistance = 150.0;
        double redPathBDistance = 180.0;
        double bluePathBDistance = 60.0;
        

        if (redPathADistance - errorThreshold < distance && redPathADistance + errorThreshold > distance)
        {
            // run Red Path A   
        }
        else if (bluePathADistance - errorThreshold < distance && bluePathADistance + errorThreshold > distance)
        {
            // run Blue Path A
        }
        else if (redPathBDistance - errorThreshold < distance && redPathBDistance + errorThreshold > distance)
        {
            // run Red Path B
        }
        else if (bluePathBDistance - errorThreshold < distance && bluePathBDistance + errorThreshold > distance)
        {
            // run Blue Path B
        }
        
    }
}