#include "Arduino.h"


class TR_Coordinates
{
public:

    // Constructor
    explicit TR_Coordinates();
    
    void setENUOrigin(double lat_in,
                      double lon_in,
                      double alt_in);
    
    void setENUOriginECEF(double x_ecef,
                          double y_ecef,
                          double z_ecef);
                      
    void geodeticToECEF(double lat, 
                        double lon,
                        double alt,
                        double& x,
                        double& y,
                        double& z);
                        
    void ecefToGeodetic(double x, 
                        double y, 
                        double z,
                        double &lat,
                        double &lon,
                        double &alt);
    
    void ECEFToENU(double x, 
                   double y, 
                   double z, 
                   double& e, 
                   double& n, 
                   double& u);
                   
    void geodeticToENU(double lat, 
                       double lon,
                       double alt,
                       double& e,
                       double& n,
                       double& u);
                       
    void Quat2Euler(float quat[4], float euler[3]);

    static constexpr float RAD2DEG = 180.0f / PI;
    static constexpr float DEG2RAD = PI / 180.0f;

protected:

    // ENU origin in ECEF (m)
    double x0; 
    double y0;
    double z0;
    
    // ENU origin (radians/m)
    double lat0;
    double lon0;
    double alt0;

    double a;  // Semi-major axis
    double f;  // Flattening
    double e2; // Eccentricity squared
    


    
};

