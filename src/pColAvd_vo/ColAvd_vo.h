/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ColAvd_vo.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ColAvd_vo_HEADER
#define ColAvd_vo_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYSegList.h"
#include "NodeRecord.h"
#include <map>

class ColAvd_vo : public AppCastingMOOSApp
{
 public:
   ColAvd_vo();
   ~ColAvd_vo();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   void computeCollisionCone();
   void visualizeCollisionCone();

 private: // Configuration variables
   double m_ship_radius;  // Radius for collision cone calculation
   double m_desired_speed;  // Desired speed (m/s)
   double m_max_speed;      // Maximum speed (m/s)
   double m_alpha_shift;    // Shift gain for VO
   double m_alpha_speed;    // Weight for speed cost
   double m_alpha_course;   // Weight for course cost
   double m_safety_radius;  // Safety distance (m)
   double m_collision_distance;  // Distance to trigger collision avoidance (m)

 private: // State variables
   // Ownship state
   double m_nav_x;
   double m_nav_y;
   double m_nav_hdg;
   double m_nav_spd;

   // Desired waypoint direction
   double m_desired_heading;

   // Contact information
   std::map<std::string, NodeRecord> m_contacts;

   // Collision avoidance outputs
   double m_avoidance_heading;
   double m_avoidance_speed;
   bool m_collision_detected;

   // Collision cone visualization
   XYSegList m_collision_cone;
};

#endif 
