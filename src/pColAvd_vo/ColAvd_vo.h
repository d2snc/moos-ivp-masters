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

 private: // State variables
   // Ownship state
   double m_nav_x;
   double m_nav_y;
   double m_nav_hdg;
   double m_nav_spd;

   // Contact information
   std::map<std::string, NodeRecord> m_contacts;

   // Collision cone visualization
   XYSegList m_collision_cone;
};

#endif 
