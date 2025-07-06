/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ColAvd.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ColAvd_HEADER
#define ColAvd_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class ColAvd : public AppCastingMOOSApp
{
 public:
   ColAvd();
   ~ColAvd();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void parseNodeReport(const std::string& node_report);

 private: // Configuration variables

 private: // State variables
  double m_nav_heading;
  double m_nav_speed;
  double m_nav_x;
  double m_nav_y;
  
  double m_contact_x;
  double m_contact_y;
  double m_contact_heading;
  double m_contact_speed;
  double m_contact_distance;
  double m_col_avd_distance;
};

#endif 
