/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ContactSpawn.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ContactSpawn_HEADER
#define ContactSpawn_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class ContactSpawn : public AppCastingMOOSApp
{
 public:
   ContactSpawn();
   ~ContactSpawn();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void spawnContactWithParameters(double heading, double relative_bearing, double distance, double speed);
   void updateContactParameters(double heading, double relative_bearing, double distance, double speed);
   bool parseSpawnParameters(const std::string& params, double& heading, double& relative_bearing, double& distance, double& speed);
   void updateContact();
   void postNodeReport();
   void cleanContact();
   void spawnContatoTesteOnStartup();
   void updateContatoTesteForMovement();
   void spawnContatoTesteAtMidpoint();
   void spawnContatoTesteCrossing();

 private: // Configuration variables
  double m_spawn_distance;    // Distance ahead to spawn contact
  double m_contact_speed;     // Speed of spawned contact
  std::string m_contact_name; // Name of spawned contact
  std::string m_contact_type; // Type of spawned contact
  bool m_spawn_on_startup;    // Whether to spawn on startup
  double m_spawn_time;        // Time when spawned
  std::string m_spawn_trigger; // MOOS variable to trigger spawn

 private: // State variables
  bool m_contact_spawned;     // Whether contact has been spawned
  double m_ownship_x;         // Ownship position X
  double m_ownship_y;         // Ownship position Y
  double m_ownship_heading;   // Ownship heading
  double m_contact_x;         // Contact position X
  double m_contact_y;         // Contact position Y
  double m_contact_heading;   // Contact heading
  double m_last_update_time;  // Last time contact was updated
  double m_wpt_index;         // Current waypoint index of ownship
  std::string avoidance_mode; //Avoidance mode published
};

#endif 
