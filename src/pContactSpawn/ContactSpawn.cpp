/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ContactSpawn.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "ContactSpawn.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ContactSpawn::ContactSpawn()
{
  // Configuration variables
  m_spawn_distance = 500.0;    // 500 meters ahead
  m_contact_speed = 5.0;       // 5 m/s
  m_contact_name = "target";   // Default contact name
  m_contact_type = "ship";     // Default contact type
  m_spawn_on_startup = false;  // Don't spawn on startup by default
  m_spawn_time = 0.0;
  m_spawn_trigger = "SPAWN_HEADON"; // Default trigger variable

  // State variables
  m_contact_spawned = false;
  m_ownship_x = 0.0;
  m_ownship_y = 0.0;
  m_ownship_heading = 0.0;
  m_contact_x = 0.0;
  m_contact_y = 0.0;
  m_contact_heading = 0.0;
  m_last_update_time = 0.0;
}

//---------------------------------------------------------
// Destructor

ContactSpawn::~ContactSpawn()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ContactSpawn::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NAV_X") {
       m_ownship_x = msg.GetDouble();
     }
     else if(key == "NAV_Y") {
       m_ownship_y = msg.GetDouble();
     }
     else if(key == "NAV_HEADING") {
       m_ownship_heading = msg.GetDouble();
     }
     else if(key == m_spawn_trigger) {
       if(!m_contact_spawned) {
         spawnContact();
       }
     }
     else if(key == "SPAWN_CLEAN") {
       if(m_contact_spawned) {
         cleanContact();
       }
     }
     else if(key == "SPAWN_OVERTAKING") {
       if(!m_contact_spawned) {
         spawnOvertakingContact();
       }
     }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ContactSpawn::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ContactSpawn::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  // If spawn on startup and not yet spawned, spawn contact
  if(m_spawn_on_startup && !m_contact_spawned) {
    spawnContact();
  }
  
  // If contact is spawned, update its position
  if(m_contact_spawned) {
    updateContact();
  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ContactSpawn::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "spawn_distance") {
      m_spawn_distance = atof(value.c_str());
      handled = true;
    }
    else if(param == "contact_speed") {
      m_contact_speed = atof(value.c_str());
      handled = true;
    }
    else if(param == "contact_name") {
      m_contact_name = value;
      handled = true;
    }
    else if(param == "contact_type") {
      m_contact_type = value;
      handled = true;
    }
    else if(param == "spawn_on_startup") {
      m_spawn_on_startup = (tolower(value) == "true");
      handled = true;
    }
    else if(param == "spawn_trigger") {
      m_spawn_trigger = value;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ContactSpawn::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register(m_spawn_trigger, 0);
  Register("SPAWN_CLEAN", 0);
  Register("SPAWN_OVERTAKING", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ContactSpawn::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: pContactSpawn                        " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Config | Value | State | Value";
  actab.addHeaderLines();
  actab << "Spawn Distance" << doubleToString(m_spawn_distance, 1) << "Contact Spawned" << (m_contact_spawned ? "Yes" : "No");
  actab << "Contact Speed" << doubleToString(m_contact_speed, 1) << "Ownship X" << doubleToString(m_ownship_x, 1);
  actab << "Contact Name" << m_contact_name << "Ownship Y" << doubleToString(m_ownship_y, 1);
  actab << "Contact Type" << m_contact_type << "Ownship Hdg" << doubleToString(m_ownship_heading, 1);
  actab << "Spawn on Startup" << (m_spawn_on_startup ? "Yes" : "No") << "Contact X" << doubleToString(m_contact_x, 1);
  actab << "Spawn Trigger" << m_spawn_trigger << "Contact Y" << doubleToString(m_contact_y, 1);
  m_msgs << actab.getFormattedString();

  return(true);
}

//---------------------------------------------------------
// Procedure: spawnContact()
//            Creates a simulated contact heading towards ownship

void ContactSpawn::spawnContact()
{
  if(m_contact_spawned) return;
  
  // Calculate spawn position ahead of ownship
  double heading_rad = m_ownship_heading * M_PI / 180.0;
  double spawn_x = m_ownship_x + m_spawn_distance * sin(heading_rad);
  double spawn_y = m_ownship_y + m_spawn_distance * cos(heading_rad);
  
  // Set contact position
  m_contact_x = spawn_x;
  m_contact_y = spawn_y;
  
  // Set contact heading to point towards ownship (opposite of ownship heading)
  m_contact_heading = fmod(m_ownship_heading + 180.0, 360.0);
  if(m_contact_heading < 0) m_contact_heading += 360.0;
  
  // Mark as spawned
  m_contact_spawned = true;
  m_spawn_time = MOOSTime();
  m_last_update_time = MOOSTime();
  
  // Post initial NODE_REPORT
  postNodeReport();
  
  reportEvent("Contact spawned: " + m_contact_name + " at (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ")");
}

//---------------------------------------------------------
// Procedure: updateContact()
//            Updates contact position and posts NODE_REPORT

void ContactSpawn::updateContact()
{
  if(!m_contact_spawned) return;
  
  double current_time = MOOSTime();
  double delta_time = current_time - m_last_update_time;
  
  if(delta_time <= 0) return;
  
  // Update contact position based on speed and heading
  double heading_rad = m_contact_heading * M_PI / 180.0;
  double delta_x = m_contact_speed * delta_time * sin(heading_rad);
  double delta_y = m_contact_speed * delta_time * cos(heading_rad);
  
  m_contact_x += delta_x;
  m_contact_y += delta_y;
  
  // Post updated NODE_REPORT
  postNodeReport();
  
  m_last_update_time = current_time;
}

//---------------------------------------------------------
// Procedure: postNodeReport()
//            Posts NODE_REPORT for the simulated contact

void ContactSpawn::postNodeReport()
{
  if(!m_contact_spawned) return;
  
  string node_report = "NAME=" + m_contact_name + ",";
  node_report += "TYPE=" + m_contact_type + ",";
  node_report += "TIME=" + doubleToString(MOOSTime(), 2) + ",";
  node_report += "X=" + doubleToString(m_contact_x, 2) + ",";
  node_report += "Y=" + doubleToString(m_contact_y, 2) + ",";
  node_report += "SPD=" + doubleToString(m_contact_speed, 2) + ",";
  node_report += "HDG=" + doubleToString(m_contact_heading, 2) + ",";
  node_report += "YAW=" + doubleToString(m_contact_heading, 2) + ",";
  node_report += "DEPTH=0.0,";
  node_report += "LENGTH=6.0,";
  node_report += "MODE=SPAWNED";
  
  Notify("NODE_REPORT", node_report);
}

//---------------------------------------------------------
// Procedure: cleanContact()
//            Removes the simulated contact
void ContactSpawn::cleanContact()
{
  if(!m_contact_spawned) return;
  
  // Mark as not spawned
  m_contact_spawned = false;
  
  // Reset contact position
  m_contact_x = 0.0;
  m_contact_y = 0.0;
  m_contact_heading = 0.0;
  
  // Post NODE_REPORT to remove contact
  string node_report = "NAME=" + m_contact_name + ",";
  node_report += "TYPE=" + m_contact_type + ",";
  node_report += "TIME=" + doubleToString(MOOSTime(), 2) + ",";
  node_report += "X=" + doubleToString(m_contact_x, 2) + ",";
  node_report += "Y=" + doubleToString(m_contact_y, 2) + ",";
  node_report += "SPD=0.0,";
  node_report += "HDG=0.0,";
  node_report += "YAW=0.0,";
  node_report += "DEPTH=0.0,";
  node_report += "LENGTH=0.0,";
  node_report += "MODE=REMOVED";
  
  Notify("NODE_REPORT", node_report);
  
  reportEvent("Contact removed: " + m_contact_name);
}

//---------------------------------------------------------
// Procedure: spawnOvertakingContact()
//            Creates a simulated contact with same heading but slower speed
void ContactSpawn::spawnOvertakingContact()
{
  if(m_contact_spawned) return;
  
  // Calculate spawn position 200m ahead of ownship
  double heading_rad = m_ownship_heading * M_PI / 180.0;
  double spawn_x = m_ownship_x + 200.0 * sin(heading_rad);
  double spawn_y = m_ownship_y + 200.0 * cos(heading_rad);
  
  // Set contact position
  m_contact_x = spawn_x;
  m_contact_y = spawn_y;
  
  // Set contact heading same as ownship (same direction)
  m_contact_heading = m_ownship_heading;
  
  // Override speed to 2 m/s for overtaking scenario
  double original_speed = m_contact_speed;
  m_contact_speed = 2.0;
  
  // Mark as spawned
  m_contact_spawned = true;
  m_spawn_time = MOOSTime();
  m_last_update_time = MOOSTime();
  
  // Post initial NODE_REPORT
  postNodeReport();
  
  reportEvent("Overtaking contact spawned: " + m_contact_name + " at (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ") with speed " + 
              doubleToString(m_contact_speed, 1));
}


