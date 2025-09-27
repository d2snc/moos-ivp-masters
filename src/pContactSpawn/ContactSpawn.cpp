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
  m_wpt_index = 0;
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
     else if(key == "SPAWN_CONTACT") {
       string sval = msg.GetString();
       double heading, relative_bearing, distance, speed;
       if(parseSpawnParameters(sval, heading, relative_bearing, distance, speed)) {
         if(!m_contact_spawned) {
           spawnContactWithParameters(heading, relative_bearing, distance, speed);
         } else {
           updateContactParameters(heading, relative_bearing, distance, speed);
         }
       } else {
         reportRunWarning("Invalid SPAWN_CONTACT parameters: " + sval);
       }
     }
     else if(key == "SPAWN_CLEAN") {
       if(m_contact_spawned) {
         cleanContact();
       }
     }
     else if(key == "WPT_INDEX") {
       m_wpt_index = msg.GetDouble();
       
       // Quando WPT_INDEX for 1, atualizar CONTATO_TESTE para movimento
       if(m_wpt_index == 1 && m_contact_spawned) {
         updateContatoTesteForMovement();
       }
       // Quando WPT_INDEX for 2, spawnar CONTATO_TESTE no ponto médio
       else if(m_wpt_index == 2) {
         spawnContatoTesteAtMidpoint();
       }
       // Quando WPT_INDEX for 3, spawnar CONTATO_TESTE cruzando
       else if(m_wpt_index == 3) {
         spawnContatoTesteCrossing();
       }
     }
     else if (key == "AVOIDANCE_MODE")
      avoidance_mode = msg.GetString();
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
  
  // Spawn on startup functionality removed - now only via SPAWN_CONTACT command
  
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
  

  // Spawn CONTATO_TESTE on startup at specified position with zero speed
  // Não lembro direito pq coloquei isso, mas depois vi que não precisava
  //spawnContatoTesteOnStartup();
  
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
  Register("SPAWN_CONTACT", 0);
  Register("SPAWN_CLEAN", 0);
  Register("WPT_INDEX", 0);
  Register("AVOIDANCE_MODE", 0);
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
  node_report += "LENGTH=10.0,";
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
// Procedure: parseSpawnParameters()
//            Parses parameters from SPAWN_CONTACT string
//            Format: "heading=RUMO,relative_bearing=MARCACAORELATIVA,distance=DISTANCIA,speed=VELOCIDADE"
bool ContactSpawn::parseSpawnParameters(const string& params, double& heading, double& relative_bearing, double& distance, double& speed)
{
  // Initialize values
  heading = 0.0;
  relative_bearing = 0.0;
  distance = 0.0;
  speed = 0.0;
  
  // Split parameters by comma
  vector<string> kvpairs = parseString(params, ',');
  
  for(unsigned int i = 0; i < kvpairs.size(); i++) {
    string kvpair = kvpairs[i];
    vector<string> parts = parseString(kvpair, '=');
    
    if(parts.size() != 2) continue;
    
    string key = tolower(stripBlankEnds(parts[0]));
    string value = stripBlankEnds(parts[1]);
    
    if(key == "heading") {
      heading = atof(value.c_str());
    }
    else if(key == "relative_bearing") {
      relative_bearing = atof(value.c_str());
    }
    else if(key == "distance") {
      distance = atof(value.c_str());
    }
    else if(key == "speed") {
      speed = atof(value.c_str());
    }
  }
  
  // Validate parameters
  if(distance <= 0 || speed < 0) {
    return false;
  }
  
  return true;
}

//---------------------------------------------------------
// Procedure: spawnContactWithParameters()
//            Creates a simulated contact with specified parameters
void ContactSpawn::spawnContactWithParameters(double heading, double relative_bearing, double distance, double speed)
{
  if(m_contact_spawned) return;
  
  // Convert relative bearing to absolute bearing
  double absolute_bearing = m_ownship_heading + relative_bearing;
  if(absolute_bearing >= 360.0) absolute_bearing -= 360.0;
  if(absolute_bearing < 0.0) absolute_bearing += 360.0;
  
  // Calculate spawn position at specified distance on the relative bearing
  double bearing_rad = absolute_bearing * M_PI / 180.0;
  double spawn_x = m_ownship_x + distance * sin(bearing_rad);
  double spawn_y = m_ownship_y + distance * cos(bearing_rad);
  
  // Set contact position
  m_contact_x = spawn_x;
  m_contact_y = spawn_y;
  
  // Set contact heading
  m_contact_heading = heading;
  
  // Set contact speed
  m_contact_speed = speed;
  
  // Mark as spawned
  m_contact_spawned = true;
  m_spawn_time = MOOSTime();
  m_last_update_time = MOOSTime();
  
  // Post initial NODE_REPORT
  postNodeReport();
  
  reportEvent("Contact spawned: " + m_contact_name + " at (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ") heading " + 
              doubleToString(m_contact_heading, 1) + "° at " + 
              doubleToString(m_contact_speed, 1) + " m/s, distance " + 
              doubleToString(distance, 1) + "m, relative bearing " + 
              doubleToString(relative_bearing, 1) + "°");
}

//---------------------------------------------------------
// Procedure: updateContactParameters()
//            Updates existing contact with new parameters
void ContactSpawn::updateContactParameters(double heading, double relative_bearing, double distance, double speed)
{
  if(!m_contact_spawned) return;
  
  // Convert relative bearing to absolute bearing
  double absolute_bearing = m_ownship_heading + relative_bearing;
  if(absolute_bearing >= 360.0) absolute_bearing -= 360.0;
  if(absolute_bearing < 0.0) absolute_bearing += 360.0;
  
  // Calculate new position at specified distance on the relative bearing
  double bearing_rad = absolute_bearing * M_PI / 180.0;
  double new_x = m_ownship_x + distance * sin(bearing_rad);
  double new_y = m_ownship_y + distance * cos(bearing_rad);
  
  // Update contact position
  m_contact_x = new_x;
  m_contact_y = new_y;
  
  // Update contact heading
  m_contact_heading = heading;
  
  // Update contact speed
  m_contact_speed = speed;
  
  // Update last update time
  m_last_update_time = MOOSTime();
  
  // Post updated NODE_REPORT
  postNodeReport();
  
  reportEvent("Contact updated: " + m_contact_name + " at (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ") heading " + 
              doubleToString(m_contact_heading, 1) + "° at " + 
              doubleToString(m_contact_speed, 1) + " m/s, distance " + 
              doubleToString(distance, 1) + "m, relative bearing " + 
              doubleToString(relative_bearing, 1) + "°");
}

//---------------------------------------------------------
// Procedure: spawnContatoTesteOnStartup()
//            Spawna CONTATO_TESTE na posição especificada com velocidade zero no startup
void ContactSpawn::spawnContatoTesteOnStartup()
{
  if(m_contact_spawned) return;
  
  // Posição inicial especificada: x=-2459.2, y=2682.3, lat=-22.90941156, lon=-43.16096590
  m_contact_x = -2459.2;
  m_contact_y = 2682.3;
  
  // Definir propriedades do contato
  m_contact_heading = 0.0;    // Heading inicial
  m_contact_speed = 0.0;      // Velocidade zero no startup
  m_contact_name = "CONTATO_TESTE";
  m_contact_type = "ship";
  
  // Marcar como spawned
  m_contact_spawned = true;
  m_spawn_time = MOOSTime();
  m_last_update_time = MOOSTime();
  
  // Post inicial NODE_REPORT
  postNodeReport();
  
  reportEvent("CONTATO_TESTE spawned on startup at (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ") with zero speed");
}

//---------------------------------------------------------
// Procedure: updateContatoTesteForMovement()
//            Atualiza CONTATO_TESTE para movimento quando WPT_INDEX = 1
void ContactSpawn::updateContatoTesteForMovement()
{
  if(!m_contact_spawned) return;
  
  // Coordenadas do ponto de origem: x=-2269,2636.3, lat=-22.90980541, lon=-43.15910654
  double origem_x = -2269.0;
  double origem_y = 2636.3;
  
  // Coordenadas do ponto de destino: x=-2273.2, y=2481.5, lat=-22.91120355, lon=-43.15912860  
  double destino_x = -2273.2;
  double destino_y = 2481.5;
  
  // Calcular heading do contato (direção do movimento)
  double delta_x = destino_x - origem_x;
  double delta_y = destino_y - origem_y;
  double heading = atan2(delta_x, delta_y) * 180.0 / M_PI;
  
  // Normalizar heading para 0-360 graus
  if(heading < 0) heading += 360.0;
  
  // Atualizar posição do contato para origem
  m_contact_x = origem_x;
  m_contact_y = origem_y;
  
  // Atualizar propriedades do contato para movimento
  m_contact_heading = heading;
  m_contact_speed = 5.0;  // 5 m/s velocidade padrão
  
  // Atualizar tempo
  m_last_update_time = MOOSTime();
  
  // Post NODE_REPORT atualizado
  postNodeReport();
  
  reportEvent("CONTATO_TESTE updated for movement when WPT_INDEX=1 at (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ") heading " + 
              doubleToString(m_contact_heading, 1) + "° rumo ao destino (" +
              doubleToString(destino_x, 1) + ", " + 
              doubleToString(destino_y, 1) + ") at " +
              doubleToString(m_contact_speed, 1) + " m/s");
}

//---------------------------------------------------------
// Procedure: spawnContatoTesteAtMidpoint()
//            Spawna CONTATO_TESTE no ponto médio entre dois pontos quando WPT_INDEX = 2
void ContactSpawn::spawnContatoTesteAtMidpoint()
{
  // Limpar contato existente se houver
  if(m_contact_spawned) {
    cleanContact();
  }
  
  // Ponto A: x=-2269, y=2636.3, lat=-22.90980541, lon=-43.15910654
  double ponto_a_x = -2269.0;
  double ponto_a_y = 2636.3;
  
  // Ponto B: x=-2163.2, y=2548.2, lat=-22.91058889, lon=-43.15806463
  double ponto_b_x = -2163.2;
  double ponto_b_y = 2548.2;
  
  // Calcular ponto médio
  double midpoint_x = (ponto_a_x + ponto_b_x) / 2.0;  // -2216.1
  double midpoint_y = (ponto_a_y + ponto_b_y) / 2.0;  // 2592.25
  
  // Calcular heading do ponto médio para ponto B
  double delta_x = ponto_b_x - midpoint_x;
  double delta_y = ponto_b_y - midpoint_y;
  double heading = atan2(delta_x, delta_y) * 180.0 / M_PI;
  
  // Normalizar heading para 0-360 graus
  if(heading < 0) heading += 360.0;
  
  // Definir posição do contato no ponto médio
  m_contact_x = midpoint_x;
  m_contact_y = midpoint_y;
  
  // Definir propriedades do contato
  m_contact_heading = heading;
  m_contact_speed = 1.0;  // Velocidade de 1 m/s
  m_contact_name = "CONTATO_TESTE";
  m_contact_type = "ship";
  
  // Marcar como spawned
  m_contact_spawned = true;
  m_spawn_time = MOOSTime();
  m_last_update_time = MOOSTime();
  
  // Post inicial NODE_REPORT
  postNodeReport();
  
  reportEvent("CONTATO_TESTE spawned at midpoint when WPT_INDEX=2 at (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ") heading " + 
              doubleToString(m_contact_heading, 1) + "° toward point B (" +
              doubleToString(ponto_b_x, 1) + ", " + 
              doubleToString(ponto_b_y, 1) + ") at " +
              doubleToString(m_contact_speed, 1) + " m/s");
}

//---------------------------------------------------------
// Procedure: spawnContatoTesteCrossing()
//            Spawna CONTATO_TESTE cruzando quando WPT_INDEX = 3
void ContactSpawn::spawnContatoTesteCrossing()
{
  // Limpar contato existente se houver
  if(m_contact_spawned) {
    cleanContact();
  }
  
  // Ponto de origem: x=-2242.9, y=2445.8, lat=-22.91152246, lon=-43.15882892
  double origem_x = -2242.9;
  double origem_y = 2445.8;
  
  // Ponto de destino: x=-2167.6, y=2708.7, lat=-22.90914026, lon=-43.15812710
  double destino_x = -2167.6;
  double destino_y = 2708.7;
  
  // Calcular heading do ponto de origem para destino
  double delta_x = destino_x - origem_x;
  double delta_y = destino_y - origem_y;
  double heading = atan2(delta_x, delta_y) * 180.0 / M_PI;
  
  // Normalizar heading para 0-360 graus
  if(heading < 0) heading += 360.0;
  
  // Definir posição do contato na origem
  m_contact_x = origem_x;
  m_contact_y = origem_y;
  
  // Definir propriedades do contato
  m_contact_heading = heading;
  m_contact_speed = 3.0;  // Velocidade de 3 m/s
  m_contact_name = "CONTATO_TESTE";
  m_contact_type = "ship";
  
  // Marcar como spawned
  m_contact_spawned = true;
  m_spawn_time = MOOSTime();
  m_last_update_time = MOOSTime();
  
  // Post inicial NODE_REPORT
  postNodeReport();
  
  reportEvent("CONTATO_TESTE spawned crossing when WPT_INDEX=3 from (" + 
              doubleToString(m_contact_x, 1) + ", " + 
              doubleToString(m_contact_y, 1) + ") heading " + 
              doubleToString(m_contact_heading, 1) + "° toward (" +
              doubleToString(destino_x, 1) + ", " + 
              doubleToString(destino_y, 1) + ") at " +
              doubleToString(m_contact_speed, 1) + " m/s");
}



