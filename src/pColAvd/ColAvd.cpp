/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ColAvd.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ColAvd.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ColAvd::ColAvd()
{
  //Own ship data
  m_nav_heading = 0.0;
  m_nav_speed = 0.0;
  m_nav_x = 0.0;
  m_nav_y = 0.0;
  
  // Simulated contact data
  m_contact_x = 0.0;
  m_contact_y = 0.0;
  m_contact_heading = 0.0;
  m_contact_speed = 0.0;
  m_contact_distance = 0.0;

  // Collision Avoidance parameters
  m_col_avd_distance = 300;
}

//---------------------------------------------------------
// Destructor

ColAvd::~ColAvd()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ColAvd::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "NAV_HEADING") {
       m_nav_heading = msg.GetDouble();
     }
     else if(key == "NAV_SPEED") {
       m_nav_speed = msg.GetDouble();
     }
     else if(key == "NAV_X") {
       m_nav_x = msg.GetDouble();
     }
     else if(key == "NAV_Y") {
       m_nav_y = msg.GetDouble();
     }
     else if(key == "NODE_REPORT") {
       string node_report = msg.GetString();
       parseNodeReport(node_report);
     }
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ColAvd::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ColAvd::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!

  // Calculate the distance to the contact
  m_contact_distance = hypot(m_contact_x - m_nav_x, m_contact_y - m_nav_y);

  //Verify the distance and trigger when it comes closer
  if (m_contact_distance < m_col_avd_distance) {
    //reportRunWarning("Contact within collision avoidance distance: " + 
    //                 doubleToString(m_contact_distance, 2) + " meters");

    //Check if the contact is moving towards the own ship with a maximum relative bearing of 12 degrees
    
  
    

    
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ColAvd::OnStartUp()
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
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

void ColAvd::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NODE_REPORT", 0);
}

//---------------------------------------------------------
// Procedure: parseNodeReport()

void ColAvd::parseNodeReport(const string& node_report)
{
  vector<string> svector = parseString(node_report, ',');
  
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    
    if(param == "X") {
      m_contact_x = strtod(value.c_str(), 0);
    }
    else if(param == "Y") {
      m_contact_y = strtod(value.c_str(), 0);
    }
    else if(param == "HDG") {
      m_contact_heading = strtod(value.c_str(), 0);
    }
    else if(param == "SPD") {
      m_contact_speed = strtod(value.c_str(), 0);
    }
  }
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ColAvd::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "Collision Avoidance Status                  " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Own Ship Navigation:" << endl;
  m_msgs << "  Position (X,Y): (" << m_nav_x << ", " << m_nav_y << ")" << endl;
  m_msgs << "  Heading: " << m_nav_heading << " degrees" << endl;
  m_msgs << "  Speed: " << m_nav_speed << " m/s" << endl;
  m_msgs << endl;

  m_msgs << "Contact Information:" << endl;
  m_msgs << "  Position (X,Y): (" << m_contact_x << ", " << m_contact_y << ")" << endl;
  m_msgs << "  Heading: " << m_contact_heading << " degrees" << endl;
  m_msgs << "  Speed: " << m_contact_speed << " m/s" << endl;
  m_msgs << "  Distance: " << m_contact_distance << " meters" << endl;

  return(true);
}




