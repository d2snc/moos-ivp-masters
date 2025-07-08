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
#include "XYPoint.h"

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

  // Initialize collision status
  collision_status = "NONE";
  
  // Initialize nodes pointer
  nodes = nullptr;
  nodes_visualized = false;

  // Nodes variables for A* algorithm
  int x_start = -2657; // Start position of the grid in the map
  int y_start = 2354; // Start position of the grid in the map
  int x_end = -1634; //Considerando uma parte do mapa de 900x900
  int y_end = 3292; //End position of the grid in the map

  // Store grid dimensions as class members
  nodes_width = abs(x_end - x_start);
  nodes_height = abs(y_end - y_start);

  // Create nodes for the A* algorithm
  nodes = new sNode[nodes_width * nodes_height];

  // Initialize nodes
  int width = nodes_width;
  int height = nodes_height;
  for (int x = x_start; x < x_end; x++) {
    for (int y = y_start; y < y_end; y++) {
      int idx = (y - y_start) * width + (x - x_start);
      nodes[idx].x = x;
      nodes[idx].y = y;
      nodes[idx].bObstacle = false;
      nodes[idx].bVisited = false;
      nodes[idx].parent = nullptr;
    }
  }

  //Create connections between nodes
  for (int x = x_start; x < x_end; x++) 
    for (int y = y_start; y < y_end; y++) {
      int idx = (y - y_start) * width + (x - x_start);
      if (y > y_start)
        nodes[idx].vecNeighbours.push_back(&nodes[(y-1-y_start) * width + (x-x_start)]); 
      if (y < y_end - 1)
        nodes[idx].vecNeighbours.push_back(&nodes[(y+1-y_start) * width + (x-x_start)]);
      if (x > x_start)
        nodes[idx].vecNeighbours.push_back(&nodes[(y-y_start) * width + (x-1-x_start)]);
      if (x < x_end - 1)
        nodes[idx].vecNeighbours.push_back(&nodes[(y-y_start) * width + (x+1-x_start)]);
    }

}

//---------------------------------------------------------
// Destructor

ColAvd::~ColAvd()
{
  if (nodes != nullptr) {
    delete[] nodes;
    nodes = nullptr;
  }
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

  //Equation (1) and (2) of paper https://www.sciencedirect.com/science/article/pii/S0029801824028907

  phi = m_contact_heading - m_nav_heading;


  beta_ts = atan2(m_contact_x - m_nav_x, m_contact_y - m_nav_y);

  // Convert beta_ts to degrees
  beta_ts = beta_ts * (180.0 / M_PI);

  // Calcula o ângulo relativo com base no rumo do navio

  if (beta_ts < 0) { //Contato está a esquerda do norte do meu navio
    beta = m_nav_heading + abs(beta_ts);
  } else {
    beta = m_nav_heading - abs(beta_ts);
  }
  
  
  
  //Verify the distance and trigger when it comes closer
  if (m_contact_distance < m_col_avd_distance) {
    // Check if its a headon situation
    // if beta_ts between -12 and 12 degrees, then it is a headon situation
    if (beta_ts > -12 && beta_ts < 12) {
      collision_status = "HEADON";
    }

    
  
    

    
  }

  // Visualize all A* grid nodes as points (only once)
  if (!nodes_visualized) {
    vector<XYPoint> grid_points;
    for (int i = 0; i < nodes_width * nodes_height; i++) {
      XYPoint point(nodes[i].x, nodes[i].y);
      point.set_label("node_" + to_string(i));
      point.set_vertex_size(1);
      point.set_color("fill_color", "darkblue");
      grid_points.push_back(point);
      string point_str = point.get_spec();
      Notify("VIEW_POINT", point_str);
    }
    nodes_visualized = true;
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
  m_msgs << " Phi (Angle between headings): " << phi << " degrees" << endl;
  m_msgs << " Beta_ts (Angle to target): " << beta_ts << " degrees" << endl;
  m_msgs << " Beta (Relative bearing): " << beta << " degrees" << endl;
  m_msgs << " Collision Status: " << collision_status << endl;

  return(true);
}




