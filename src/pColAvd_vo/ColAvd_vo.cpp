/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ColAvd_vo.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "ColAvd_vo.h"
#include "NodeRecordUtils.h"
#include "GeomUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

ColAvd_vo::ColAvd_vo()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_hdg = 0;
  m_nav_spd = 0;
  m_ship_radius = 5.0;  // Default 5 meters
}

//---------------------------------------------------------
// Destructor

ColAvd_vo::~ColAvd_vo()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ColAvd_vo::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if(key == "NAV_X")
      m_nav_x = msg.GetDouble();
    else if(key == "NAV_Y")
      m_nav_y = msg.GetDouble();
    else if(key == "NAV_HEADING")
      m_nav_hdg = msg.GetDouble();
    else if(key == "NAV_SPEED")
      m_nav_spd = msg.GetDouble();
    else if(key == "NODE_REPORT") {
      NodeRecord new_node = string2NodeRecord(msg.GetString());
      if(new_node.valid())
        m_contacts[new_node.getName()] = new_node;
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
   }

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ColAvd_vo::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ColAvd_vo::Iterate()
{
  AppCastingMOOSApp::Iterate();

  computeCollisionCone();
  visualizeCollisionCone();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ColAvd_vo::OnStartUp()
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

void ColAvd_vo::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("NODE_REPORT", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool ColAvd_vo::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "Velocity Obstacle Collision Avoidance       " << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Ownship Position: (" << m_nav_x << ", " << m_nav_y << ")" << endl;
  m_msgs << "Ownship Heading: " << m_nav_hdg << " deg" << endl;
  m_msgs << "Ownship Speed: " << m_nav_spd << " m/s" << endl;
  m_msgs << "Number of Contacts: " << m_contacts.size() << endl;

  ACTable actab(5);
  actab << "Contact | X | Y | Heading | Speed";
  actab.addHeaderLines();

  map<string, NodeRecord>::iterator it;
  for(it = m_contacts.begin(); it != m_contacts.end(); it++) {
    NodeRecord contact = it->second;
    actab << contact.getName();
    actab << contact.getX();
    actab << contact.getY();
    actab << contact.getHeading();
    actab << contact.getSpeed();
  }
  m_msgs << actab.getFormattedString();

  return(true);
}

//---------------------------------------------------------
// Procedure: computeCollisionCone()
// Purpose: Compute the velocity obstacle (collision cone) based on
//          the Velocity Obstacles paper by Fiorini & Shiller

void ColAvd_vo::computeCollisionCone()
{
  m_collision_cone.clear();

  if(m_contacts.empty())
    return;

  // For each contact, compute collision cone
  map<string, NodeRecord>::iterator it;
  for(it = m_contacts.begin(); it != m_contacts.end(); it++) {
    NodeRecord contact = it->second;

    // Get contact position and velocity
    double cx = contact.getX();
    double cy = contact.getY();
    double ch = contact.getHeading();
    double cv = contact.getSpeed();

    // Compute relative position vector from ownship to contact
    double dx = cx - m_nav_x;
    double dy = cy - m_nav_y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < 0.1)  // Too close, skip
      continue;

    // Calculate the tangent angles for the collision cone
    // Using the combined radius (ownship + contact)
    double combined_radius = 2 * m_ship_radius;

    if(dist <= combined_radius)
      continue;  // Already in collision

    // Angle of the line from ownship to contact
    double theta = atan2(dy, dx) * 180.0 / M_PI;

    // Half-angle of the collision cone
    double alpha = asin(combined_radius / dist) * 180.0 / M_PI;

    // The two tangent lines of the collision cone
    double theta1 = theta - alpha;
    double theta2 = theta + alpha;

    // Project the cone out to a reasonable distance (based on speeds)
    double cone_length = (m_nav_spd + cv) * 100.0;  // 100 seconds ahead
    if(cone_length < 100.0)
      cone_length = 100.0;

    // Create segment list for the collision cone
    // Cone originates from ownship position
    double x1 = m_nav_x + cone_length * cos(theta1 * M_PI / 180.0);
    double y1 = m_nav_y + cone_length * sin(theta1 * M_PI / 180.0);

    double x2 = m_nav_x + cone_length * cos(theta2 * M_PI / 180.0);
    double y2 = m_nav_y + cone_length * sin(theta2 * M_PI / 180.0);

    // Draw the collision cone edges
    XYSegList cone_edge1;
    cone_edge1.add_vertex(m_nav_x, m_nav_y);
    cone_edge1.add_vertex(x1, y1);
    cone_edge1.set_label("VO_edge1_" + contact.getName());
    cone_edge1.set_color("edge", "red");
    cone_edge1.set_edge_size(2);
    Notify("VIEW_SEGLIST", cone_edge1.get_spec());

    XYSegList cone_edge2;
    cone_edge2.add_vertex(m_nav_x, m_nav_y);
    cone_edge2.add_vertex(x2, y2);
    cone_edge2.set_label("VO_edge2_" + contact.getName());
    cone_edge2.set_color("edge", "red");
    cone_edge2.set_edge_size(2);
    Notify("VIEW_SEGLIST", cone_edge2.get_spec());

    // Draw the arc of the cone at the obstacle
    int num_arc_points = 20;
    XYSegList cone_arc;
    for(int i = 0; i <= num_arc_points; i++) {
      double angle = theta1 + (theta2 - theta1) * i / num_arc_points;
      double arc_x = cx + combined_radius * cos(angle * M_PI / 180.0);
      double arc_y = cy + combined_radius * sin(angle * M_PI / 180.0);
      cone_arc.add_vertex(arc_x, arc_y);
    }
    cone_arc.set_label("VO_arc_" + contact.getName());
    cone_arc.set_color("edge", "red");
    cone_arc.set_edge_size(2);
    Notify("VIEW_SEGLIST", cone_arc.get_spec());

    // Draw line from ownship to contact (for visualization)
    XYSegList ship_to_contact;
    ship_to_contact.add_vertex(m_nav_x, m_nav_y);
    ship_to_contact.add_vertex(cx, cy);
    ship_to_contact.set_label("ship_contact_" + contact.getName());
    ship_to_contact.set_color("edge", "blue");
    ship_to_contact.set_edge_size(1);
    Notify("VIEW_SEGLIST", ship_to_contact.get_spec());

    // ========== VELOCITY OBSTACLE IN VELOCITY SPACE ==========
    // According to Fiorini & Shiller, the velocity obstacle VO is the set
    // of velocities that will cause a collision with the obstacle.
    // VO = {v | v in CC(O - P) + v_B}
    // where CC is the collision cone, O is obstacle, P is robot position,
    // and v_B is the obstacle velocity

    // Contact velocity vector
    double cv_x = cv * cos(ch * M_PI / 180.0);
    double cv_y = cv * sin(ch * M_PI / 180.0);

    // The velocity obstacle is the collision cone translated by the contact's velocity
    // We'll draw this in a separate location on the map (offset from origin)

    // Choose a location to draw the velocity space diagram
    // Place it offset from the ownship
    double vo_origin_x = m_nav_x + 200;  // 200m to the right
    double vo_origin_y = m_nav_y + 200;  // 200m up

    // Scale factor for velocity visualization (m/s to map units)
    double vel_scale = 20.0;  // 20 map units per m/s

    // The velocity obstacle apex is at the contact's velocity
    double vo_apex_x = vo_origin_x + cv_x * vel_scale;
    double vo_apex_y = vo_origin_y + cv_y * vel_scale;

    // The velocity obstacle extends from the apex at the same angles as collision cone
    double vo_length = cone_length / 10.0;  // Shorter for velocity space

    double vo_x1 = vo_apex_x + vo_length * cos(theta1 * M_PI / 180.0);
    double vo_y1 = vo_apex_y + vo_length * sin(theta1 * M_PI / 180.0);

    double vo_x2 = vo_apex_x + vo_length * cos(theta2 * M_PI / 180.0);
    double vo_y2 = vo_apex_y + vo_length * sin(theta2 * M_PI / 180.0);

    // Draw velocity obstacle edges
    XYSegList vo_edge1;
    vo_edge1.add_vertex(vo_apex_x, vo_apex_y);
    vo_edge1.add_vertex(vo_x1, vo_y1);
    vo_edge1.set_label("VO_vel_edge1_" + contact.getName());
    vo_edge1.set_color("edge", "orange");
    vo_edge1.set_edge_size(2);
    Notify("VIEW_SEGLIST", vo_edge1.get_spec());

    XYSegList vo_edge2;
    vo_edge2.add_vertex(vo_apex_x, vo_apex_y);
    vo_edge2.add_vertex(vo_x2, vo_y2);
    vo_edge2.set_label("VO_vel_edge2_" + contact.getName());
    vo_edge2.set_color("edge", "orange");
    vo_edge2.set_edge_size(2);
    Notify("VIEW_SEGLIST", vo_edge2.get_spec());

    // Draw arc for velocity obstacle base
    XYSegList vo_arc;
    double vo_arc_dist = combined_radius * vel_scale / dist;  // Scale the arc appropriately
    for(int i = 0; i <= num_arc_points; i++) {
      double angle = theta1 + (theta2 - theta1) * i / num_arc_points;
      double vo_arc_x = vo_apex_x + vo_arc_dist * cos(angle * M_PI / 180.0);
      double vo_arc_y = vo_apex_y + vo_arc_dist * sin(angle * M_PI / 180.0);
      vo_arc.add_vertex(vo_arc_x, vo_arc_y);
    }
    vo_arc.set_label("VO_vel_arc_" + contact.getName());
    vo_arc.set_color("edge", "orange");
    vo_arc.set_edge_size(2);
    Notify("VIEW_SEGLIST", vo_arc.get_spec());

    // Draw ownship current velocity in velocity space
    double os_vel_x_vs = vo_origin_x + m_nav_spd * cos(m_nav_hdg * M_PI / 180.0) * vel_scale;
    double os_vel_y_vs = vo_origin_y + m_nav_spd * sin(m_nav_hdg * M_PI / 180.0) * vel_scale;

    XYSegList os_vel_arrow;
    os_vel_arrow.add_vertex(vo_origin_x, vo_origin_y);
    os_vel_arrow.add_vertex(os_vel_x_vs, os_vel_y_vs);
    os_vel_arrow.set_label("OS_velocity_space");
    os_vel_arrow.set_color("edge", "green");
    os_vel_arrow.set_edge_size(3);
    Notify("VIEW_SEGLIST", os_vel_arrow.get_spec());

    // Draw axes for velocity space (reference)
    XYSegList vel_space_x_axis;
    vel_space_x_axis.add_vertex(vo_origin_x - 50, vo_origin_y);
    vel_space_x_axis.add_vertex(vo_origin_x + 50, vo_origin_y);
    vel_space_x_axis.set_label("vel_space_x_axis");
    vel_space_x_axis.set_color("edge", "white");
    vel_space_x_axis.set_edge_size(1);
    Notify("VIEW_SEGLIST", vel_space_x_axis.get_spec());

    XYSegList vel_space_y_axis;
    vel_space_y_axis.add_vertex(vo_origin_x, vo_origin_y - 50);
    vel_space_y_axis.add_vertex(vo_origin_x, vo_origin_y + 50);
    vel_space_y_axis.set_label("vel_space_y_axis");
    vel_space_y_axis.set_color("edge", "white");
    vel_space_y_axis.set_edge_size(1);
    Notify("VIEW_SEGLIST", vel_space_y_axis.get_spec());
  }
}

//---------------------------------------------------------
// Procedure: visualizeCollisionCone()

void ColAvd_vo::visualizeCollisionCone()
{
  // Visualization is done in computeCollisionCone()
  // This function is kept for potential future use
}




