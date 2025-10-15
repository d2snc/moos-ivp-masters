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
  m_desired_speed = 2.0;  // Default 2 m/s
  m_max_speed = 5.0;  // Default 5 m/s
  m_alpha_shift = 0.5;  // Shift gain for VO
  m_alpha_speed = 1.0;  // Weight for speed in cost function
  m_alpha_course = 1.0;  // Weight for course in cost function
  m_safety_radius = 25.0;  // 25 meters safety zone
  m_collision_distance = 50.0;  // 50 meters collision avoidance activation
  m_desired_heading = 0;
  m_avoidance_heading = 0;
  m_avoidance_speed = 0;
  m_collision_detected = false;
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
  m_msgs << endl;

  m_msgs << "Collision Avoidance:" << endl;
  m_msgs << "  Collision Detected: " << (m_collision_detected ? "YES" : "NO") << endl;
  m_msgs << "  Avoidance Heading: " << m_avoidance_heading << " deg" << endl;
  m_msgs << "  Avoidance Speed: " << m_avoidance_speed << " m/s" << endl;
  m_msgs << "  Activation Distance: " << m_collision_distance << " m" << endl;
  m_msgs << endl;

  m_msgs << "Contacts: " << m_contacts.size() << endl;

  ACTable actab(6);
  actab << "Contact | X | Y | Dist(m) | Heading | Speed";
  actab.addHeaderLines();

  map<string, NodeRecord>::iterator it;
  for(it = m_contacts.begin(); it != m_contacts.end(); it++) {
    NodeRecord contact = it->second;
    double dx = contact.getX() - m_nav_x;
    double dy = contact.getY() - m_nav_y;
    double dist = sqrt(dx*dx + dy*dy);

    actab << contact.getName();
    actab << contact.getX();
    actab << contact.getY();
    actab << dist;
    actab << contact.getHeading();
    actab << contact.getSpeed();
  }
  m_msgs << actab.getFormattedString();

  return(true);
}

//---------------------------------------------------------
// Procedure: angleDiff()
// Purpose: Calculate the difference between two angles in degrees
//          Returns value in range [-180, 180]

double ColAvd_vo::angleDiff(double a1, double a2)
{
  double diff = a1 - a2;
  while(diff > 180.0) diff -= 360.0;
  while(diff < -180.0) diff += 360.0;
  return diff;
}

//---------------------------------------------------------
//---------------------------------------------------------
// Procedure: computeCollisionCone()
// Purpose: Implement VO-based collision avoidance from Cho et al. (2019)
//          "Experimental validation of a velocity obstacle based
//          collision avoidance algorithm for unmanned surface vehicles"

void ColAvd_vo::computeCollisionCone()
{
  m_collision_cone.clear();
  m_collision_detected = false;

  if(m_contacts.empty()) {
    m_avoidance_heading = m_desired_heading;
    m_avoidance_speed = m_desired_speed;
    // No collision - disable constant heading
    Notify("CONSTANT_HEADING", "false");
    return;
  }

  // Check if any contact is within collision avoidance distance
  bool within_collision_distance = false;
  double closest_distance = 1e10;

  map<string, NodeRecord>::iterator check_it;
  for(check_it = m_contacts.begin(); check_it != m_contacts.end(); check_it++) {
    NodeRecord contact = check_it->second;
    double cx = contact.getX();
    double cy = contact.getY();
    double dx = cx - m_nav_x;
    double dy = cy - m_nav_y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < closest_distance)
      closest_distance = dist;

    if(dist < m_collision_distance) {
      within_collision_distance = true;
      m_collision_detected = true;
    }
  }

  if(!within_collision_distance) {
    // Outside collision avoidance zone - use desired heading
    m_avoidance_heading = m_desired_heading;
    m_avoidance_speed = m_desired_speed;
    Notify("CONSTANT_HEADING", "false");
    return;
  }

  // Sample velocity space to find optimal collision-free velocity
  // Simplified algorithm: find heading that avoids collision cone
  double best_heading = m_nav_hdg;  // Use current heading as fallback
  double best_speed = m_desired_speed;
  double min_cost = 1e10;

  // Get the closest contact to focus avoidance
  NodeRecord closest_contact;
  double min_dist = 1e10;

  map<string, NodeRecord>::iterator it2;
  for(it2 = m_contacts.begin(); it2 != m_contacts.end(); it2++) {
    NodeRecord contact = it2->second;
    double cx = contact.getX();
    double cy = contact.getY();
    double dx = cx - m_nav_x;
    double dy = cy - m_nav_y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < min_dist && dist < m_collision_distance) {
      min_dist = dist;
      closest_contact = contact;
    }
  }

  if(min_dist < m_collision_distance) {
    // Calculate collision cone boundaries for the closest contact
    double cx = closest_contact.getX();
    double cy = closest_contact.getY();
    double dx = cx - m_nav_x;
    double dy = cy - m_nav_y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist > m_safety_radius) {
      // Angle to contact (atan2 returns angle from X-axis, convert to heading from North)
      double theta_to_contact_xy = atan2(dy, dx) * 180.0 / M_PI;
      double theta_to_contact = 90.0 - theta_to_contact_xy;  // Convert: East=0° -> North=0°

      // Half-angle of collision cone
      double alpha = asin(m_safety_radius / dist) * 180.0 / M_PI;

      // Collision cone boundaries (in nautical heading: North=0°, clockwise)
      double cone_left = theta_to_contact - alpha;
      double cone_right = theta_to_contact + alpha;

      // Normalize angles to [0, 360)
      while(cone_left < 0) cone_left += 360.0;
      while(cone_right < 0) cone_right += 360.0;
      while(cone_left >= 360.0) cone_left -= 360.0;
      while(cone_right >= 360.0) cone_right -= 360.0;

      reportEvent("VO: Collision cone from " + doubleToString(cone_left, 1) +
                  " to " + doubleToString(cone_right, 1) + " deg (North=0)");

      // Desviar sempre para boreste (direita) = cone_right + margem
      best_heading = cone_right + 2.0;

      // Normalizar heading
      while(best_heading < 0) best_heading += 360.0;
      while(best_heading >= 360.0) best_heading -= 360.0;
    }
  }

  m_avoidance_heading = best_heading;
  m_avoidance_speed = best_speed;

  // Notify MOOS variables for collision avoidance
  if(within_collision_distance) {
    Notify("CONSTANT_HEADING", "true");
    string hdg_update = "heading=" + doubleToString(best_heading, 1);
    Notify("CONST_HDG_UPDATES", hdg_update);

    // Debug info
    string debug_msg = "VO: Collision detected at " + doubleToString(closest_distance, 1) +
                       "m, commanding heading " + doubleToString(best_heading, 1) + " deg";
    reportEvent(debug_msg);
  }

  // ========== VISUALIZATION ==========add_vertex
  // Visualize the velocity obstacles for all contacts
  double vo_origin_x = m_nav_x + 200;
  double vo_origin_y = m_nav_y + 200;
  double vel_scale = 20.0;

  map<string, NodeRecord>::iterator it;
  for(it = m_contacts.begin(); it != m_contacts.end(); it++) {
    NodeRecord contact = it->second;

    double cx = contact.getX();
    double cy = contact.getY();
    double ch = contact.getHeading();
    double cv = contact.getSpeed();

    double dx = cx - m_nav_x;
    double dy = cy - m_nav_y;
    double dist = sqrt(dx*dx + dy*dy);
    if(dist < 0.1) continue;

    // Contact velocity
    double cv_x = cv * cos(ch * M_PI / 180.0);
    double cv_y = cv * sin(ch * M_PI / 180.0);

    // Collision cone angles
    double combined_radius = m_safety_radius;
    if(dist <= combined_radius) continue;

    double theta = atan2(dy, dx) * 180.0 / M_PI;
    double alpha = asin(combined_radius / dist) * 180.0 / M_PI;
    double theta1 = theta - alpha;
    double theta2 = theta + alpha;

    // Visualize collision cone in configuration space
    double cone_length = 150.0;
    XYSegList cone_edge1;
    cone_edge1.add_vertex(m_nav_x, m_nav_y);
    cone_edge1.add_vertex(m_nav_x + cone_length * cos(theta1 * M_PI / 180.0),
                          m_nav_y + cone_length * sin(theta1 * M_PI / 180.0));
    cone_edge1.set_label("CC_edge1_" + contact.getName());
    cone_edge1.set_color("edge", "red");
    cone_edge1.set_edge_size(2);
    Notify("VIEW_SEGLIST", cone_edge1.get_spec());

    XYSegList cone_edge2;
    cone_edge2.add_vertex(m_nav_x, m_nav_y);
    cone_edge2.add_vertex(m_nav_x + cone_length * cos(theta2 * M_PI / 180.0),
                          m_nav_y + cone_length * sin(theta2 * M_PI / 180.0));
    cone_edge2.set_label("CC_edge2_" + contact.getName());
    cone_edge2.set_color("edge", "red");
    cone_edge2.set_edge_size(2);
    Notify("VIEW_SEGLIST", cone_edge2.get_spec());

    // Line to contact
    XYSegList contact_line;
    contact_line.add_vertex(m_nav_x, m_nav_y);
    contact_line.add_vertex(cx, cy);
    contact_line.set_label("contact_line_" + contact.getName());
    contact_line.set_color("edge", "blue");
    contact_line.set_edge_size(1);
    Notify("VIEW_SEGLIST", contact_line.get_spec());

    // Velocity obstacle in velocity space (orange cone)
    double vo_apex_x = vo_origin_x + cv_x * vel_scale;
    double vo_apex_y = vo_origin_y + cv_y * vel_scale;
    double vo_length = 80.0;

    XYSegList vo_edge1;
    vo_edge1.add_vertex(vo_apex_x, vo_apex_y);
    vo_edge1.add_vertex(vo_apex_x + vo_length * cos(theta1 * M_PI / 180.0),
                        vo_apex_y + vo_length * sin(theta1 * M_PI / 180.0));
    vo_edge1.set_label("VO_edge1_" + contact.getName());
    vo_edge1.set_color("edge", "orange");
    vo_edge1.set_edge_size(2);
    Notify("VIEW_SEGLIST", vo_edge1.get_spec());

    XYSegList vo_edge2;
    vo_edge2.add_vertex(vo_apex_x, vo_apex_y);
    vo_edge2.add_vertex(vo_apex_x + vo_length * cos(theta2 * M_PI / 180.0),
                        vo_apex_y + vo_length * sin(theta2 * M_PI / 180.0));
    vo_edge2.set_label("VO_edge2_" + contact.getName());
    vo_edge2.set_color("edge", "orange");
    vo_edge2.set_edge_size(2);
    Notify("VIEW_SEGLIST", vo_edge2.get_spec());
  }

  // Draw velocity space axes
  XYSegList vel_x_axis;
  vel_x_axis.add_vertex(vo_origin_x - 50, vo_origin_y);
  vel_x_axis.add_vertex(vo_origin_x + 50, vo_origin_y);
  vel_x_axis.set_label("vel_x_axis");
  vel_x_axis.set_color("edge", "white");
  vel_x_axis.set_edge_size(1);
  Notify("VIEW_SEGLIST", vel_x_axis.get_spec());

  XYSegList vel_y_axis;
  vel_y_axis.add_vertex(vo_origin_x, vo_origin_y - 50);
  vel_y_axis.add_vertex(vo_origin_x, vo_origin_y + 50);
  vel_y_axis.set_label("vel_y_axis");
  vel_y_axis.set_color("edge", "white");
  vel_y_axis.set_edge_size(1);
  Notify("VIEW_SEGLIST", vel_y_axis.get_spec());

  // Draw selected velocity (green arrow)
  double sel_vel_x = vo_origin_x + best_speed * cos(best_heading * M_PI / 180.0) * vel_scale;
  double sel_vel_y = vo_origin_y + best_speed * sin(best_heading * M_PI / 180.0) * vel_scale;

  XYSegList selected_vel;
  selected_vel.add_vertex(vo_origin_x, vo_origin_y);
  selected_vel.add_vertex(sel_vel_x, sel_vel_y);
  selected_vel.set_label("selected_velocity");
  selected_vel.set_color("edge", "green");
  selected_vel.set_edge_size(3);
  Notify("VIEW_SEGLIST", selected_vel.get_spec());
}





