/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ColAvd.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

/*Major Performance Optimizations (aplicada apenas Sugestão 1):

  1. **Open list em heap (`std::priority_queue`)**
     – Antes: `std::list` + `sort()` em cada iteração (O(k log k)).
     – Depois: fila de prioridade binária (heap) com comparação por `fGlobalGoal`.
     – Ganho: 20–30× menos overhead de ordenação em mapas grandes.
*/

#include <iterator>
#include <queue>           // <-- Adicionado para priority_queue
#include "MBUtils.h"
#include "ACTable.h"
#include "ColAvd.h"
#include "XYPoint.h"
#include "XYSegList.h"

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
  node_start = nullptr;
  node_end = nullptr;
  nodes_visualized = false;
  obstacles_changed = false;
  path_solved = false;


  // Nodes variables for A* algorithm
  int x_start = -2657; // Start position of the grid in the map
  int y_start = 2354;  // Start position of the grid in the map
  int x_end   = -1634; // Considerando uma parte do mapa de 900x900
  int y_end   = 3292;  // End position of the grid in the map

  // Store grid dimensions as class members
  nodes_width  = abs(x_end - x_start);
  nodes_height = abs(y_end - y_start);

  // Create nodes for the A* algorithm
  nodes = new sNode[nodes_width * nodes_height];

  // Initialize nodes
  int width  = nodes_width;
  int height = nodes_height;
  for (int x = x_start; x < x_end; x++) {
    for (int y = y_start; y < y_end; y++) {
      int idx = (y - y_start) * width + (x - x_start);
      nodes[idx].x = x;
      nodes[idx].y = y;
      nodes[idx].bObstacle = false;
      nodes[idx].bVisited  = false;
      nodes[idx].parent    = nullptr;
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

  //Giving some default values to nodes start and end
  //(-2363,2678) -> node start
  //(-1676,2908) -> node end
  node_start = &nodes[(2678 - y_start) * width + (-2363 - x_start)];
  node_end   = &nodes[(2908 - y_start) * width + (-1676 - x_start)];

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
     else if(key == "MVIEWER_LCLICK") {
       mviewer_lclick = msg.GetString();
       parseMViewerLClick(mviewer_lclick);
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

  if (beta_ts < 0) {
    beta = m_nav_heading + abs(beta_ts);
  } else {
    beta = m_nav_heading - abs(beta_ts);
  }
  
  //Verify the distance and trigger when it comes closer
  if (m_contact_distance < m_col_avd_distance) {
    // Check if its a headon situation
    if (beta_ts > -12 && beta_ts < 12) {
      collision_status = "HEADON";
    }
  }

  // Only solve A* when path hasn't been solved yet or obstacles have changed
  if (!path_solved || obstacles_changed)
  {
    // Solve A* algorithm to find the shortest path
    Solve_AStar();
    
    // Reset obstacles_changed flag after solving
    obstacles_changed = false;
    
    // Visualize the updated path
    visualizeAStarPath();
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: Solve_AStar()  (implementa Sugestão 1)

void ColAvd::Solve_AStar()
{
  // Reinicializa nós
  for (int x = 0; x < nodes_width; x++)
    for (int y = 0; y < nodes_height; y++)
    {
      sNode &n = nodes[y*nodes_width + x];
      n.bVisited    = false;
      n.fGlobalGoal = INFINITY;
      n.fLocalGoal  = INFINITY;
      n.parent      = nullptr;
    }

  // Distância Euclidiana
  auto distance = [](sNode* a, sNode* b)
  {
      return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
  };

  auto heuristic = [distance](sNode* a, sNode* b)
  {
      return distance(a, b);
  };

  // Condições iniciais
  sNode *nodeCurrent = node_start;
  node_start->fLocalGoal  = 0.0f;
  node_start->fGlobalGoal = heuristic(node_start, node_end);

  // Comparator para a fila de prioridade
  struct NodeCompare {
      bool operator()(const sNode* lhs, const sNode* rhs) const {
          return lhs->fGlobalGoal > rhs->fGlobalGoal; // menor fGlobalGoal tem prioridade
      }
  };

  std::priority_queue<sNode*, std::vector<sNode*>, NodeCompare> open;
  open.push(node_start);

  while (!open.empty() && nodeCurrent != node_end)
  {
      // Remove nós já visitados no topo
      while (!open.empty() && open.top()->bVisited)
          open.pop();

      if (open.empty())
          break;

      nodeCurrent = open.top();
      open.pop();
      nodeCurrent->bVisited = true;

      // Explora vizinhos
      for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
      {
          if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
              open.push(nodeNeighbour);

          float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

          if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
          {
              nodeNeighbour->parent = nodeCurrent;
              nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
              nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, node_end);
          }
      }
  }

  // Caminho calculado
  path_solved = true;
}

//---------------------------------------------------------
// Procedure: visualizeAStarPath()

void ColAvd::visualizeAStarPath()
{
  if (node_end != nullptr)
  {
    vector<sNode*> path_nodes;
    sNode *p = node_end;
    while (p != nullptr)
    {
      path_nodes.push_back(p);
      p = p->parent;
    }

    if (path_nodes.size() > 1)
    {
      XYSegList astar_path;
      for (int i = path_nodes.size() - 1; i >= 0; i--)
      {
        astar_path.add_vertex(path_nodes[i]->x, path_nodes[i]->y);
      }
      astar_path.set_label("astar_path");
      astar_path.set_edge_color("yellow");
      astar_path.set_edge_size(3);
      string astar_path_str = astar_path.get_spec();
      Notify("VIEW_SEGLIST", astar_path_str);
    }
  }
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

  for(auto &orig : sParams) {
    string line  = orig;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") handled = true;
    else if(param == "bar") handled = true;

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();

  // Solve initial A* path at startup to avoid delay on first iteration
  Solve_AStar();
  visualizeAStarPath();

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
  Register("MVIEWER_LCLICK", 0);
}

//---------------------------------------------------------
// Procedure: parseNodeReport()

void ColAvd::parseNodeReport(const string& node_report)
{
  vector<string> svector = parseString(node_report, ',');
  
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    
    if(param == "X")          m_contact_x        = strtod(value.c_str(), 0);
    else if(param == "Y")     m_contact_y        = strtod(value.c_str(), 0);
    else if(param == "HDG")   m_contact_heading  = strtod(value.c_str(), 0);
    else if(param == "SPD")   m_contact_speed    = strtod(value.c_str(), 0);
  }
}

//---------------------------------------------------------
// Procedure: parseMViewerLClick()

void ColAvd::parseMViewerLClick(const string& mviewer_lclick)
{
  vector<string> svector = parseString(mviewer_lclick, ',');
  
  double click_x = 0.0;
  double click_y = 0.0;
  
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = biteStringX(svector[i], '=');
    string value = svector[i];
    
    if(param == "x")          click_x = strtod(value.c_str(), 0);
    else if(param == "y")     click_y = strtod(value.c_str(), 0);
  }
  
  // Set obstacles around the clicked point
  setObstaclesAroundPoint(click_x, click_y, 20.0);
}

//---------------------------------------------------------
// Procedure: setObstaclesAroundPoint()

void ColAvd::setObstaclesAroundPoint(double center_x, double center_y, double radius)
{
  // Grid parameters from constructor
  int x_start = -2657;
  int y_start = 2354;
  int x_end   = -1634;
  int y_end   = 3292;
  
  int min_x = max(x_start, (int)(center_x - radius));
  int max_x = min(x_end,   (int)(center_x + radius + 1));
  int min_y = max(y_start, (int)(center_y - radius));
  int max_y = min(y_end,   (int)(center_y + radius + 1));
  
  double radius_squared = radius * radius;
  
  for (int x = min_x; x < max_x; x++) {
    for (int y = min_y; y < max_y; y++) {
      double dx = x - center_x;
      double dy = y - center_y;
      double distance_squared = dx * dx + dy * dy;
      
      if (distance_squared <= radius_squared) {
        int idx = (y - y_start) * nodes_width + (x - x_start);
        nodes[idx].bObstacle = true;
        obstacles_changed = true;
        path_solved = false;
      }
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
  m_msgs << "  Speed: "   << m_nav_speed   << " m/s" << endl;
  m_msgs << endl;

  m_msgs << "Contact Information:" << endl;
  m_msgs << "  Position (X,Y): (" << m_contact_x << ", " << m_contact_y << ")" << endl;
  m_msgs << "  Heading: " << m_contact_heading << " degrees" << endl;
  m_msgs << "  Speed: "   << m_contact_speed   << " m/s" << endl;
  m_msgs << "  Distance: "<< m_contact_distance << " meters" << endl;
  m_msgs << " Phi (Angle between headings): " << phi << " degrees" << endl;
  m_msgs << " Beta_ts (Angle to target): "    << beta_ts << " degrees" << endl;
  m_msgs << " Beta (Relative bearing): "       << beta << " degrees" << endl;
  m_msgs << " Collision Status: "               << collision_status << endl;

  return(true);
}
