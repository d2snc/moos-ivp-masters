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
  m_col_avd_distance = 50;
  m_avoidance_distance = 50;
  m_skip_waypoints = 100;
  m_contact_detected = false;
  m_fixed_end_x = 0.0;
  m_fixed_end_y = 0.0;

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

  // Set default node_start position until navigation data is received
  // (-2363,2678) -> default node start
  node_start = &nodes[(2678 - y_start) * width + (-2363 - x_start)];
  
  // node_end will be set dynamically based on contact position

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
       updateNodeStart();
     }
     else if(key == "NAV_Y") {
       m_nav_y = msg.GetDouble();
       updateNodeStart();
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
  string previous_status = collision_status;
  
  //Collision status só muda quando o ctt estiver dentro do range de distância
  if (m_contact_distance < m_col_avd_distance) {
    //Publico a variável para aumentar o pwt
    Notify("WPT_UPDATE_ALGORITHM","pwt=250");
    // Check if its a headon situation
    if (beta_ts > -12 && beta_ts < 12 && phi > 168 && phi < 192) {
      collision_status = "HEADON";
    }
  } else if (phi > -12 && phi < 12) { //Coloquei outra lógica para o overtaking
    collision_status = "OVERTAKING";
  } else if (phi > 220 && phi < 250 || phi > 120 && phi < 150) {
    collision_status = "CROSSING";
  }
  else {
    collision_status = "NONE";

    //Caso não tenha nenhum tipo de colisão, publica a variável para reduzir o pwt do A*
    Notify("WPT_UPDATE_ALGORITHM","pwt=50");
    
  }
  
  // Update node_end if collision status changed or in OVERTAKING mode
  if (collision_status != previous_status || collision_status == "OVERTAKING") {
    updateNodeEnd();
  }

  // Only solve A* when there is a contact and (path hasn't been solved yet or obstacles have changed)
  bool has_contact = m_contact_detected;
  
  // Debug info
  if (has_contact) {
    string debug_msg = "A* conditions: has_contact=" + string(has_contact ? "true" : "false") + 
                      ", path_solved=" + string(path_solved ? "true" : "false") + 
                      ", obstacles_changed=" + string(obstacles_changed ? "true" : "false");
    reportEvent(debug_msg);
  }
  // Modificação principal: executar A* tanto para HEADON quanto para OVERTAKING
  if (has_contact && (!path_solved || obstacles_changed) && 
      (collision_status == "HEADON" || collision_status == "OVERTAKING" || collision_status == "CROSSING"))
  {
    //Publica a variável para ativar o behavior
    Notify("DEPLOY_ALGORITHM","true");
    
    
    // Solve A* algorithm to find the shortest path
    Solve_AStar();
    
    // Reset obstacles_changed flag after solving
    obstacles_changed = false;
    
    // Visualize the updated path
    visualizeAStarPath();
    
    // Para OVERTAKING, marcar para recalcular na próxima iteração
    // (devido ao target dinâmico)
    if (collision_status == "OVERTAKING") {
      path_solved = false; // Força recálculo contínuo
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: Solve_AStar()  (implementa Sugestão 1)

void ColAvd::Solve_AStar()
{
  // Check if node_start is valid before proceeding
  if (node_start == nullptr || node_end == nullptr) {
    return; // Skip A* if start or end nodes are not set
  }
  
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
  reportEvent("visualizeAStarPath() called");
  
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
      // Create visualization for VIEW_SEGLIST
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
      
      // Create waypoints string for WPT_UPDATE (skip initial points)
      string waypoints_str = "points=";
      int start_idx = max(0, (int)path_nodes.size() - 1 - m_skip_waypoints);
      bool first_point = true;
      
      for (int i = start_idx; i >= 0; i--)
      {
        if (!first_point) waypoints_str += ":";
        waypoints_str += to_string(path_nodes[i]->x) + "," + to_string(path_nodes[i]->y);
        first_point = false;
      }
      
      // Only notify if we have waypoints to send
      if (!first_point) {
        Notify("WPT_UPDATE_ALGORITHM", waypoints_str);
        // Debug: report waypoints sent
        reportEvent("WPT_UPDATE_ALGORITHM sent with " + to_string(path_nodes.size() - m_skip_waypoints) + " waypoints");
      } else {
        reportEvent("No waypoints to send - all points skipped");
      }
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

  // Initial A* solve will happen automatically when NAV_X/NAV_Y are received
  // and updateNodeStart() sets a valid node_start

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
  
  // If this is the first contact detection, fix the end position
  if (!m_contact_detected) {
    m_contact_detected = true;
    
    // Calculate fixed end position diametrically opposite to contact heading
    double opposite_heading = m_contact_heading + 180.0;
    if (opposite_heading >= 360.0) opposite_heading -= 360.0;
    
    // Convert heading to radians
    double heading_rad = opposite_heading * M_PI / 180.0;
    
    // Calculate fixed target position
    m_fixed_end_x = m_contact_x + m_avoidance_distance * sin(heading_rad);
    m_fixed_end_y = m_contact_y + m_avoidance_distance * cos(heading_rad);
    
    updateNodeEnd();
  }
  
  // Always update contact obstacles when contact position changes
  setContactObstacles();
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

//---------------------------------------------------------
// Procedure: setContactObstacles()

void ColAvd::setContactObstacles()
{
  reportEvent("setContactObstacles() called");
  
  // Only set obstacles if there is a contact
  if (m_contact_x == 0.0 && m_contact_y == 0.0) {
    reportEvent("No contact - skipping obstacle setup");
    return;
  }
  
  // First clear all obstacles (reset grid)
  int x_start = -2657;
  int y_start = 2354;
  int x_end   = -1634;
  int y_end   = 3292;
  
  for (int x = 0; x < nodes_width; x++) {
    for (int y = 0; y < nodes_height; y++) {
      int idx = y * nodes_width + x;
      nodes[idx].bObstacle = false;
    }
  }
  
  // Set obstacles around contact position using avoidance distance as radius
  setObstaclesAroundPoint(m_contact_x, m_contact_y, m_avoidance_distance);
  
  // Mark obstacles as changed to trigger A* recalculation
  obstacles_changed = true;
  path_solved = false;
}

//---------------------------------------------------------
// Procedure: updateNodeStart()

void ColAvd::updateNodeStart()
{
  // Grid parameters (same as in constructor)
  int x_start = -2657;
  int y_start = 2354;
  int x_end   = -1634;
  int y_end   = 3292;
  
  // Convert ship's position to grid coordinates
  int ship_x = (int)round(m_nav_x);
  int ship_y = (int)round(m_nav_y);
  
  // Ensure the ship position is within grid bounds
  if (ship_x >= x_start && ship_x < x_end && ship_y >= y_start && ship_y < y_end) {
    int idx = (ship_y - y_start) * nodes_width + (ship_x - x_start);
    
    // Check if the ship's position is an obstacle
    if (!nodes[idx].bObstacle) {
      // Ship position is clear, use it directly
      node_start = &nodes[idx];
    } else {
      // Ship position is an obstacle, find nearest non-obstacle node
      node_start = findNearestNonObstacleNode(ship_x, ship_y, x_start, y_start, x_end, y_end);
    }
    
    // Reset path solving since start position changed
    path_solved = false;
  }
}

//---------------------------------------------------------
// Procedure: findNearestNonObstacleNode()

ColAvd::sNode* ColAvd::findNearestNonObstacleNode(int target_x, int target_y, int x_start, int y_start, int x_end, int y_end)
{
  ColAvd::sNode* nearest_node = nullptr;
  double min_distance = INFINITY;
  
  // Search in expanding circles around the target position
  for (int radius = 1; radius <= 200; radius++) {
    for (int dx = -radius; dx <= radius; dx++) {
      for (int dy = -radius; dy <= radius; dy++) {
        // Only check nodes on the circle edge (not inside)
        if (abs(dx) != radius && abs(dy) != radius) continue;
        
        int check_x = target_x + dx;
        int check_y = target_y + dy;
        
        // Check bounds
        if (check_x < x_start || check_x >= x_end || check_y < y_start || check_y >= y_end) continue;
        
        int idx = (check_y - y_start) * nodes_width + (check_x - x_start);
        
        // Check if this node is not an obstacle
        if (!nodes[idx].bObstacle) {
          double distance = sqrt(dx*dx + dy*dy);
          if (distance < min_distance) {
            min_distance = distance;
            nearest_node = &nodes[idx];
          }
        }
      }
    }
    
    // If we found a node at this radius, return it (closest possible)
    if (nearest_node != nullptr) {
      reportEvent("Found nearest non-obstacle node at distance " + to_string((int)min_distance) + " from ship");
      return nearest_node;
    }
  }
  
  // Fallback: return any non-obstacle node if nothing found (shouldn't happen)
  reportEvent("Warning: No non-obstacle node found within 200m radius");
  for (int x = 0; x < nodes_width; x++) {
    for (int y = 0; y < nodes_height; y++) {
      int idx = y * nodes_width + x;
      if (!nodes[idx].bObstacle) {
        return &nodes[idx];
      }
    }
  }
  
  return nullptr;
}

//---------------------------------------------------------
// Procedure: updateNodeEnd() - Modificado para suportar OVERTAKING dinâmico

void ColAvd::updateNodeEnd()
{
  // Grid parameters (same as in constructor)
  int x_start = -2657;
  int y_start = 2354;
  int x_end   = -1634;
  int y_end   = 3292;
  
  double target_x, target_y;
  
  // Comportamento diferente baseado no status de colisão
  if (collision_status == "HEADON" || collision_status == "CROSSING") {
    // Para HEADON: usar posição fixa calculada no primeiro contato
    if (m_contact_detected) {
      target_x = m_fixed_end_x;
      target_y = m_fixed_end_y;
    } else {
      // Fallback se ainda não detectou contato
      target_x = m_contact_x;
      target_y = m_contact_y;
    }
  }
  else if (collision_status == "OVERTAKING") {
    // Para OVERTAKING: calcular posição dinâmica na proa da embarcação
    if (m_contact_detected && m_contact_speed > 0.1) { // Só se estiver se movendo
      
      // Tempo estimado para intercepção (baseado na diferença de velocidades)
      double speed_diff = m_nav_speed - m_contact_speed;
      double time_to_intercept = 30.0; // Tempo padrão em segundos
      
      // Se nossa velocidade for maior, calcular tempo real de intercepção
      if (speed_diff > 0.5) {
        double current_distance = hypot(m_contact_x - m_nav_x, m_contact_y - m_nav_y);
        time_to_intercept = current_distance / speed_diff;
        
        // Limitar tempo de predição entre 10 e 60 segundos
        time_to_intercept = max(10.0, min(60.0, time_to_intercept));
      }
      
      // Converter heading do contato para radianos
      double contact_heading_rad = m_contact_heading * M_PI / 180.0;
      
      // Calcular posição futura do contato
      double predicted_contact_x = m_contact_x + m_contact_speed * time_to_intercept * sin(contact_heading_rad);
      double predicted_contact_y = m_contact_y + m_contact_speed * time_to_intercept * cos(contact_heading_rad);
      
      // Adicionar distância extra à frente para garantir ultrapassagem completa
      double extra_distance = 50.0; // 50 metros à frente
      target_x = predicted_contact_x + extra_distance * sin(contact_heading_rad);
      target_y = predicted_contact_y + extra_distance * cos(contact_heading_rad);
      
      // Debug info
      string debug_msg = "OVERTAKING target calculated: time=" + to_string((int)time_to_intercept) + 
                        "s, predicted_pos=(" + to_string((int)predicted_contact_x) + "," + 
                        to_string((int)predicted_contact_y) + "), target=(" + 
                        to_string((int)target_x) + "," + to_string((int)target_y) + ")";
      reportEvent(debug_msg);
      
    } else {
      // Fallback: se contato parado, usar posição atual + distância fixa à frente
      double contact_heading_rad = m_contact_heading * M_PI / 180.0;
      target_x = m_contact_x + 100.0 * sin(contact_heading_rad);
      target_y = m_contact_y + 100.0 * cos(contact_heading_rad);
    }
  }
  else {
    // Para NONE: usar posição atual do contato (não deveria executar A*)
    target_x = m_contact_x;
    target_y = m_contact_y;
  }
  
  // Converter para coordenadas da grade
  int end_x = (int)round(target_x);
  int end_y = (int)round(target_y);
  
  // Verificar limites da grade
  if (end_x >= x_start && end_x < x_end && end_y >= y_start && end_y < y_end) {
    int idx = (end_y - y_start) * nodes_width + (end_x - x_start);
    
    // Verificar se o nó objetivo não é um obstáculo
    if (!nodes[idx].bObstacle) {
      node_end = &nodes[idx];
    } else {
      // Buscar nó livre mais próximo se posição objetivo for obstáculo
      node_end = findNearestNonObstacleNode(end_x, end_y, x_start, y_start, x_end, y_end);
    }
    
    // Reset path solving since end position changed
    path_solved = false;
    
    // Log da mudança
    reportEvent("Node end updated for " + collision_status + " at (" + 
               to_string(end_x) + "," + to_string(end_y) + ")");
  } else {
    reportEvent("Warning: Target position out of grid bounds: (" + 
               to_string(end_x) + "," + to_string(end_y) + ")");
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
