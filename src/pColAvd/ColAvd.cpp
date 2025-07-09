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

  //Giving some default values to nodes start and end
  //(-2363,2678) -> node start
  //(-1676,2908) -> node end
  //How to put a node like nodestart with coordinates (-2363,2678) and nodeend with coordinates (-1676,2908)
  node_start = &nodes[(2678 - y_start) * width + (-2363 - x_start)];
  node_end  = &nodes[(2908 - y_start) * width + (-1676 - x_start)];

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
  // Usd only for debugging purposes
  /*if (!nodes_visualized) {
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
  }*/

  // Visualize start and end nodes
  if (!nodes_visualized) {
    // Visualize start node (red)
    XYPoint start_point(node_start->x, node_start->y);
    start_point.set_label("node_start");
    start_point.set_vertex_size(8);
    start_point.set_color("fill_color", "red");
    string start_point_str = start_point.get_spec();
    Notify("VIEW_POINT", start_point_str);

    // Visualize end node (green)
    XYPoint end_point(node_end->x, node_end->y);
    end_point.set_label("node_end");
    end_point.set_vertex_size(8);
    end_point.set_color("fill_color", "green");
    string end_point_str = end_point.get_spec();
    Notify("VIEW_POINT", end_point_str);

    // Create segment list connecting start and end nodes
    /*XYSegList path_seglist;
    path_seglist.add_vertex(node_start->x, node_start->y);
    path_seglist.add_vertex(node_end->x, node_end->y);
    path_seglist.set_label("path");
    path_seglist.set_edge_color("black");
    path_seglist.set_edge_size(2);
    string seglist_str = path_seglist.get_spec();
    Notify("VIEW_SEGLIST", seglist_str);*/

    nodes_visualized = true;

    
  }

  // Solve A* algorithm to find the shortest path (always solve for now, but could be optimized)
  Solve_AStar();

  // Reset obstacles_changed flag after solving
  obstacles_changed = false;

  // After solving A*, visualize the path if needed
  // Draw Path by starting at the end, and following the parent node trail
  // back to the start - the start node will not have a parent path to follow
  if (node_end != nullptr)
  {
    // Create a vector to store the path nodes in reverse order
    vector<sNode*> path_nodes;
    sNode *p = node_end;
    
    // Trace back from end to start
    while (p != nullptr)
    {
      path_nodes.push_back(p);
      p = p->parent;
    }
    
    // Create SEGLIST for the A* path
    if (path_nodes.size() > 1)
    {
      XYSegList astar_path;
      // Add vertices in reverse order (start to end)
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

  AppCastingMOOSApp::PostReport();
  return(true);
}

void ColAvd::Solve_AStar()
{
  // A* algorithm implementation goes here
  // Update the graph, all the parents, and the nodes, global goals and local goals accordingly
		for (int x = 0; x < nodes_width; x++)
			for (int y = 0; y < nodes_height; y++)
			{
				nodes[y*nodes_width + x].bVisited = false;
				nodes[y*nodes_width + x].fGlobalGoal = INFINITY;
				nodes[y*nodes_width + x].fLocalGoal = INFINITY;
				nodes[y*nodes_width + x].parent = nullptr;	// No parents
			}
  
  // Pitagoras for distance between two nodes
  auto distance = [](sNode* a, sNode* b) // For convenience
		{
			return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
		};

  // Calculate the heuristic (For standard A* is just the distance again)
  auto heuristic = [distance](sNode* a, sNode* b) // So we can experiment with heuristic
		{
			return distance(a, b);
		};

  // Setup starting conditions
  sNode *nodeCurrent = node_start; // Start node
  node_start->fLocalGoal = 0.0f; // Zero local
  node_start->fGlobalGoal = heuristic(node_start, node_end); 

  // Add start node to not tested list - this will ensure it gets tested.
  // As the algorithm progresses, newly discovered nodes get added to this
  // list, and will themselves be tested later
  list<sNode*> listNotTestedNodes;
  listNotTestedNodes.push_back(node_start);


  // if the not tested list contains nodes, there may be better paths
  // which have not yet been explored. However, we will also stop 
  // searching when we reach the target - there may well be better
  // paths but this one will do - it wont be the longest.
  while (!listNotTestedNodes.empty() && nodeCurrent != node_end)// Find absolutely shortest path // && nodeCurrent != nodeEnd)
		{
			// Sort Untested nodes by global goal, so lowest is first
			listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal; } );
			
			// Front of listNotTestedNodes is potentially the lowest distance node. Our
			// list may also contain nodes that have been visited, so ditch these...
			while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
				listNotTestedNodes.pop_front();

			// ...or abort because there are no valid nodes left to test
			if (listNotTestedNodes.empty())
				break;

			nodeCurrent = listNotTestedNodes.front();
			nodeCurrent->bVisited = true; // We only explore a node once
			
					
			// Check each of this node's neighbours...
			for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
			{
				// ... and only if the neighbour is not visited and is 
				// not an obstacle, add it to NotTested List
				if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
					listNotTestedNodes.push_back(nodeNeighbour);

				// Calculate the neighbours potential lowest parent distance
				float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

				// If choosing to path through this node is a lower distance than what 
				// the neighbour currently has set, update the neighbour to use this node
				// as the path source, and set its distance scores as necessary
				if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
				{
					nodeNeighbour->parent = nodeCurrent;
					nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

					// The best path length to the neighbour being tested has changed, so
					// update the neighbour's score. The heuristic is used to globally bias
					// the path algorithm, so it knows if its getting better or worse. At some
					// point the algo will realise this path is worse and abandon it, and then go
					// and search along the next best path.
					nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, node_end);
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
    
    if(param == "x") {
      click_x = strtod(value.c_str(), 0);
    }
    else if(param == "y") {
      click_y = strtod(value.c_str(), 0);
    }
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
  int x_end = -1634;
  int y_end = 3292;
  
  // Check all nodes in the grid
  for (int x = x_start; x < x_end; x++) {
    for (int y = y_start; y < y_end; y++) {
      // Calculate distance from node to clicked point
      double distance = hypot(x - center_x, y - center_y);
      
      if (distance <= radius) {
        // Mark node as obstacle
        int idx = (y - y_start) * nodes_width + (x - x_start);
        nodes[idx].bObstacle = true;
        obstacles_changed = true;
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




