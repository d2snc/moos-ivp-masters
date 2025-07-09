/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: ColAvd.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ColAvd_HEADER
#define ColAvd_HEADER

#include <vector> 
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"


class ColAvd : public AppCastingMOOSApp
{
 private:
  struct sNode
	{
		bool bObstacle = false;			// Is the node an obstruction?
		bool bVisited = false;			// Have we searched this node before?
		float fGlobalGoal;				// Distance to goal so far
		float fLocalGoal;				// Distance to goal if we took the alternative route
		int x;							// Nodes position in 2D space
		int y;
		std::vector<sNode*> vecNeighbours;	// Connections to neighbours
		sNode* parent;					// Node connecting to this node that offers shortest parent
	};

 public:
   ColAvd();
   ~ColAvd();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void parseNodeReport(const std::string& node_report);

 private: // Configuration variables

 private: // State variables
  double m_nav_heading;
  double m_nav_speed;
  double m_nav_x;
  double m_nav_y;
  
  double m_contact_x;
  double m_contact_y;
  double m_contact_heading;
  double m_contact_speed;
  double m_contact_distance;
  double m_col_avd_distance;
  double phi;
  double beta_ts;
  double beta;
  std::string collision_status;
  
  // A* algorithm nodes
  sNode *nodes;
  sNode *node_start;
  sNode *node_end;
  int nodes_width;
  int nodes_height;
  bool nodes_visualized;
};

#endif 
