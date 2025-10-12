#--------------------------------------------------------
# The CMakeLists.txt for:                  pColAvd_astar
# Author(s):                              Douglas Lima
#--------------------------------------------------------

SET(SRC
  ColAvd.cpp
  ColAvd_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pColAvd_astar ${SRC})

TARGET_LINK_LIBRARIES(pColAvd_astar
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   genutil
   m
   pthread)

