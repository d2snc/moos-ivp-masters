#--------------------------------------------------------
# The CMakeLists.txt for:                       pColAvd
# Author(s):                              Douglas Lima
#--------------------------------------------------------

SET(SRC
  ColAvd.cpp
  ColAvd_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pColAvd ${SRC})

TARGET_LINK_LIBRARIES(pColAvd
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   genutil
   m
   pthread)

