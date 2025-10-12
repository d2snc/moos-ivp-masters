#--------------------------------------------------------
# The CMakeLists.txt for:                       pColAvd_vo
# Author(s):                              Douglas Lima
#--------------------------------------------------------

SET(SRC
  ColAvd_vo.cpp
  ColAvd_vo_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pColAvd_vo ${SRC})

TARGET_LINK_LIBRARIES(pColAvd_vo
   ${MOOS_LIBRARIES}
   apputil
   contacts
   geometry
   mbutil
   genutil
   logic
   ivpbuild
   m
   pthread)

