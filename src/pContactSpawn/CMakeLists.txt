#--------------------------------------------------------
# The CMakeLists.txt for:                       pContactSpawn
# Author(s):                              Douglas Lima
#--------------------------------------------------------

SET(SRC
  ContactSpawn.cpp
  ContactSpawn_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pContactSpawn ${SRC})

TARGET_LINK_LIBRARIES(pContactSpawn
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

