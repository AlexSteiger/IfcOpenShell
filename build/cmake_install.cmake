# Install script for directory: /home/alex/Documents/IfcOpenShell/cmake

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/ifcparse/Argument.h;/usr/local/include/ifcparse/ArgumentType.h;/usr/local/include/ifcparse/Ifc2x3-latebound.h;/usr/local/include/ifcparse/Ifc2x3.h;/usr/local/include/ifcparse/Ifc2x3enum.h;/usr/local/include/ifcparse/IfcBaseClass.h;/usr/local/include/ifcparse/IfcCharacterDecoder.h;/usr/local/include/ifcparse/IfcEntityDescriptor.h;/usr/local/include/ifcparse/IfcEntityInstanceData.h;/usr/local/include/ifcparse/IfcEntityList.h;/usr/local/include/ifcparse/IfcException.h;/usr/local/include/ifcparse/IfcFile.h;/usr/local/include/ifcparse/IfcGlobalId.h;/usr/local/include/ifcparse/IfcHierarchyHelper.h;/usr/local/include/ifcparse/IfcLogger.h;/usr/local/include/ifcparse/IfcParse.h;/usr/local/include/ifcparse/IfcSIPrefix.h;/usr/local/include/ifcparse/IfcSpfHeader.h;/usr/local/include/ifcparse/IfcSpfStream.h;/usr/local/include/ifcparse/IfcWrite.h;/usr/local/include/ifcparse/ifc_parse_api.h;/usr/local/include/ifcparse/utils.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/ifcparse" TYPE FILE FILES
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/Argument.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/ArgumentType.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/Ifc2x3-latebound.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/Ifc2x3.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/Ifc2x3enum.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcBaseClass.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcCharacterDecoder.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcEntityDescriptor.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcEntityInstanceData.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcEntityList.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcException.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcFile.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcGlobalId.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcHierarchyHelper.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcLogger.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcParse.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcSIPrefix.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcSpfHeader.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcSpfStream.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/IfcWrite.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/ifc_parse_api.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcparse/utils.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/ifcgeom/IfcGeom.h;/usr/local/include/ifcgeom/IfcGeomElement.h;/usr/local/include/ifcgeom/IfcGeomFilter.h;/usr/local/include/ifcgeom/IfcGeomIterator.h;/usr/local/include/ifcgeom/IfcGeomIteratorSettings.h;/usr/local/include/ifcgeom/IfcGeomMaterial.h;/usr/local/include/ifcgeom/IfcGeomRenderStyles.h;/usr/local/include/ifcgeom/IfcGeomRepresentation.h;/usr/local/include/ifcgeom/IfcGeomShapeType.h;/usr/local/include/ifcgeom/IfcGeomTree.h;/usr/local/include/ifcgeom/IfcRegister.h;/usr/local/include/ifcgeom/IfcRegisterConvertCurve.h;/usr/local/include/ifcgeom/IfcRegisterConvertFace.h;/usr/local/include/ifcgeom/IfcRegisterConvertShape.h;/usr/local/include/ifcgeom/IfcRegisterConvertShapes.h;/usr/local/include/ifcgeom/IfcRegisterConvertWire.h;/usr/local/include/ifcgeom/IfcRegisterCreateCache.h;/usr/local/include/ifcgeom/IfcRegisterDef.h;/usr/local/include/ifcgeom/IfcRegisterGeomHeader.h;/usr/local/include/ifcgeom/IfcRegisterPurgeCache.h;/usr/local/include/ifcgeom/IfcRegisterShapeType.h;/usr/local/include/ifcgeom/IfcRegisterUndef.h;/usr/local/include/ifcgeom/IfcRepresentationShapeItem.h;/usr/local/include/ifcgeom/ifc_geom_api.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/ifcgeom" TYPE FILE FILES
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeom.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomElement.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomFilter.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomIterator.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomIteratorSettings.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomMaterial.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomRenderStyles.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomRepresentation.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomShapeType.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcGeomTree.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegister.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterConvertCurve.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterConvertFace.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterConvertShape.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterConvertShapes.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterConvertWire.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterCreateCache.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterDef.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterGeomHeader.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterPurgeCache.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterShapeType.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRegisterUndef.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/IfcRepresentationShapeItem.h"
    "/home/alex/Documents/IfcOpenShell/cmake/../src/ifcgeom/ifc_geom_api.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libIfcParse.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/home/alex/Documents/IfcOpenShell/build/libIfcParse.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libIfcGeom.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/home/alex/Documents/IfcOpenShell/build/libIfcGeom.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/IfcConvert" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/IfcConvert")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/IfcConvert"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/IfcConvert")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/alex/Documents/IfcOpenShell/build/IfcConvert")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/IfcConvert" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/IfcConvert")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/IfcConvert"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/IfcConvert")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/IfcGeomServer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/IfcGeomServer")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/IfcGeomServer"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/IfcGeomServer")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/alex/Documents/IfcOpenShell/build/IfcGeomServer")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/IfcGeomServer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/IfcGeomServer")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/IfcGeomServer"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/IfcGeomServer")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/alex/Documents/IfcOpenShell/build/ifcwrap/cmake_install.cmake")
  include("/home/alex/Documents/IfcOpenShell/build/examples/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/alex/Documents/IfcOpenShell/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
