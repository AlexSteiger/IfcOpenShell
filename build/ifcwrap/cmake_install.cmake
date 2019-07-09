# Install script for directory: /home/alex/Documents/IfcOpenShell/src/ifcwrap

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
   "/usr/lib/python3/dist-packages/ifcopenshell/ifcopenshell_wrapper.py;/usr/lib/python3/dist-packages/ifcopenshell/__init__.py;/usr/lib/python3/dist-packages/ifcopenshell/entity_instance.py;/usr/lib/python3/dist-packages/ifcopenshell/file.py;/usr/lib/python3/dist-packages/ifcopenshell/guid.py;/usr/lib/python3/dist-packages/ifcopenshell/main.py;/usr/lib/python3/dist-packages/ifcopenshell/template.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/lib/python3/dist-packages/ifcopenshell" TYPE FILE FILES
    "/home/alex/Documents/IfcOpenShell/build/ifcwrap/ifcopenshell_wrapper.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/__init__.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/entity_instance.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/file.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/guid.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/main.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/template.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/python3/dist-packages/ifcopenshell/geom/__init__.py;/usr/lib/python3/dist-packages/ifcopenshell/geom/app.py;/usr/lib/python3/dist-packages/ifcopenshell/geom/code_editor_pane.py;/usr/lib/python3/dist-packages/ifcopenshell/geom/main.py;/usr/lib/python3/dist-packages/ifcopenshell/geom/occ_utils.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/lib/python3/dist-packages/ifcopenshell/geom" TYPE FILE FILES
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/geom/__init__.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/geom/app.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/geom/code_editor_pane.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/geom/main.py"
    "/home/alex/Documents/IfcOpenShell/src/ifcwrap/../ifcopenshell-python/ifcopenshell/geom/occ_utils.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/lib/python3/dist-packages/ifcopenshell" TYPE MODULE FILES "/home/alex/Documents/IfcOpenShell/build/ifcwrap/_ifcopenshell_wrapper.so")
  if(EXISTS "$ENV{DESTDIR}/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/lib/python3/dist-packages/ifcopenshell/_ifcopenshell_wrapper.so")
    endif()
  endif()
endif()

