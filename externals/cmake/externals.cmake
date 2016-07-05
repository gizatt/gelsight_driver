if(NOT USE_SYSTEM_LCM)
  set(lcm_proj lcm)
  set(lcm_url https://github.com/lcm-proj/lcm/releases/download/v1.3.0/lcm-1.3.0.zip)
  set(lcm_download_hash 5d46a902fe04608809af3dc526909f9b)
  set(lcm_depends)
  set(lcm_external_args
    CONFIGURE_COMMAND ${source_prefix}/lcm/configure --prefix=${CMAKE_INSTALL_PREFIX}
    BUILD_IN_SOURCE 1
    )
endif()

set(bot_core_lcmtypes_url https://github.com/iamwolf/bot_core_lcmtypes.git)
set(bot_core_lcmtypes_revision c29cd6076d13ca2a3ecc23ffcbe28a0a1ab46314)
set(bot_core_lcmtypes_depends ${lcm_proj})

# set(libbot_url https://github.com/openhumanoids/libbot.git)
# set(libbot_revision ed4a76423f2a21594436490341f907710d3f78dd)
# set(libbot_depends bot_core_lcmtypes ${lcm_proj})

set(Eigen_pod_url https://github.com/RobotLocomotion/eigen-pod.git)
set(Eigen_pod_revision 20a633e8f0a285edf9dcc5a06cc9e80774920a7f)
set(Eigen_pod_depends)

# if(NOT USE_SYSTEM_OPENCV)
#   set(opencv_proj opencv)
#   set(opencv_url https://github.com/Itseez/opencv.git)
#   set(opencv_revision 2.4.12.3)
#   set(opencv_depends Eigen_pod)
#   set(opencv_external_args
#     CMAKE_CACHE_ARGS
#       ${default_cmake_args}
#       ${python_args}
#       -DWITH_CUDA:BOOL=OFF
#     )
# endif()

# set(snopt_url ssh://git@github.com/openhumanoids/snopt.git)
# set(snopt_revision 95d908275156f2665ef3941f08cb89c767480a6e)
# set(snopt_depends)

# set(gurobi-private_url ssh://git@github.com/openhumanoids/gurobi-private.git)
# set(gurobi-private_revision cfeea24766ea1a11d5fc6eeff193ab520c3e58d2)
# set(gurobi-private_depends)
# set(gurobi-private_external_args ${download_only_args})
#
# set(gurobi_url https://github.com/RobotLocomotion/gurobi.git)
# set(gurobi_revision b95a186b4d988db00ada55bd8efb08c651a83fe7)
# if(APPLE)
#   set(gurobi_distro_file ${source_prefix}/gurobi-private/gurobi5.6.2_mac64.pkg)
# else()
#   set(gurobi_distro_file ${source_prefix}/gurobi-private/gurobi5.6.2_linux64.tar.gz)
# endif()
# set(gurobi_environment_args GUROBI_DISTRO=${gurobi_distro_file})
# set(gurobi_depends gurobi-private)

# set(cmake_scripts_url https://github.com/RobotLocomotion/cmake.git)
# set(cmake_scripts_revision 54f5a4c0734015695334970ecedac79e12c3116f)
# set(cmake_scripts_external_args
#   ${download_only_args}
#   SOURCE_DIR ${source_prefix}/../drake/drake/cmake
#   )

set(yaml_cpp_url https://github.com/jbeder/yaml-cpp.git)
set(yaml_cpp_revision 57805dfd6a741e55477ea8d4d5b3b6f0272d1f0e)
set(yaml_cpp_external_args
  CMAKE_CACHE_ARGS
    ${default_cmake_args}
    -DBUILD_SHARED_LIBS:BOOL=ON
  )

set(externals
  Eigen_pod
  ${lcm_proj}
  bot_core_lcmtypes
#   libbot
#  ${opencv_proj}
#   cmake_scripts
  yaml_cpp
  )

if(BUILD_PRIVATE_EXTERNALS)

#   list(APPEND externals
#     gurobi-private
#     gurobi
#     snopt
#     )

endif()


if(NOT APPLE)

## Nothing to do here, anymore

endif()

macro(add_external proj)

  # depending on which variables are defined, the external project
  # might be mercurial, svn, git, or an archive download.
  if(DEFINED ${proj}_hg_tag)
    set(download_args
      HG_REPOSITORY ${${proj}_url}
      HG_TAG ${${proj}_hg_tag})
  elseif(DEFINED ${proj}_svn_revision)
    set(download_args
    SVN_REPOSITORY ${${proj}_url}
    SVN_REVISION -r ${${proj}_revision})
  elseif(DEFINED ${proj}_download_hash)
    set(download_args
    URL ${${proj}_url}
    URL_MD5 ${${proj}_download_hash})
  else()
    set(download_args
      GIT_REPOSITORY ${${proj}_url}
      GIT_TAG ${${proj}_revision})
  endif()

  # if this variable is not defined then this external will be treated as
  # a standard pod so we'll define the required configure and build commands
  if(NOT DEFINED ${proj}_external_args)
    set(pod_build_args
      CONFIGURE_COMMAND ${empty_command}
      INSTALL_COMMAND ${empty_command}
      BUILD_COMMAND $(MAKE) BUILD_PREFIX=${CMAKE_INSTALL_PREFIX} BUILD_TYPE=${CMAKE_BUILD_TYPE} ${${proj}_environment_args}
      BUILD_IN_SOURCE 1
    )
    set(${proj}_external_args ${pod_build_args})
    set(${proj}_is_pod TRUE)
  endif()

  # this supports non-standard download locations
  set(source_dir_arg)
  list(FIND ${proj}_external_args SOURCE_DIR res)
  if (res EQUAL -1)
    set(source_dir_arg SOURCE_DIR ${source_prefix}/${proj})
  endif()

  # workaround a cmake issue, we need to support empty strings as list elements
  # so replace the string NONE with empty string here right before arg conversion
  # and then the variable will be quoted in the following call to ExternalProject_Add.
  string(REGEX REPLACE "NONE" "" ${proj}_external_args "${${proj}_external_args}")

  ExternalProject_Add(${proj}
    DEPENDS ${${proj}_depends}
    ${download_args}
    ${source_dir_arg}
    "${${proj}_external_args}"
    )

endmacro()

foreach(external ${externals})
  add_external(${external})
endforeach()
