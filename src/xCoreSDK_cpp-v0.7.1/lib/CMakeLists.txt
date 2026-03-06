# Add shared/static libraries
add_library(${PROJECT_NAME}_lib SHARED IMPORTED GLOBAL)
add_library(${PROJECT_NAME}_static STATIC IMPORTED GLOBAL)
add_library(xmatemodel_lib STATIC IMPORTED)

# Upgrade libraries
add_library(${PROJECT_NAME}-Upgrade_lib SHARED IMPORTED GLOBAL)
add_library(${PROJECT_NAME}-Upgrade_static STATIC IMPORTED GLOBAL)

if(WIN32)
    if(${CMAKE_SIZEOF_VOID_P} EQUAL 8) #64bit
        set(_ARCH_NAME "64bit")
    elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 4) #32bit
        set(_ARCH_NAME "32bit")
    endif()

    set_target_properties(xmatemodel_lib PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/64bit/xMateModel.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/64bit/xMateModeld.lib)
    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        IMPORTED_IMPLIB_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/${PROJECT_NAME}.lib
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/${PROJECT_NAME}.dll
        IMPORTED_IMPLIB_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/${PROJECT_NAME}.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/${PROJECT_NAME}.dll
        )
    set_target_properties(${PROJECT_NAME}_static PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/${PROJECT_NAME}_static.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/${PROJECT_NAME}_static.lib
        INTERFACE_LINK_LIBRARIES xmatemodel_lib
        )
    set_target_properties(${PROJECT_NAME}-Upgrade_lib PROPERTIES
        IMPORTED_IMPLIB_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/${PROJECT_NAME}_Upgrade.lib
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/${PROJECT_NAME}_Upgrade.dll
        IMPORTED_IMPLIB_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/${PROJECT_NAME}_Upgrade.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/${PROJECT_NAME}_Upgrade.dll
        INTERFACE_LINK_LIBRARIES ${PROJECT_NAME}_lib
    )
    set_target_properties(${PROJECT_NAME}-Upgrade_static PROPERTIES
        IMPORTED_LOCATION_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Release/${_ARCH_NAME}/${PROJECT_NAME}_Upgrade_static.lib
        IMPORTED_LOCATION_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/Windows/Debug/${_ARCH_NAME}/${PROJECT_NAME}_Upgrade_static.lib
        INTERFACE_LINK_LIBRARIES ${PROJECT_NAME}_static
    )
elseif(UNIX)
    if(NOT (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64"))
        message(FATAL_ERROR "Unsupported: target ${CMAKE_SYSTEM_PROCESSOR}")
    endif ()

    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        INTERFACE_LINK_LIBRARIES xmatemodel_lib)

    set(XCORESDK_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Linux/${CMAKE_SYSTEM_PROCESSOR})
    # set path to model library
    set_target_properties(xmatemodel_lib PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxMateModel.a)

    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}.so.${CMAKE_PROJECT_VERSION}
        )
    set_target_properties(${PROJECT_NAME}_static PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/lib${PROJECT_NAME}.a
        INTERFACE_LINK_LIBRARIES xmatemodel_lib
        )

    # Controller upgrade
    set_target_properties(${PROJECT_NAME}-Upgrade_lib PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK_Upgrade.so.0.1
    )
    set_target_properties(${PROJECT_NAME}-Upgrade_static PROPERTIES
        IMPORTED_LOCATION ${XCORESDK_LIBRARY_DIR}/libxCoreSDK_Upgrade.a
        INTERFACE_LINK_LIBRARIES ${PROJECT_NAME}_static
    )

endif()

if(XCORE_LINK_SHARED_LIBS)
    add_library(Rokae::Rokae ALIAS ${CMAKE_PROJECT_NAME}_lib)
    add_library(Rokae::Upgrade ALIAS ${PROJECT_NAME}-Upgrade_lib)
else()
    add_library(Rokae::Rokae ALIAS ${CMAKE_PROJECT_NAME}_static)
    add_library(Rokae::Upgrade ALIAS ${PROJECT_NAME}-Upgrade_static)
endif()

# install runtime artifacts
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.21")
    install(IMPORTED_RUNTIME_ARTIFACTS ${PROJECT_NAME}_lib ${PROJECT_NAME}-Upgrade_lib
        RUNTIME
        DESTINATION ${CMAKE_INSTALL_BINDIR})
else()
    string(TOUPPER ${CMAKE_BUILD_TYPE} build_type_upper)
    get_target_property(runtime_dll_location ${PROJECT_NAME}_lib IMPORTED_LOCATION_${build_type_upper})
    if(${runtime_dll_location} MATCHES "NOTFOUND$")
        get_target_property(runtime_dll_location ${PROJECT_NAME}_lib IMPORTED_LOCATION)
    endif()
    if(NOT ${runtime_dll_location} MATCHES "NOTFOUND$")
        install(FILES ${runtime_dll_location} DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
endif()
