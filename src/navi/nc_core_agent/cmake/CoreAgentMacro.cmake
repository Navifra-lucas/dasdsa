macro(NAVIFRA_SOURCES_PLAT out name platform)
    source_group("${name}\\Source Files" FILES ${ARGN})
    list(APPEND ${out} ${ARGN})
    if(NOT(${platform}))
        set_source_files_properties(${ARGN} PROPERTIES HEADER_FILE_ONLY TRUE)
    endif()
endmacro()

macro(NAVIFRA_SOURCES out name)
    NAVIFRA_SOURCES_PLAT( ${out} ${name} ON ${ARGN})
endmacro()

macro(NAVIFRA_SOURCES_AUTO out)
    NAVIFRA_SOURCES_AUTO_PLAT( ${out} ON ${ARGN})
endmacro()

macro(NAVIFRA_SOURCES_AUTO_PLAT out platform)
    foreach(f ${ARGN})
        get_filename_component(fname ${f} NAME)

        # Read the package name from the source file
        file(STRINGS ${f} package REGEX "// Package: (.*)")
        if(package)
            string(REGEX REPLACE ".*: (.*)" "\\1" name ${package})

            # Files of the Form X_UNIX.cpp are treated as headers
            if(${fname} MATCHES ".*_.*\\..*")
                #message(STATUS "Platform: ${name} ${f} ${platform}")
                NAVIFRA_SOURCES_PLAT( ${out} ${name} OFF ${f})
            else()
                #message(STATUS "Source: ${name} ${f} ${platform}")
                NAVIFRA_SOURCES_PLAT( ${out} ${name} ${platform} ${f})
            endif()
        else()
            #message(STATUS "Source: Unknown ${f} ${platform}")
            NAVIFRA_SOURCES_PLAT( ${out} Unknown ${platform} ${f})
        endif()
    endforeach()
endmacro()