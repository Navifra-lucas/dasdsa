set (FORDER_NAME parameter_information)

# 복사할 헤더 파일 리스트
set (EXPORT_HEADER_FILES
        motion_parameter.hpp   
        param_msg.hpp  
    )

    # 헤더를 복사하는 매크로
macro (copy_headers MODULE_NAME OUT_DIR_PREFIX FILE_LIST)

    foreach (HEADER_FILE ${FILE_LIST})
        # 파일 명을 취득
        get_filename_component (FNAME ${HEADER_FILE} NAME)
        # 파일 명의 일부를 사용하여 타겟 명을 생성
        get_filename_component (TEMP_TARGET ${HEADER_FILE} NAME_WE)
        set (TEMP_TARGET copy_header_${OUT_DIR_PREFIX}_${TEMP_TARGET})
        # 복사하는 명령어
        add_custom_target (
            ${TEMP_TARGET}
            COMMAND
            ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/${TARGET_NAME}/${OUT_DIR_PREFIX}/${HEADER_FILE}
                                     ${GLOBAL_HEADER_DIR}/${PROJECT_NAME}/${TARGET_NAME}/${OUT_DIR_PREFIX}/${FNAME}
        )
        # 의존 관계를 지정(라이브러리 빌드 전에 복사가 실행되도록 한다)
        add_dependencies (${MODULE_NAME} ${TEMP_TARGET})
    endforeach (HEADER_FILE ${FILE_LIST})

endmacro (copy_headers MODULE_NAME OUT_DIR_PREFIX FILE_LIST)


# 매크로이므로 인수에 리스트를 넘기 때는 ""으로 감싸는 것에 주의.
copy_headers (${PROJECT_NAME} ${FORDER_NAME} "${EXPORT_HEADER_FILES}")