# Resolve Bluedroid private headers (needed for sniff policy tweak)
set(BTA_PRIV_DIR "${IDF_PATH}/components/bt/host/bluedroid/bta/include")
if(NOT EXISTS "${BTA_PRIV_DIR}")
    set(BTA_PRIV_DIR "${CMAKE_CURRENT_LIST_DIR}/../../esp-idf/components/bt/host/bluedroid/bta/include")
endif()

set(BT_COMMON_PRIV_DIR "${IDF_PATH}/components/bt/host/bluedroid/common/include")
if(NOT EXISTS "${BT_COMMON_PRIV_DIR}")
    set(BT_COMMON_PRIV_DIR "${CMAKE_CURRENT_LIST_DIR}/../../esp-idf/components/bt/host/bluedroid/common/include")
endif()

set(BT_BASE_PRIV_DIR "${IDF_PATH}/components/bt/common/include")
if(NOT EXISTS "${BT_BASE_PRIV_DIR}")
    set(BT_BASE_PRIV_DIR "${CMAKE_CURRENT_LIST_DIR}/../../esp-idf/components/bt/common/include")
endif()

set(BT_STACK_PRIV_DIR "${IDF_PATH}/components/bt/host/bluedroid/stack/include")
if(NOT EXISTS "${BT_STACK_PRIV_DIR}")
    set(BT_STACK_PRIV_DIR "${CMAKE_CURRENT_LIST_DIR}/../../esp-idf/components/bt/host/bluedroid/stack/include")
endif()

set(BT_DM_PRIV_DIR "${IDF_PATH}/components/bt/host/bluedroid/bta/dm/include")
if(NOT EXISTS "${BT_DM_PRIV_DIR}")
    set(BT_DM_PRIV_DIR "${CMAKE_CURRENT_LIST_DIR}/../../esp-idf/components/bt/host/bluedroid/bta/dm/include")
endif()

get_filename_component(PROJECT_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

idf_component_register(
    SRCS "bt_audio.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS
        "${BTA_PRIV_DIR}"
        "${BT_COMMON_PRIV_DIR}"
        "${BT_BASE_PRIV_DIR}"
        "${BT_STACK_PRIV_DIR}"
        "${BT_DM_PRIV_DIR}"
        "${PROJECT_ROOT_DIR}/main"
    REQUIRES bt nvs_flash state freertos audio audio_player input oled ui_shell
)
