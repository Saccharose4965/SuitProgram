# components/storage/CMakeLists.txt
idf_component_register(
    SRCS storage_sd.c
    INCLUDE_DIRS include
    REQUIRES hw vfs fatfs sdmmc
)

