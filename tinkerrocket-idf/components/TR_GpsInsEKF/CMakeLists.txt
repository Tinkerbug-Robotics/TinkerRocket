# TR_RocketComputerTypes: EkfGnssFeed.h takes the converter's GNSSDataSI record (#1107).
idf_component_register(SRCS "TR_GpsInsEKF.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
