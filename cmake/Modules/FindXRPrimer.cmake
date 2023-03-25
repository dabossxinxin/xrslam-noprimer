
FIND_PATH(XRPrimer_INCLUDE_DIR NAMES xrprimer_export.h HINTS ${CMAKE_SOURCE_DIR}/../../SDK/xrprimer/include)
FIND_FILE(XRPrimer_DEBUG_LIB xrprimer.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/xrprimer/lib)
FIND_FILE(XRPrimer_RELEASE_LIB xrprimer.lib HINTS ${CMAKE_SOURCE_DIR}/../../SDK/xrprimer/lib)
