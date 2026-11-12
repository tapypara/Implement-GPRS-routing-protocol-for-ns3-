build_lib(
  LIBNAME gpsr
  SOURCE_FILES
    helper/gpsr-helper.cc
    model/gpsr-routing-protocol.cc
  HEADER_FILES
    helper/gpsr-helper.h
    model/gpsr-routing-protocol.h
  LIBRARIES_TO_LINK
    ${libinternet}
    ${libwifi}
    ${libmobility}
    ${libnetwork}
    ${libflow-monitor}
)
