build_lib_example(
  NAME gpsr-paper-scenario
  SOURCE_FILES gpsr-paper-scenario.cc
  LIBRARIES_TO_LINK
    ${libgpsr}
    ${libinternet}
    ${libwifi}
    ${libmobility}
    ${libflow-monitor}
)
