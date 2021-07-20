if(${PACKAGE_VERSION} VERSION_LESS "8.0")
  add_llvm_loadable_module(OptSched OBJECT ${OPT_SCHED_SRCS})
else()
  add_llvm_library(OptSched MODULE OBJECT ${OPT_SCHED_SRCS})
endif()

set(OPTSCHED_EXTRA_DEFINITIONS "-DINSERT_ON_STEPFRWRD -DDUMMY")

add_dependencies(OptSched ${OPT_SCHED_TARGET_DEPS})

add_definitions(${OPTSCHED_EXTRA_DEFINITIONS})

add_compile_options(-g)
