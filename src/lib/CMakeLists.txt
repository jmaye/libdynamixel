remake_include()

remake_add_directories(com)
remake_add_directories(exceptions)
remake_pkg_config_generate(EXTRA_CFLAGS -std=c++0x)
