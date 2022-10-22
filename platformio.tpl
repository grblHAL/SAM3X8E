#
# Template for Web Builder, SAM3X8E
#

[platformio]
src_dir = src
include_dir = src

[common]
build_flags   =
  -g3 
  -fmax-errors=5
  -D OVERRIDE_MY_MACHINE
lib_archive   = no
lib_deps      = 
extra_scripts =
src_filter    = +<src/*>

#
# Default values apply to all 'env:' prefixed environments  
#
[env]
framework     = arduino
extra_scripts = ${common.extra_scripts}
build_flags   = ${common.build_flags}
lib_deps      = ${common.lib_deps}
monitor_speed = 250000
monitor_flags =

[env:%env_name%]
platform = atmelsam
board = due
build_flags = ${env.build_flags}
%build_flags%
lib_deps = ${env.lib_deps}
