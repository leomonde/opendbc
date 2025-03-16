# Safety tests

# Determine compiler based on platform
system = host_machine.system()
safety_cc = 'gcc'
if system == 'darwin'
  safety_cc = 'gcc-13' # Use homebrew GCC on macOS
endif

# Override compiler if mutation testing is enabled
if get_option('mutation')
  safety_cc = 'clang-17'
endif

# Setup compiler flags for safety library
safety_flags = [
  '-Wall',
  '-Wextra', 
  '-Werror',
  '-nostdlib',
  '-fno-builtin',
  '-std=gnu11',
  '-Wfatal-errors',
  '-Wno-pointer-to-int-cast',
]

# Add mutation flags if enabled
if get_option('mutation')
  mutation_flags = [
    '-fprofile-instr-generate',
    '-fcoverage-mapping',
    '-fpass-plugin=/usr/lib/mull-ir-frontend-17',
    '-g',
    '-grecord-command-line',
  ]
  safety_flags += mutation_flags
endif

# Add UBSAN flags if sanitizers are enabled
if get_option('b_sanitize') == 'undefined'
  safety_flags += [
    '-fsanitize=undefined',
    '-fno-sanitize-recover=undefined',
  ]
endif

# Add coverage flags if enabled
if get_option('coverage')
  safety_flags += [
    '-fprofile-arcs', 
    '-ftest-coverage', 
    '-fprofile-abs-path',
  ]
  # Need to link with gcov for coverage
  gcov_dep = declare_dependency(link_args: ['-lgcov'])
else
  gcov_dep = dependency('', required: false)
endif

# Compile the safety library
safety_includes = include_directories('.', '../../board/', '../../')

# For the safety library, we need to link with libc for printf
libc_dep = meson.get_compiler('c').find_library('c')

safety_obj = static_library('safety_obj',
  'safety.c',
  c_args: safety_flags,
  include_directories: safety_includes,
)

libsafety = shared_library('safety',
  objects: safety_obj.extract_all_objects(recursive: true),
  c_args: safety_flags, 
  link_args: safety_flags,
  dependencies: [gcov_dep, libc_dep],
  include_directories: safety_includes,
)