# CAN library

can_sources = [
  'dbc.cc',
  'parser.cc',
  'packer.cc',
  'common.cc',
]

# Set platform-specific link flags
system = host_machine.system()

# Get NumPy include path for current environment
numpy_inc_cmd = run_command(py, ['-c', 'import numpy; print(numpy.get_include())'], check: true)
numpy_inc_path = numpy_inc_cmd.stdout().strip()
numpy_inc_args = '-I' + numpy_inc_path

# CAN library specific flags
can_lib_flags = common_flags + [dbc_file_path, numpy_inc_args]

# Create shared library
libdbc = shared_library('dbc',
  sources: can_sources,
  cpp_args: can_lib_flags,
  include_directories: base_includes,
  dependencies: base_dependencies,
  install: true,
)

# Create Cython extensions
cython_args = common_flags + ['-Wno-#warnings', '-Wno-shadow', '-Wno-deprecated-declarations']

# Setup link flags for Cython extensions
link_args = []
if system == 'darwin'
  link_args += ['-bundle', '-undefined', 'dynamic_lookup']
elif system == 'linux'
  if host_machine.cpu_family() == 'aarch64'
    link_args += ['-shared']
  else
    link_args += ['-pthread', '-shared']
  endif
endif

# Manually compile Cython files to C++ instead of through meson
# This is necessary because meson's automatic Cython handling seems to have issues with our code

parser_cpp = custom_target('parser_pyx_cpp',
  input: 'parser_pyx.pyx',
  output: 'parser_pyx.cpp',
  command: [find_program('cython'), '-3', '--cplus', '@INPUT@', '-o', '@OUTPUT@']
)

packer_cpp = custom_target('packer_pyx_cpp',
  input: 'packer_pyx.pyx',
  output: 'packer_pyx.cpp',
  command: [find_program('cython'), '-3', '--cplus', '@INPUT@', '-o', '@OUTPUT@']
)

# Generate parser Python extension
parser_ext = py.extension_module('parser_pyx',
  parser_cpp,
  link_with: libdbc,
  cpp_args: cython_args + ['-std=c++17'],
  link_args: link_args,
  dependencies: base_dependencies,
  include_directories: base_includes,
  install: true,
)

# Generate packer Python extension
packer_ext = py.extension_module('packer_pyx',
  packer_cpp,
  link_with: libdbc,
  cpp_args: cython_args + ['-std=c++17'],
  link_args: link_args,
  dependencies: base_dependencies,
  include_directories: base_includes,
  install: true,
)