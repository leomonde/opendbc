project('opendbc', ['c', 'cpp', 'cython'],
  version : '0.2.1',
  default_options : [
    'c_std=gnu11',
    'cpp_std=c++17',
    'warning_level=2',
    'werror=true',
  ]
)

# Add Python dependency
py_mod = import('python')
py = py_mod.find_installation('python3')
py_dep = py.dependency()

# Find NumPy dependency
numpy_dep = dependency('numpy', required: false)
if not numpy_dep.found()
  message('NumPy not found via pkg-config, getting include path directly')
endif

# Common compiler flags
common_flags = [
  '-fPIC',
  '-O2',
  '-Wunused',
  '-Wshadow',
  '-Wno-sign-compare',       # Fix sign comparison warnings
  '-Wno-missing-field-initializers', # Fix missing initializer warnings
  '-Wno-unused-parameter',   # Fix unused parameter warnings
]

# Sanitizer options
if get_option('b_sanitize') == 'address'
  common_flags += ['-fno-omit-frame-pointer']
endif

# Base C/C++ dependencies
base_dependencies = [
  py_dep,
]

# Use local include directories only
base_includes = include_directories('.')

# Configure CAN library
dbc_path_rel = 'opendbc/dbc'
dbc_file_path = '-DDBC_FILE_PATH="@0@/@1@"'.format(meson.current_source_dir(), dbc_path_rel)

# Subdirectories
subdir('opendbc/can')
subdir('opendbc/dbc')

# Only build tests if not minimal build
if get_option('build_tests')
  subdir('opendbc/safety/tests/libsafety')
endif