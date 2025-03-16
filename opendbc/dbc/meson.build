# DBC Generator

# Find the generator script
python_script = find_program('generator/generator.py')

# Use a simpler approach - specify input files directly if needed
# Define a custom target to generate DBC files
generated_dbcs = custom_target('generated_dbcs',
  output: 'generated_dbcs_stamp',
  command: [
    'sh', '-c', 
    'python3 @0@ && touch @OUTPUT@'.format(meson.current_source_dir() / 'generator/generator.py')
  ],
  build_by_default: true,
  depend_files: [
    'generator/generator.py',
    'generator/honda/_bosch_2018.dbc',
    'generator/honda/_bosch_adas_2018.dbc',
    'generator/honda/_community.dbc',
    'generator/honda/_honda_common.dbc',
    'generator/honda/_nidec_common.dbc',
    'generator/honda/_steering_sensors_a.dbc',
    'generator/honda/_steering_sensors_b.dbc',
    'generator/toyota/_community.dbc',
    'generator/toyota/_toyota_2017.dbc',
    'generator/toyota/_toyota_adas_standard.dbc',
  ],
)