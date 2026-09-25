if(NOT DEFINED BOX2D_SOURCE_DIR)
  message(FATAL_ERROR "BOX2D_SOURCE_DIR must point to the fetched Box2D source")
endif()

set(constants_file "${BOX2D_SOURCE_DIR}/src/constants.h")
file(READ "${constants_file}" constants_contents)

set(original_definition "#define B2_LINEAR_SLOP ( 0.005f * b2_lengthUnitsPerMeter )")
set(reduced_definition "#define B2_LINEAR_SLOP ( 0.0025f * b2_lengthUnitsPerMeter )")
string(FIND "${constants_contents}" "${original_definition}" original_index)
if(original_index GREATER_EQUAL 0)
  string(REPLACE "${original_definition}" "${reduced_definition}" constants_contents "${constants_contents}")
elseif(NOT constants_contents MATCHES "#define B2_LINEAR_SLOP \\( 0\\.0025f")
  message(FATAL_ERROR "Unexpected B2_LINEAR_SLOP definition in ${constants_file}")
endif()

file(WRITE "${constants_file}" "${constants_contents}")