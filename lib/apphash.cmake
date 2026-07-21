# © 2025 Unit Circle Inc.
# SPDX-License-Identifier: Apache-2.0

# Since this file is brought in via include(), we do the work in a
# function to avoid polluting the top-level scope.

function(zephyr_hash_tasks)
  # Extensionless prefix of any output file.
  set(output ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME})

  # We want to add hash before any other post build commands run (like STM32 signing tool)
  get_property(previous_post_build_commands
    GLOBAL PROPERTY
    extra_post_build_commands
    )

  # CMake guarantees that multiple COMMANDs given to
  # add_custom_command() are run in order, so adding the 'west sign'
  # calls to the "extra_post_build_commands" property ensures they run
  # after the commands which generate the unsigned versions.

  # No APPEND on the first command, to reset the property
  set_property(GLOBAL PROPERTY extra_post_build_commands COMMAND
    echo "Adding apphash to ${output}.elf")
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${UCLOG_ROOT_DIR}/scripts/hash.py -f -i ${output}.bin -o ${output}.hash)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${CMAKE_OBJCOPY} --update-section .apphash=${output}.hash ${output}.elf ${output}.elf)

  # variables used by zephyr/CMakeLists.txt for bin/hex generation
  set(remove_sections_argument_list "")
  foreach(section .comment COMMON)
    list(APPEND remove_sections_argument_list
      $<TARGET_PROPERTY:bintools,elfconvert_flag_section_remove>${section})
  endforeach()

  set(gap_fill_prop "$<TARGET_PROPERTY:bintools,elfconvert_flag_gapfill>")
  set(gap_fill "$<$<BOOL:${gap_fill_prop}>:${gap_fill_prop}${CONFIG_BUILD_GAP_FILL_PATTERN}>")

  # re-generate bin and hex files
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${CMAKE_OBJCOPY} -O binary ${gap_fill} --keep-section=.apphash ${remove_sections_argument_list} ${output}.elf ${output}.bin)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${CMAKE_OBJCOPY} -O ihex $<$<BOOL:${CONFIG_BUILD_OUTPUT_HEX_GAP_FILL}>:${gap_fill}> --keep-section=.apphash ${remove_sections_argument_list} ${output}.elf ${output}.hex)

  # Add previous post build commands back after our commands
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands ${previous_post_build_commands})

endfunction()

zephyr_hash_tasks()
