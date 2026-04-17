# © 2024 Unit Circle Inc.
# SPDX-License-Identifier: Apache-2.0

# Since this file is brought in via include(), we do the work in a
# function to avoid polluting the top-level scope.

function(zephyr_log_tasks)
  # Extensionless prefix of any output file.
  set(output ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME})

  if(CONFIG_UC_SIGNED_IMAGE)
    set(extra_cache_args)
  else()
    set(extra_cache_args -f --hashfile ${output}.hash)
  endif()

  # CMake guarantees that multiple COMMANDs given to
  # add_custom_command() are run in order, so adding the 'west sign'
  # calls to the "extra_post_build_commands" property ensures they run
  # after the commands which generate the unsigned versions.
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    echo "Build logdata ${output}.logdata")
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${UCLOG_ROOT_DIR}/scripts/logdata.py --ofile ${output}.logdata ${output}.elf)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    echo "Add logdata to ${output}.elf")
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${CMAKE_OBJCOPY} --add-section .logdata_cbor=${output}.logdata ${output}.elf)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    echo "Cache logdata ${output}.logdata")
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${UCLOG_ROOT_DIR}/scripts/cachelogdata.py --bin ${output}.bin --logdata ${output}.logdata ${extra_cache_args})
endfunction()

zephyr_log_tasks()

