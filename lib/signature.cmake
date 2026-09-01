# © 2023 Unit Circle Inc.
# SPDX-License-Identifier: Apache-2.0

# Since this file is brought in via include(), we do the work in a
# function to avoid polluting the top-level scope.

function(zephyr_runner_file type path)
  # Property magic which makes west flash choose the signed build
  # output of a given type.
  set_target_properties(runners_yaml_props_target PROPERTIES "${type}_file" "${path}")
endfunction()

function(zephyr_sbl_tasks)
  # Extensionless prefix of any output file.
  set(output ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME})

  # List of additional build byproducts.
  set(byproducts)

  # Set up .bin outputs.
  if(CONFIG_BUILD_OUTPUT_BIN)
    list(APPEND byproducts ${output}.signed.bin)
    zephyr_runner_file(bin ${output}.signed.bin)
    set(BYPRODUCT_KERNEL_SIGNED_BIN_NAME "${output}.signed.bin"
        CACHE FILEPATH "Signed kernel bin file" FORCE
    )
  endif()

  # Set up .hex outputs.
  if(CONFIG_BUILD_OUTPUT_HEX)
    list(APPEND byproducts ${output}.signed.hex)
    zephyr_runner_file(hex ${output}.signed.hex)
    set(BYPRODUCT_KERNEL_SIGNED_HEX_NAME "${output}.signed.hex"
        CACHE FILEPATH "Signed kernel hex file" FORCE
    )
  endif()

  if(DEFINED ENV{UC_SIGNATURE_KEY})
    set(keyarg $ENV{UC_SIGNATURE_KEY})
    set(keyfile "")
  else()
    set(keyfile "${CONFIG_UC_SIGNATURE_KEY_FILE}")
    if("${keyfile}" STREQUAL "")
      # No signature key file
      message(FATAL_ERROR "No signature key file provided. Set CONFIG_UC_SIGNATURE_KEY_FILE to correct path to key file")
    endif()
  endif()

  if(DEFINED ENV{UC_SIGNATURE_CERT})
    set(certfile $ENV{UC_SIGNATURE_CERT})
  else()
    set(certfile "${CONFIG_UC_SIGNATURE_CERT_FILE}")
  endif()
  if("${certfile}" STREQUAL "")
    # No cert file
    message(FATAL_ERROR "No certificate file provided. Set CONFIG_UC_SIGNATURE_CERT_FILE to correct path to certificate file")
  endif()

  foreach(file keyfile certfile)
    if(NOT "${${file}}" STREQUAL "")
      if(NOT IS_ABSOLUTE "${${file}}")
        # Relative paths are relative to 'west topdir'.
        set(${file} "${WEST_TOPDIR}/${${file}}")
      endif()

      if(NOT EXISTS "${${file}}")
        message(FATAL_ERROR "uc sign can't find file ${${file}} (Note: Relative paths are relative to the west workspace topdir \"${WEST_TOPDIR}\")")
      elseif(NOT (CONFIG_BUILD_OUTPUT_BIN OR CONFIG_BUILD_OUTPUT_HEX))
        message(FATAL_ERROR "Can't sign images: Neither CONFIG_BUILD_OUTPUT_BIN nor CONFIG_BUILD_OUTPUT_HEX is enabled, so there's nothing to sign.")
      endif()
    endif()
  endforeach()

  if(NOT "${keyfile}" STREQUAL "")
    set(keyarg ${keyfile})
  endif()

  # CMake guarantees that multiple COMMANDs given to
  # add_custom_command() are run in order, so adding the 'west sign'
  # calls to the "extra_post_build_commands" property ensures they run
  # after the commands which generate the unsigned versions.
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    echo "Signing ${output} CERT ${certfile}")
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${UCLOG_ROOT_DIR}/scripts/sbl.py sign --key "${keyarg}" --code ${output}.hex --cert "${certfile}" ${output}.signed.hex)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands COMMAND
    ${UCLOG_ROOT_DIR}/scripts/sbl.py sign --key "${keyarg}" --code ${output}.bin --cert "${certfile}" ${output}.signed.bin)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts ${byproducts})
endfunction()

zephyr_sbl_tasks()
