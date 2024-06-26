# Copyright 2024 NXP
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_MIMX8QX6_ADSP)
  zephyr_include_directories(adsp)
  add_subdirectory(adsp)

  zephyr_sources(
    adsp/pinctrl_soc.h
  )

  #  west sign

  # See detailed comments in soc/intel/intel_adsp/common/CMakeLists.txt
  add_custom_target(zephyr.ri ALL
    DEPENDS ${CMAKE_BINARY_DIR}/zephyr/zephyr.ri
  )

  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/zephyr/zephyr.ri
    COMMENT "west sign --if-tool-available --tool rimage ..."
    COMMAND  west sign --if-tool-available --tool rimage --build-dir ${CMAKE_BINARY_DIR} ${WEST_SIGN_OPTS}
    DEPENDS ${CMAKE_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
  )

  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/adsp/linker.ld CACHE INTERNAL "")
endif()

if(CONFIG_SOC_MIMX8QX6_A35)
  zephyr_include_directories(a35)
  add_subdirectory(a35)

  zephyr_sources(
    a35/pinctrl_soc.h
    a35/soc.c
  )

  zephyr_sources_ifdef(CONFIG_ARM_MMU a35/mmu_regions.c)

  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm64/scripts/linker.ld CACHE INTERNAL "")
endif()
