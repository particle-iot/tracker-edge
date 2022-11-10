# Place this custom makefile here: <project-path>/src/build.mk

INCLUDE_DIRS += $(SOURCE_PATH)/$(USRSRC)  # add user sources to include path
# add C and CPP files - if USRSRC is not empty, then add a slash
CPPSRC += $(call target_files,$(USRSRC_SLASH),*.cpp)
CSRC += $(call target_files,$(USRSRC_SLASH),*.c)

# Remove certain libraries based on device OS version
# ---------------------------------------------
ifeq ($(shell test $(VERSION) -ge 5000; echo $$?),0)
$(info $$MODULE_LIBSV2 is [${MODULE_LIBSV2}])
MODULE_LIBSV2_FILTERED:=$(filter-out $(SOURCE_PATH)lib/memfault,$(MODULE_LIBSV2))
MODULE_LIBSV2=$(MODULE_LIBSV2_FILTERED)
$(info $$MODULE_LIBSV2 is [${MODULE_LIBSV2}])
endif
