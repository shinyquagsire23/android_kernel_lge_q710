#Android makefile to build kernel as a part of Android Build
PERL		= perl

KERNEL_TARGET := $(strip $(INSTALLED_KERNEL_TARGET))
ifeq ($(KERNEL_TARGET),)
INSTALLED_KERNEL_TARGET := $(PRODUCT_OUT)/kernel
endif

TARGET_KERNEL_ARCH := $(strip $(TARGET_KERNEL_ARCH))
ifeq ($(TARGET_KERNEL_ARCH),)
KERNEL_ARCH := arm
else
KERNEL_ARCH := $(TARGET_KERNEL_ARCH)
endif

TARGET_KERNEL_HEADER_ARCH := $(strip $(TARGET_KERNEL_HEADER_ARCH))
ifeq ($(TARGET_KERNEL_HEADER_ARCH),)
KERNEL_HEADER_ARCH := $(KERNEL_ARCH)
else
$(warning Forcing kernel header generation only for '$(TARGET_KERNEL_HEADER_ARCH)')
KERNEL_HEADER_ARCH := $(TARGET_KERNEL_HEADER_ARCH)
endif

KERNEL_HEADER_DEFCONFIG := $(strip $(KERNEL_HEADER_DEFCONFIG))
ifeq ($(KERNEL_HEADER_DEFCONFIG),)
KERNEL_HEADER_DEFCONFIG := $(KERNEL_DEFCONFIG)
endif

# Force 32-bit binder IPC for 64bit kernel with 32bit userspace
ifeq ($(KERNEL_ARCH),arm64)
ifeq ($(TARGET_ARCH),arm)
KERNEL_CONFIG_OVERRIDE := CONFIG_ANDROID_BINDER_IPC_32BIT=y
endif
endif

TARGET_KERNEL_CROSS_COMPILE_PREFIX := $(strip $(TARGET_KERNEL_CROSS_COMPILE_PREFIX))
ifeq ($(TARGET_KERNEL_CROSS_COMPILE_PREFIX),)
KERNEL_CROSS_COMPILE := arm-eabi-
else
KERNEL_CROSS_COMPILE := $(TARGET_KERNEL_CROSS_COMPILE_PREFIX)
endif

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_GCC_NOANDROID_CHK := $(shell (echo "int main() {return 0;}" | $(KERNEL_CROSS_COMPILE)gcc -E -mno-android - > /dev/null 2>&1 ; echo $$?))
ifeq ($(strip $(KERNEL_GCC_NOANDROID_CHK)),0)
KERNEL_CFLAGS := KCFLAGS=-mno-android
endif

mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
current_dir := $(notdir $(patsubst %/,%,$(dir $(mkfile_path))))
ifeq ($(TARGET_KERNEL_VERSION),)
    TARGET_KERNEL_VERSION := 3.18
endif
TARGET_KERNEL := msm-$(TARGET_KERNEL_VERSION)
ifeq ($(TARGET_KERNEL),$(current_dir))
    # New style, kernel/msm-version
    BUILD_ROOT_LOC := ../../
    TARGET_KERNEL_SOURCE := kernel/$(TARGET_KERNEL)
    KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/kernel/$(TARGET_KERNEL)
    KERNEL_SYMLINK := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
    KERNEL_USR := $(KERNEL_SYMLINK)/usr
    KERNEL_HEADERS_TIMESTAMP := $(KERNEL_SYMLINK)/usr/build-timestamp
else
    # Legacy style, kernel source directly under kernel
    KERNEL_LEGACY_DIR := true
    BUILD_ROOT_LOC := ../
    TARGET_KERNEL_SOURCE := kernel
    KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
    KERNEL_USR :=
    KERNEL_HEADERS_TIMESTAMP := $(KERNEL_OUT)/usr/build-timestamp
endif

KERNEL_CONFIG := $(KERNEL_OUT)/.config

ifeq ($(KERNEL_DEFCONFIG)$(wildcard $(KERNEL_CONFIG)),)
$(error Kernel configuration not defined, cannot build kernel)
else

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/Image
else
ifeq ($(KERNEL_ARCH),arm64)
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/Image.gz
else
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/zImage
endif
endif

ifeq ($(TARGET_KERNEL_APPEND_DTB), true)
$(info Using appended DTB)
TARGET_PREBUILT_INT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)-dtb
endif

KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_INSTALL := system
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules

TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)

define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

ifneq ($(KERNEL_LEGACY_DIR),true)
$(KERNEL_USR): $(KERNEL_HEADERS_INSTALL)
	rm -rf $(KERNEL_SYMLINK)
	ln -s kernel/$(TARGET_KERNEL) $(KERNEL_SYMLINK)

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_USR)
endif

KERNEL_SOURCE_FILES := $(call find-files-in-subdirs, $(TARGET_KERNEL_SOURCE), "*", .)
KERNEL_SOURCE_FILES := $(addprefix $(TARGET_KERNEL_SOURCE)/, $(KERNEL_SOURCE_FILES))

ifeq ($(INIT_BOOTCHART2), true)
KERNEL_CONFIG_OVERRIDE_FILES := bootchart2_defconfig
endif

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_SOURCE_FILES | $(KERNEL_OUT)
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_DEFCONFIG)
	$(hide) if [ ! -z "$(KERNEL_CONFIG_OVERRIDE)" ]; then \
			echo "Overriding kernel config with '$(KERNEL_CONFIG_OVERRIDE)'"; \
			for override_config in $(KERNEL_CONFIG_OVERRIDE); \
				do echo $$override_config >> $(KERNEL_OUT)/.config; done; \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) oldconfig; fi
	$(hide) if [ ! -z "$(KERNEL_CONFIG_OVERRIDE_FILES)" ]; then \
			echo "Overriding kernel config with '$(KERNEL_CONFIG_OVERRIDE_FILES)'"; \
			for override_file in $(KERNEL_CONFIG_OVERRIDE_FILES); \
				do cat $(TARGET_KERNEL_SOURCE)/arch/$(KERNEL_ARCH)/configs/$$override_file >> $(KERNEL_OUT)/.config; done; \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) $(KERNEL_MAKE_ENV) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) oldconfig; fi

ifeq ($(PRODUCT_SUPPORT_EXFAT), y)
TUXERA_MODULES_OUT := $(TARGET_OUT_VENDOR)/lib/modules
sinclude $(ANDROID_BUILD_TOP)/device/lge/common/tuxera.mk
endif

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_HEADERS_INSTALL)  $(KERNEL_SOURCE_FILES) | $(KERNEL_OUT)
	$(hide) echo "Building kernel..."
	$(hide) rm -rf $(KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/dts
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_CFLAGS)
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_CFLAGS) modules
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) INSTALL_MOD_PATH=$(BUILD_ROOT_LOC)../$(KERNEL_MODULES_INSTALL) INSTALL_MOD_STRIP=1 ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) modules_install
ifeq ($(PRODUCT_SUPPORT_EXFAT), y)
	@mkdir -p $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)
	@mkdir -p $(ANDROID_BUILD_TOP)/$(TARGET_OUT_EXECUTABLES)
	@cp -f $(ANDROID_BUILD_TOP)/kernel/msm-3.18/tuxera_update.sh $(ANDROID_BUILD_TOP)
	@sh tuxera_update.sh --target target/lg.d/$(TUXERA_TARGET) --use-cache --latest --max-cache-entries 2 --source-dir $(ANDROID_BUILD_TOP)/kernel/msm-3.18 --output-dir $(ANDROID_BUILD_TOP)/$(KERNEL_OUT) $(SUPPORT_EXFAT_TUXERA)
	@tar -xzf tuxera-*.tgz
	@cp $(ANDROID_BUILD_TOP)/tuxera-*/exfat/kernel-module/texfat.ko $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)
	@cp $(ANDROID_BUILD_TOP)/tuxera-*/exfat/tools/* $(ANDROID_BUILD_TOP)/$(TARGET_OUT_EXECUTABLES)
	perl $(ANDROID_BUILD_TOP)/kernel/msm-3.18/scripts/sign-file sha1 $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.priv $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.x509 $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)/texfat.ko
ifeq ($(PRODUCT_SUPPORT_NTFS), y)
	@cp $(ANDROID_BUILD_TOP)/tuxera-*/ntfs/kernel-module/tntfs.ko $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)
	@cp $(ANDROID_BUILD_TOP)/tuxera-*/ntfs/tools/* $(ANDROID_BUILD_TOP)/$(TARGET_OUT_EXECUTABLES)
	perl $(ANDROID_BUILD_TOP)/kernel/msm-3.18/scripts/sign-file sha1 $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.priv $(ANDROID_BUILD_TOP)/$(KERNEL_OUT)/signing_key.x509 $(ANDROID_BUILD_TOP)/$(TUXERA_MODULES_OUT)/tntfs.ko
endif
	@rm -f kheaders*.tar.bz2
	@rm -f tuxera-*.tgz
	@rm -rf tuxera-*
	@rm -f tuxera_update.sh
endif
	$(mv-modules)
	$(clean-module-folder)

$(KERNEL_HEADERS_TIMESTAMP): $(KERNEL_HEADERS_INSTALL) $(KERNEL_USR)

$(KERNEL_HEADERS_INSTALL): $(KERNEL_SOURCE_FILES) | $(KERNEL_OUT)
	$(hide) rm -rf $(KERNEL_HEADERS_INSTALL)
	$(hide) if [ ! -z "$(KERNEL_HEADER_DEFCONFIG)" ]; then \
			rm -f $(BUILD_ROOT_LOC)$(KERNEL_CONFIG); \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_HEADER_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_HEADER_DEFCONFIG); \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_HEADER_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) headers_install; fi
	$(hide) if [ "$(KERNEL_HEADER_DEFCONFIG)" != "$(KERNEL_DEFCONFIG)" ]; then \
			echo "Used a different defconfig for header generation"; \
			rm -f $(BUILD_ROOT_LOC)$(KERNEL_CONFIG); \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_DEFCONFIG); fi
	$(hide) if [ ! -z "$(KERNEL_CONFIG_OVERRIDE)" ]; then \
			echo "Overriding kernel config with '$(KERNEL_CONFIG_OVERRIDE)'"; \
			for override_config in $(KERNEL_CONFIG_OVERRIDE); \
				do echo $$override_config >> $(KERNEL_OUT)/.config; done; \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) oldconfig; fi
	$(hide) if [ ! -z "$(KERNEL_CONFIG_OVERRIDE_FILES)" ]; then \
			echo "Overriding kernel config with '$(KERNEL_CONFIG_OVERRIDE_FILES)'"; \
			for override_file in $(KERNEL_CONFIG_OVERRIDE_FILES); \
            	do cat $(TARGET_KERNEL_SOURCE)/arch/$(KERNEL_ARCH)/configs/$$override_file >> $(KERNEL_OUT)/.config; done; \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) KERNEL_COMPRESSION_SUFFIX=$(TARGET_KERNEL_COMPRESSION_SUFFIX) oldconfig; fi
	$(hide) touch $@/build-timestamp

kerneltags: $(KERNEL_CONFIG) | $(KERNEL_OUT)
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) tags

kernelconfig: $(KERNEL_CONFIG) | $(KERNEL_OUT)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) savedefconfig
	cp $(KERNEL_OUT)/defconfig $(TARGET_KERNEL_SOURCE)/arch/$(KERNEL_ARCH)/configs/$(KERNEL_DEFCONFIG)

endif
endif

ifeq ($(PRODUCT_SUPPORT_OTG), y)
KERNEL_CONFIG_OVERRIDE += \
	CONFIG_USB_HOST_NOTIFY=y
endif

ifeq ($(LGE_USB_DIAG_LOCK_TRF), y)
    KERNEL_CONFIG_OVERRIDE += \
    CONFIG_LGE_USB_DIAG_LOCK_TRF=y
endif

ifeq ($(LGE_USB_BC_12), y)
    KERNEL_CONFIG_OVERRIDE += \
    CONFIG_LGE_USB_BC_12_VZW=y
endif
