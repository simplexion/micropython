AMBD_SDK=$(shell pwd)/../../lib/ambd_sdk/
PROJECT_BASE=$(AMBD_SDK)/project/realtek_amebaD_va0_example/GCC-RELEASE/
IMAGE_DIR=$(shell pwd)/image

.PHONY: all
all: $(IMAGE_DIR) $(IMAGE_DIR)/imgtool_flashloader_amebad.bin $(IMAGE_DIR)/km0_boot_all.bin $(IMAGE_DIR)/km4_boot_all.bin $(IMAGE_DIR)/km0_km4_image2.bin

.PHONY: image_package
image_package: $(IMAGE_DIR).tar.gz

$(IMAGE_DIR):
	@rm -rf $(IMAGE_DIR)
	@mkdir -p $(IMAGE_DIR)

$(IMAGE_DIR)/imgtool_flashloader_amebad.bin:
	@cp $(AMBD_SDK)/tools/AmebaD/Image_Tool/imgtool_flashloader_amebad.bin $(IMAGE_DIR)/

$(IMAGE_DIR)/km0_boot_all.bin:
	@make -C lp AMBD_SDK=$(AMBD_SDK)
	@cp $(PROJECT_BASE)/project_lp/asdk/image/km0_boot_all.bin $@
.PHONY: $(IMAGE_DIR)/km0_boot_all.bin

$(IMAGE_DIR)/km4_boot_all.bin:
	@make -C hp AMBD_SDK=$(AMBD_SDK)
	@cp $(PROJECT_BASE)/project_hp/asdk/image/km4_boot_all.bin $@
.PHONY: $(IMAGE_DIR)/km4_boot_all.bin

$(IMAGE_DIR)/km0_km4_image2.bin: $(IMAGE_DIR)/km0_boot_all.bin $(IMAGE_DIR)/km4_boot_all.bin
	@cp $(PROJECT_BASE)/project_hp/asdk/image/km0_km4_image2.bin $@

$(IMAGE_DIR).tar.gz: $(IMAGE_DIR)/km0_boot_all.bin $(IMAGE_DIR)/km4_boot_all.bin $(IMAGE_DIR)/km0_km4_image2.bin
	@rm -f $@
	@tar czf $@ ./image

.PHONY: submodules
submodules:
	make -C hp/micropython submodules

.PHONY: clean
clean:
	@rm -rf $(IMAGE_DIR)
	@rm -f $(IMAGE_DIR).tar.gz
	@make -C lp AMBD_SDK=$(AMBD_SDK) clean
	@make -C hp AMBD_SDK=$(AMBD_SDK) clean
	@make -C hp/micropython AMBD_SDK=$(AMBD_SDK) clean
